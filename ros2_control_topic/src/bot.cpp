#include "bot.h"


/*==================MicroROS消息============================*/
sensor_msgs__msg__JointState joint_msg;
trajectory_msgs__msg__JointTrajectory trajectory_msg; // 軌跡訊息接收

/*==================MicroROS配置============================*/
static micro_ros_utilities_memory_conf_t conf = {0}; // 配置 Micro-ROS 库中的静态的内存管理器
// 机器人配置信息中键值对（key-value）的最大容量
/*==================MicroROS订阅发布者服务========================*/
rcl_publisher_t joint_state_publisher;
rcl_subscription_t trajectory_subscriber; // 軌跡訊息訂閱者

/*==================MicroROS相关执行器&节点===================*/
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;


PidController vel_pid[1];     // 速度 PID

// 軌跡跟隨相關變數
struct TrajectoryState {
  float target_position;    // 目標位置 (rad)
  float target_velocity;    // 目標速度 (rad/s)
  float target_acceleration; // 目標加速度 (rad/s²)
  unsigned long target_time; // 目標時間 (微秒)
  bool has_trajectory;      // 是否有有效軌跡
} trajectory_state;

trajectory_msgs__msg__JointTrajectory current_trajectory; // 當前軌跡
unsigned long trajectory_start_time = 0; // 軌跡開始時間
int current_trajectory_point = 0;        // 當前軌跡點索引

Esp32PcntEncoder encoders[1]; // ESP32 PCNT 
uint8_t pwm = 0;
Adafruit_NeoPixel pixels(NUM_PIXELS, LED_PIN, NEO_GRB + NEO_KHZ800);
enum states
{
  WAITING_AGENT,
  AGENT_AVAILABLE,
  AGENT_CONNECTED,
  AGENT_DISCONNECTED
} state;

#define vel_P 0.6 
#define vel_I 0.1 
#define vel_D 0.3  

// 前饋控制增益參數
#define VELOCITY_FEEDFORWARD_GAIN 0.3    // 速度前饋增益
#define ACCELERATION_FEEDFORWARD_GAIN 0.03  // 加速度前饋增益




// 軌跡訊息回調函數
void trajectory_callback(const void *msgin) {
  const trajectory_msgs__msg__JointTrajectory *msg = (const trajectory_msgs__msg__JointTrajectory *)msgin;
  
  // 檢查是否有軌跡點
  if (msg->points.size > 0) {
    // 停止當前軌跡（如果有的話）
    trajectory_state.has_trajectory = false;
    delay(100);  // 短暫停止
    
    // 重置PID
    vel_pid[0].reset();
    vel_pid[0].update_pid(vel_P, vel_I, vel_D);
    vel_pid[0].out_limit(-250, 250);
    
    current_trajectory = *msg;
    
    // // 調試：打印接收到的軌跡信息
    // Serial2.println("=== 接收軌跡 ===");
    // Serial2.print("軌跡點數: ");
    // Serial2.println(msg->points.size);
    // for (int i = 0; i < msg->points.size; i++) {
    //   Serial2.print("點 ");
    //   Serial2.print(i);
    //   Serial2.print(": 位置=");
    //   Serial2.print(msg->points.data[i].positions.data[0]);
    //   Serial2.print(", 時間=");
    //   Serial2.println(msg->points.data[i].time_from_start.sec);
    // }
    
    // 設定軌跡開始時間
    trajectory_start_time = micros();
    current_trajectory_point = 0;
    trajectory_state.has_trajectory = true;
  } else {
    trajectory_state.has_trajectory = false;
  }
}

void callback_publisher(rcl_timer_t *timer, int64_t last_call_time){
  static float vel[1], pos[1];
  RCLC_UNUSED(last_call_time);

  if (timer != NULL)
  {
    int64_t stamp = rmw_uros_epoch_millis();
    joint_msg.header.stamp.sec = static_cast<int32_t>(stamp / 1000);              // 秒部分
    joint_msg.header.stamp.nanosec = static_cast<uint32_t>((stamp % 1000) * 1e6); // 纳秒部分
    
    // 獲取實際的位置和速度數據
    pos[0] = encoder_to_rad();      // 當前位置 (rad)
    vel[0] = encoder_to_rads();     // 當前角速度 (rad/s)
    
    joint_msg.position.data[0] = pos[0];
    joint_msg.velocity.data[0] = vel[0];
    joint_msg.effort.data[0] = 0.0; 

    RCSOFTCHECK(rcl_publish(&joint_state_publisher, &joint_msg, NULL));
  }
}

void loop_bot_control()
{
  float out_motor_speed[1];
  static uint64_t last_update_info_time = millis();
  static uint64_t last_debug_time = millis();
  
  // 更新軌跡插值
  interpolate_trajectory(micros());
  
  // 獲取當前實際狀態
  float current_position = encoder_to_rad();
  float current_velocity = encoder_to_rads();
  
  if (trajectory_state.has_trajectory) {
    // 檢查是否到達最終目標位置（只在軌跡時間結束後檢查）
    float position_error = abs(trajectory_state.target_position - current_position);
    
    // 計算期望速度（位置誤差轉換為期望速度）
    float position_error_signed = trajectory_state.target_position - current_position;
    float desired_velocity = position_error_signed * 0.8; // 位置誤差到速度的轉換係數
    
    // 加上軌跡的期望速度
    desired_velocity += trajectory_state.target_velocity * VELOCITY_FEEDFORWARD_GAIN;
    
    // 速度PID控制（控制實際速度跟隨期望速度）
    vel_pid[0].update_target(desired_velocity);
    float pid_output = vel_pid[0].update(current_velocity);
    
    // 加上加速度前饋
    float feedforward = trajectory_state.target_acceleration * ACCELERATION_FEEDFORWARD_GAIN;
    // 組合控制輸出
    out_motor_speed[0] = pid_output + feedforward;
    
    // 限制輸出範圍，防止過沖
    out_motor_speed[0] = constrain(out_motor_speed[0], -250, 250);
    
  } else {
    // 沒有軌跡時停止
    out_motor_speed[0] = 0;
  }
  
  updateMotor(out_motor_speed[0], 0);
}

void initializeHardware(){
  
  
  encoders[0].init(0, 13, 14);  // 左前

  // 初始化軌跡狀態
  trajectory_state.target_position = 0.0;
  trajectory_state.target_velocity = 0.0;
  trajectory_state.target_acceleration = 0.0;
  trajectory_state.has_trajectory = false;

  // 3.设置PID
  vel_pid[0].update_target(0.0);
  vel_pid[0].update_pid(vel_P, vel_I, vel_D);
  vel_pid[0].out_limit(-220, 220);  // 降低最大輸出
  


  // 設定馬達引腳模式
  pinMode(one_1, OUTPUT);
  pinMode(one_2, OUTPUT);
  pinMode(one_P, OUTPUT);
  pixels.begin();           // 初始化 NeoPixel
  pixels.setBrightness(10); // 亮度設定 (0~255)
  pixels.setPixelColor(0, pixels.Color(255, 255, 255));
  pixels.show();
}
void loop_bot_transport(){
    // 處理狀態機
    switch (state)
    {
    case WAITING_AGENT:
      // 每隔 2 秒 ping Agent
      EXECUTE_EVERY_N_MS(2000,
                         state = (RMW_RET_OK == rmw_uros_ping_agent(100, 5)) ? AGENT_AVAILABLE : WAITING_AGENT;);
      RGB(255, 0, 0, 50, true); // red閃爍
      break;

    case AGENT_AVAILABLE:
      // 嘗試建立傳輸（建立 ROS node、publisher/subscriber 等）
      if (create_bot_transport())
      {
        state = AGENT_CONNECTED;
      }
      else
      {
        state = WAITING_AGENT;
        destory_bot_transport(); // 清掉失敗建立的資源
      }
      break;

    case AGENT_CONNECTED:
      // 定時 ping 檢查 Agent 是否還在線
      EXECUTE_EVERY_N_MS(2000, if (RMW_RET_OK != rmw_uros_ping_agent(100, 5)) { state = AGENT_DISCONNECTED; });

      if (state == AGENT_CONNECTED)
      {
        // 如果還沒同步時間，嘗試同步
        if (!rmw_uros_epoch_synchronized())
        {
          RCSOFTCHECK(rmw_uros_sync_session(1000));
          if (rmw_uros_epoch_synchronized())
          {
            setTime(rmw_uros_epoch_millis() / 1000 + SECS_PER_HOUR * 8);
          }
        }
        RGB(50, 50, 50, 50, true); // 閃爍
        // 處理 ROS callback（non-blocking）
        RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
      }
      break;

    case AGENT_DISCONNECTED:
      destory_bot_transport(); // 釋放原本的 ROS 資源
      state = WAITING_AGENT;   // 回到等待 agent 的狀態
      break;

    default:
      state = WAITING_AGENT;
      break;
    }

    if (state != AGENT_CONNECTED)
    {
      delay(10); // idle 時稍作延遲以節省資源
    }
  
}
bool create_bot_transport(){
  joint_msg.header.frame_id = micro_ros_string_utilities_set(joint_msg.header.frame_id, "base_link");
  const unsigned int timer_timeout = 10;
  delay(500);
  // 默认的内存分配器 allocator
  allocator = rcl_get_default_allocator();
  // RCSOFTCHECK 是一个宏定义，用于检查执行函数的返回值是否出错，如果出错，则会打印错误信息并退出程序。
  // 调用 rclc_support_init 函数初始化 ROS 2 运行时的支持库，传入 allocator
  RCSOFTCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  // 调用 rclc_node_init_default 函数初始化 ROS 2 节点，传入节点名称、命名空间和支持库
  RCSOFTCHECK(rclc_node_init_default(&node, "esp32_", "", &support));
  // 簡化記憶體配置 - 只設定必要的參數
  conf.max_basic_type_sequence_capacity = 1; // 1個數值陣列 (position, velocity, effort)
  
  // 為軌跡訊息配置記憶體
  micro_ros_utilities_memory_conf_t trajectory_conf = {0};
  trajectory_conf.max_basic_type_sequence_capacity = 5; // 軌跡點數量
  trajectory_conf.max_ros2_type_sequence_capacity = 5;  // 軌跡點陣列
  micro_ros_utilities_create_message_memory(
      ROSIDL_GET_MSG_TYPE_SUPPORT(trajectory_msgs, msg, JointTrajectory),
      &trajectory_msg,
      trajectory_conf);
  
  RCSOFTCHECK(rclc_publisher_init_best_effort(
      &joint_state_publisher,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
      "/joint_states"));
  micro_ros_utilities_memory_rule_t rules[] = {
      {"frame_id",20},
      {"name", 1},      // 1個關節名稱
      {"position", 1},  // 1個位置值
      {"velocity", 1},  // 1個速度值
      {"effort", 1}};   // 1個力矩值
  conf.rules = rules;
  conf.n_rules = sizeof(rules) / sizeof(rules[0]);
  micro_ros_utilities_create_message_memory(
      ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
      &joint_msg,
      conf);


  joint_msg.name.size = 1;
  joint_msg.name.data[0] = micro_ros_string_utilities_set(joint_msg.name.data[0], "joint");
  joint_msg.position.size = 1;
  joint_msg.velocity.size = 1;
  joint_msg.effort.size = 1;
  
  // 創建軌跡訂閱者
  RCSOFTCHECK(rclc_subscription_init_default(
      &trajectory_subscriber,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(trajectory_msgs, msg, JointTrajectory),
      "/joint_trajectory_controller/joint_trajectory"));

  // 調用 rclc_timer_init_default 函數初始化 ROS 2 定時器，傳入支持庫、定時器周期和回調函數
  RCSOFTCHECK(rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(timer_timeout), callback_publisher));
  // 調用 rclc_executor_init 函數初始化 ROS 2 執行器，傳入支持庫、執行器線程數和內存分配器
  RCSOFTCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator)); // 改為 2 個處理程序（timer + subscription）
  
  // 將軌跡訂閱者添加到執行器中
  RCSOFTCHECK(rclc_executor_add_subscription(&executor, &trajectory_subscriber, &trajectory_msg, &trajectory_callback, ON_NEW_DATA));

  // 調用 rclc_executor_add_timer 函數將定時器添加到執行器中，傳入執行器和定時器。
  RCSOFTCHECK(rclc_executor_add_timer(&executor, &timer));
  return true;
}

// 用于销毁 ROS 2 节点中的一些资源
bool destory_bot_transport(){
  // 获取 ROS 2 上下文中的 RMW 上下文，并将其赋值给 rmw_context 变量。
  rmw_context_t *rmw_context = rcl_context_get_rmw_context(&support.context);
  // 设置 ROS 2 上下文中的 RMW 上下文的实体销毁会话超时时间为 0，这意味着实体销毁操作将立即返回，而不是等待超时
  (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);
  // RCSOFTCHECK 是一个宏定义，用于检查执行函数的返回值是否出错，如果出错，则会打印错误信息并退出程序。
  // 用於銷毀一個 ROS 2 發布者（Publisher）
  RCSOFTCHECK(rcl_publisher_fini(&joint_state_publisher, &node));
  // 用於銷毀軌跡訂閱者
  RCSOFTCHECK(rcl_subscription_fini(&trajectory_subscriber, &node));
  
  // 清理訊息記憶體 - 避免記憶體洩漏
  rosidl_runtime_c__String__Sequence__fini(&joint_msg.name);
  rosidl_runtime_c__double__Sequence__fini(&joint_msg.position);
  rosidl_runtime_c__double__Sequence__fini(&joint_msg.velocity);
  rosidl_runtime_c__double__Sequence__fini(&joint_msg.effort);
  
  // 清理軌跡訊息記憶體
  trajectory_msgs__msg__JointTrajectory__fini(&trajectory_msg);
  

  RCSOFTCHECK(rcl_timer_fini(&timer));
  RCSOFTCHECK(rclc_executor_fini(&executor));
  RCSOFTCHECK(rcl_node_fini(&node));
  rclc_support_fini(&support);
  return true;
}
uint8_t light = 0;
bool updown = false;
void RGB(uint8_t r, uint8_t g, uint8_t b, uint8_t Brightness, bool Brightness_on){
  // RGB LED 亮度控制
  // r, g, b: RGB LED 的顏色值 (0~255)
  // Brightness: LED 亮度 (0~255)
  // Brightness_on: 是否開啟亮度控制 (true/false)

  // 設定亮度
  // 如果 Brightness_on 為 true，則設定亮度為指定值
  // 否則，將亮度設為 0 (關閉亮度)
  // 設定 RGB LED 顏色

  if (Brightness_on)
  {
    if (updown)
    {
      light--;
      if (light == 0)
      {
        updown = false;
      }
    }
    else
    {
      light++;
      if (light == Brightness)
      {
        updown = true;
      }
    }
    pixels.setBrightness(light);
  }
  else
  {
    pixels.setBrightness(10);
  }

  pixels.setPixelColor(0, pixels.Color(r, g, b));
  pixels.show();
}
void updateMotor(float speed, uint8_t index){

  pwm = abs(speed);

  switch (index)
  {
  case 0:
    if (speed > 0)
    {
      digitalWrite(one_1, HIGH);
      digitalWrite(one_2, LOW);
    }
    else if (speed < 0)
    {
      digitalWrite(one_1, LOW);
      digitalWrite(one_2, HIGH);
    }
    else
    {
      digitalWrite(one_1, HIGH);
      digitalWrite(one_2, HIGH);
    }
    analogWrite(one_P, pwm);
    break;
}}
float encoder_to_rad(){
  return encoders[0].getTicks() * 0.0083733333; //公式(PI*2/750);
}
float encoder_to_rads(){//rad/s
  static long last_ticks = 0;
  static unsigned long last_time = 0;
  
  long current_ticks = encoders[0].getTicks();
  unsigned long current_time = micros();
  
  // 第一次調用時初始化
  if (last_time == 0) {
    last_ticks = current_ticks;
    last_time = current_time;
    return 0.0;
  }
  
  // 計算時間差（轉換為秒）
  float dt = (current_time - last_time) * 0.000001; // 微秒轉秒

  // 避免除零錯誤
  if (dt <= 0) {
    return 0.0;
  }
  
  // 計算編碼器變化量
  long tick_diff = current_ticks - last_ticks;
  
  // 計算角速度 (rad/s)
  float angular_velocity = (tick_diff * 0.0083733333) / dt;//公式(tick_diff*PI*2/750)/dt

  // 更新上次的值
  last_ticks = current_ticks;
  last_time = current_time;
  
  return angular_velocity;
}
// 軌跡插值函數
void interpolate_trajectory(unsigned long current_time) {
  if (!trajectory_state.has_trajectory || current_trajectory.points.size == 0) {
    return;
  }
  
  // 計算相對時間（秒）
  float elapsed_time = (current_time - trajectory_start_time) / 1000000.0;

  // 尋找當前時間對應的軌跡段
  int target_point = -1;
  for (int i = 0; i < current_trajectory.points.size; i++) {
    float point_time = current_trajectory.points.data[i].time_from_start.sec + 
                      current_trajectory.points.data[i].time_from_start.nanosec / 1e9;
    if (elapsed_time <= point_time) {
      target_point = i;
      break;
    }
  }
  
  // // 調試信息：顯示軌跡執行狀態
  // static unsigned long last_debug_time = 0;
  // if (millis() - last_debug_time > 500) {  // 每500ms打印一次
  //   Serial2.print("時間: ");
  //   Serial2.print(elapsed_time);
  //   Serial2.print(" | 目標點: ");
  //   Serial2.print(target_point);
  //   Serial2.print(" | 總點數: ");
  //   Serial2.print(current_trajectory.points.size);
  //   if (target_point >= 0 && target_point < current_trajectory.points.size) {
  //     Serial2.print(" | 目標位置: ");
  //     Serial2.print(current_trajectory.points.data[target_point].positions.data[0]);
  //   }
  //   Serial2.println("");
  //   last_debug_time = millis();
  // }
  if (target_point == -1) {
    // 軌跡時間結束，設定最終目標位置
    target_point = current_trajectory.points.size - 1;
    trajectory_msgs__msg__JointTrajectoryPoint *final_point = &current_trajectory.points.data[target_point];
    
    // 設定最終位置並停止
    if (final_point->positions.size > 0) {
      trajectory_state.target_position = final_point->positions.data[0];
    }
    trajectory_state.target_velocity = 0.0;      // 停止速度
    trajectory_state.target_acceleration = 0.0; // 停止加速度
    
    // 檢查是否到達最終位置
    float current_pos = encoder_to_rad();
    float final_error = abs(trajectory_state.target_position - current_pos);
    
    static unsigned long final_check_time = 0;
    if (final_check_time == 0) {
      final_check_time = millis();
    }
    
    // 如果到達最終位置或超時，停止軌跡
    if (final_error < 0.1 || (millis() - final_check_time > 3000)) {
      trajectory_state.has_trajectory = false;
      final_check_time = 0;
    }
    
    return;
  }
  trajectory_msgs__msg__JointTrajectoryPoint *current_point = &current_trajectory.points.data[target_point];
  
  
  if (target_point == 0) {
    // 第一個點，直接使用
    if (current_point->positions.size > 0) {
      trajectory_state.target_position = current_point->positions.data[0];
    }
    if (current_point->velocities.size > 0) {
      trajectory_state.target_velocity = current_point->velocities.data[0];
    } else {
      trajectory_state.target_velocity = 0.0;
    }
    if (current_point->accelerations.size > 0) {
      trajectory_state.target_acceleration = current_point->accelerations.data[0];
    } else {
      trajectory_state.target_acceleration = 0.0;
    }
  } else {
    // 在兩點之間插值
    trajectory_msgs__msg__JointTrajectoryPoint *prev_point = &current_trajectory.points.data[target_point - 1];
    
    float prev_time = prev_point->time_from_start.sec + prev_point->time_from_start.nanosec / 1e9;
    float curr_time = current_point->time_from_start.sec + current_point->time_from_start.nanosec / 1e9;
    
    // 插值係數
    float alpha = (elapsed_time - prev_time) / (curr_time - prev_time);
    alpha = constrain(alpha, 0.0, 1.0);
    
    // 位置插值
    if (prev_point->positions.size > 0 && current_point->positions.size > 0) {
      trajectory_state.target_position = prev_point->positions.data[0] + 
                                       alpha * (current_point->positions.data[0] - prev_point->positions.data[0]);
    }
    
    // 速度插值
    if (prev_point->velocities.size > 0 && current_point->velocities.size > 0) {
      trajectory_state.target_velocity = prev_point->velocities.data[0] + 
                                       alpha * (current_point->velocities.data[0] - prev_point->velocities.data[0]);
    } else {
      trajectory_state.target_velocity = 0.0;
    }
    
    // 加速度插值
    if (prev_point->accelerations.size > 0 && current_point->accelerations.size > 0) {
      trajectory_state.target_acceleration = prev_point->accelerations.data[0] + 
                                           alpha * (current_point->accelerations.data[0] - prev_point->accelerations.data[0]);
    } else {
      trajectory_state.target_acceleration = 0.0;
    }
  }
}