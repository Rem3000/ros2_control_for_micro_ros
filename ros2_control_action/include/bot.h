#ifndef __BOT_H__
#define __BOT_H__
#include "Arduino.h"
#include "bot_config.h"
#include <Esp32PcntEncoder.h> 
#include <micro_ros_platformio.h>
#include <Wire.h>
#include <stdint.h>
#include <PidController.h>
#include <Wire.h>
#include <TimeLib.h> 

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rosidl_runtime_c/string_functions.h>
#include <rosidl_runtime_c/primitives_sequence_functions.h>
#include <micro_ros_utilities/type_utilities.h>
#include <micro_ros_utilities/string_utilities.h>
#include <std_msgs/msg/header.h>
#include <sensor_msgs/msg/joint_state.h>
#include <control_msgs/action/follow_joint_trajectory.h>
#include <rmw/qos_profiles.h>



#include <Adafruit_NeoPixel.h>
#define LED_PIN    48    // RGB LED 連接的腳位
#define NUM_PIXELS 1     // 通常只有一顆內建的 RGB LED
extern Adafruit_NeoPixel pixels;

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)) {} else {}}
#define EXECUTE_EVERY_N_MS(MS, X)          \
    do                                     \
    {                                      \
        static volatile int64_t init = -1; \
        if (init == -1)                    \
        {                                  \
            init = millis();               \
        }                                  \
        if (millis() - init > MS)          \
        {                                  \
            X;                             \
            init = millis();               \
        }                                  \
    } while (0);

void interpolate_trajectory(unsigned long current_time);
void callback_publisher(rcl_timer_t *timer, int64_t last_call_time);
void loop_bot_control();
void initializeHardware();
void loop_bot_transport();
bool create_bot_transport();
bool destory_bot_transport();
void updateMotor(float speed, uint8_t index);
void RGB(uint8_t r, uint8_t g, uint8_t b, uint8_t Brightness=10, bool Brightness_on = false);
float encoder_to_rad();
float encoder_to_rads(); 
void error_loop();

// Action callback functions
rcl_ret_t handle_goal(rclc_action_goal_handle_t *goal_handle, void *context);
bool handle_cancel(rclc_action_goal_handle_t *goal_handle, void *context); 
#endif