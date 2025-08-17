# ros2_control_for_micro_ros
#Please use platform
action Net yet finished
if u want to use topic 
ros2 topic pub --once /joint_trajectory_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory '{
  joint_names: ["front_left_wheel_joint"],
  points: [
    {
      positions: [1.57],
      velocities: [0.0],
      accelerations: [0.0],
      time_from_start: {sec: 5, nanosec: 0}
    },
    {
      positions: [3.14],
      velocities: [0.0],
      accelerations: [0.0],
      time_from_start: {sec: 10, nanosec: 0}
    }
  ]
}'
-----------------------------------
Q&A
Q If /joint_state has no information

A This is a problem with best_effect. After modifying the configuration file, delete .pio/libdeps/4d_systems_esp32s3_gen4_r8n16/micro_ros_platformio/libmicroros.

Click ctrl+alt+b to resolve.
open the .pio/libdeps/4d_systems_esp32s3_gen4_r8n16/micro_ros_platformio/metas/colcon.meta
Paste 
{
    "names": {
        "rmw_microxrcedds": {
            "cmake-args": [
                "-DRMW_UXRCE_MAX_NODES=1",
                "-DRMW_UXRCE_MAX_PUBLISHERS=10",
                "-DRMW_UXRCE_MAX_SUBSCRIPTIONS=10",
                "-DRMW_UXRCE_MAX_SERVICES=6",
                "-DRMW_UXRCE_MAX_CLIENTS=1",
                "-DRMW_UXRCE_MAX_HISTORY=1",
                "-DRMW_UXRCE_TRANSPORT=custom"
            ]
        },
        "microxrcedds_client":{
                "cmake-args": [
                    "-DUCLIENT_CUSTOM_TRANSPORT_MTU=1024",
                ]
        }
    }
}
