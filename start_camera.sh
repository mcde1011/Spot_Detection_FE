#!/bin/bash

# Start camera driver
gnome-terminal -- bash -c "source /opt/ros/humble/setup.bash; source install/setup.bash; ros2 launch ricoh_theta_ros full_stack.launch.py device:=cuda:0 imgsz:=640 conf:=0.25 iou:=0.45 period_s:=1.0 log_timing_level:=info"

# Start transformation to occupancy map
gnome-terminal -- bash -c "source /opt/ros/humble/setup.bash; source install/setup.bash; ros2 launch transform_to_map transform_to_map_launch.py"
