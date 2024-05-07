export ROS_LOG_DIR=/workspace/logs

ros2 launch foxglove_bridge foxglove_bridge_launch.xml &

ros2 launch tareeq tareeqav_launch.py

