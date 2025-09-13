# HapticDevice

# 2025 version with Waveshare RoArm M1

Add user access to ttyUSB0:
sudo usermod -a -G dialout $USER


https://www.waveshare.com/wiki/RoArm-M1_Tutorial_V:_ROS2_Serial_Communication_Node


# Starting the robot:
ros2 launch roarm roarm.launch.py
ros2 run serial_ctrl serial_ctrl