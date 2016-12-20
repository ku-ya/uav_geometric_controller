# ROS_odroid_node
Odroid node class implementation for ROS

- IMU subscriber moved out of code as callback so that it can accept different IMU drivers.
- Controller callback
- Vicon subscriber
- Fake imu sensor data using python code
- VN100 IMU driver for ROS has been tested with kinetic [link](https://github.com/KumarRobotics/imu_vn_100)

Check gazebo IMU sensor plugin (https://github.com/alessandrosettimi/test_imu_sensor)
