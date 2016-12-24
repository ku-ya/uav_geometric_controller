# ROS_odroid_node
Odroid node class implementation for ROS

- IMU subscriber moved out of code as callback so that it can accept different IMU drivers.
- Controller callback
  - member function inputs has been reduced
  - calculates forces and throttle values
- Vicon subscriber
- Fake imu sensor data using python code
- VN100 IMU driver for ROS has been tested with kinetic [link](https://github.com/KumarRobotics/imu_vn_100)
- Hector quadrotor repository for simulation testing
- Dynamic reconfiguration for gains and value prints on console
- I2C command function for motor control
  - install i2c-dev for linux

TODO:
- Hardware test
- Check gazebo IMU sensor plugin (https://github.com/alessandrosettimi/test_imu_sensor)
- Clear unnecessary header files and functions
