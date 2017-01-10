# ROS_odroid_node
Odroid node class implementation for ROS

- IMU subscriber moved out of code as callback so that it can accept different IMU drivers.
- Controller callback
  - member function inputs has been reduced
  - calculates forces and throttle values
- Vicon subscriber
- Fake imu sensor data using python code
- VN100 IMU driver for ROS has been tested with kinetic [link](https://github.com/KumarRobotics/imu_vn_100)
  - If you get /imu: VN: Permission denied, On line 217 in src/imu_vn_100.cpp, try changing `BINARY_ASYNC_MODE_SERIAL_2` to `BINARY_ASYNC_MODE_SERIAL_1`
- Hector quadrotor repository for simulation testing
- Dynamic reconfiguration for gains and value prints on console
- I2C command function for motor control
  - install i2c-dev for linux


To get master rosnode communicate with slaves
  - `hostname -I`, to check host name for roscore
  - `export ROS_MASTER_URI=http://10.0.1.xxx:11311/`, master IP setting
  - `export ROS_IP=10.0.1.xxx`, set ROS_IP for the remote
  - `rosrun urg_node urg_node _ip_address:="192.168.0.10"`

TODO:
- Hardware test
- Check gazebo IMU sensor plugin (https://github.com/alessandrosettimi/test_imu_sensor)
- Clear unnecessary header files and functions
- publisher in subscriber callback function and make a node handle within class
- Run vicon_bridge [ROS wiki](http://wiki.ros.org/vicon_bridge) [github](https://github.com/ethz-asl/vicon_bridge)

Check:
- If the compiling error appear on Odroid, check rospkg, catkin_pkg installation. Python distribution change to conda caused the issue. 01/07/2017
