# ROS_odroid_node
## Odroid node class implementation for ROS

- IMU subscriber moved out of code as callback so that it can accept different IMU drivers.
- Controller callback
  - member function inputs has been reduced
  - calculates forces and throttle values
- Vicon subscriber
  - Uses Vicon_bridge package for communication [ROS wiki](http://wiki.ros.org/vicon_bridge) [github](https://github.com/ethz-asl/vicon_bridge)
- Fake imu sensor data using python code
- VN100 IMU driver for ROS has been tested with kinetic [link](https://github.com/KumarRobotics/imu_vn_100)
  - If you get /imu: VN: Permission denied, On line 217 in src/imu_vn_100.cpp, try changing `BINARY_ASYNC_MODE_SERIAL_2` to `BINARY_ASYNC_MODE_SERIAL_1`
- Hector quadrotor repository for simulation testing
- Dynamic reconfiguration for gains and value prints on console
- I2C command function for motor control
  - install i2c-dev for linux

## Update versions
tag V0.2
- Synchronized message from IMU and Vicon for sensor callback
tag V0.1
- Initial working node for attitude control on a spherical joint


To get master rosnode communicate with slaves
  - `hostname -I`, to check host name for roscore
  - `export ROS_MASTER_URI=http://hostname:11311/`, master IP setting
  - `export ROS_IP=localhost`, set ROS_IP for the remote
  - `rosrun urg_node urg_node _ip_address:="192.168.0.10"`
  - Use nmap command to debug some of the network communication issues

## TODO:
- set time between odroid and ground station
  - ```
    sudo apt-get install chrony
    sudo apt-get install ntpdate
    sudo ntpdate ntp.ubuntu.com
    ```
  - [link to roswiki](http://wiki.ros.org/turtlebot/Tutorials/indigo/Network%20Configuration)
- Hardware test
 - Record the experiment with camera rosbag
 - test vicon_bridge with position controller
 - bring USB splitter
- Check gazebo IMU sensor plugin (https://github.com/alessandrosettimi/test_imu_sensor)
- Clear unnecessary header files and functions
- publisher in subscriber callback function and make a node handle within class

20170111 Testing

1. test attitude controller with vicon
2.
3. test position controller

*Make sure to ROSBAG all the test data!!!*

Check:
- If the compiling error appear on Odroid, check rospkg, catkin_pkg installation. Python distribution change to conda caused the issue. 01/07/2017
- compiling with anaconda python https://gist.github.com/mitmul/538315c68c2069f16f11
