# ROS_odroid_node
## Installation

1. First you need [ROS](http://wiki.ros.org/indigo/Installation) (use kinetic distro)
  * This is easy and is simply a `apt-get` call. Just follow the instructions like a good engineer
2. Also install `rosserial-arduino` using
~~~
$ sudo apt-get install ros-<distro>-rosserial-arduino
~~~
3. Create a `catkin_ws/src` workspace in a convienent directory
~~~
$ mkdir -p ~/fdcl_ros/src
$ cd ~/fdcl_ros
~~~
4. Clone and then build the package
~~~
$ cd ~/fdcl/src
$ git clone https://github.com/fdcl-gwu/ROS_odroid_node.git .
$ cd ../
$ catkin_make
~~~
5. Source the package path files `$ source devel/setup.bash`
6. ....

roslaunch odroid empty_gazebo.launch
roslaunch odroid odroid_gazebo.launch
rosrun rqt_reqconfigure

7. Profit


## Odroid node class implementation for ROS

### Hardware procedure

1. `roslaunch openni2_launch openni2.launch`
- `rosrun odroid sense.py`
- run IMU driver
- run controller node
- Run node on the base station to map


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
- Throttle calibration node: arduino_daq.py
  - Use C implementation not python
  - include #define USE_USBCON in the sketch of arduino (working code in arduino/ADC folder)
  - [Omega manual](https://www.omega.com/manuals/manualpdf/M3598.pdf)
  - [I2C example for arduino ros](http://wiki.ros.org/rosserial_arduino/Tutorials/Measuring%20Temperature)


```
sudo apt-get install ros-kinetic-rosserial-server
rosrun rosserial_server serial_node _port:=/dev/ttyACM0 _baud:=115200
```

## Update versions  
tag v0.3
- Gazebo simulation quad controller integration complete

tag V0.2
- Synchronized message from IMU and Vicon for sensor callback  

tag V0.1
- Initial working node for attitude control on a spherical joint



To get master rosnode communicate with slaves
  - `hostname -I`, to check host name for roscore
  - `export ROS_MASTER_URI=http://<ip_address>:11311/`, master IP setting
  - `export ROS_IP=localhost`, set ROS_IP for the remote
  - `rosrun urg_node urg_node _ip_address:="192.168.0.10"`
  - Use nmap command to debug some of the network communication issues

## TODO:
- set time between odroid and ground station
```
sudo apt-get install chrony
sudo apt-get install ntpdate
sudo ntpdate ntp.ubuntu.com
or
sudo date --set="$(ssh user@host date)"
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

## Some common issues

Check:
- If the compiling error appear on Odroid, check rospkg, catkin_pkg installation. Python distribution change to conda caused the issue. 01/07/2017
- compiling with anaconda python https://gist.github.com/mitmul/538315c68c2069f16f11

### Python issues

  * If you have Anaconda installed, you can simply uncomment the path modifcation in your `~/.bashrc` file
  This will ensure that the system Python is Pin use rather than Anaconda. 

    More explicitly, make sure the following line is commented in `~/.bashrc` and then run `source ~/.bashrc` or open a new terminal window and verify that the system python is running.
    ~~~
    # added by Anaconda3 4.3.0 installer
    # export PATH="/home/shankar/anaconda3/bin:$PATH"
    ~~~
  Ensure you have `catkin_pkg` installed via `pip install --user catkin_pkg`
  * Check which Python you're using by `$ python --version`
  * Install `pip` using the system package manager - `$ sudo apt-get install python-pip python3-pip`

