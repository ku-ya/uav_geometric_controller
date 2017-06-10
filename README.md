# ROS_node for UAV geometric controller

This repository contains all the ROS source code developed for UAV for indooor (MOCA) and simulation (Gazebo) environments.

## Installation

1. First you need [ROS](http://wiki.ros.org/indigo/Installation) (use kinetic distro)
      * This is super easy now.
        We've created a setup script that will install all the necessary packages.
2. Create a workspace in a convienent directory and clone the repo.
This is creating the `catkin_ws`

      ~~~
      $ cd
      $ mkdir -p catkin_ws/src
      $ cd catkin_ws/src
      $ git clone https://github.com/fdcl-gwu/ROS_node.git
      ~~~

3. Run the comprehensive setup script

    ~~~
    $ cd ROS_node/setup
    $ ./ros_install.bash
    ~~~

4. The script will install ROS and all necessary packages.
You can then go to your `catkin_ws` and build as necessary
5. Modify the path
    ~~~
    $ cd ~/catkin_ws
    $ source devel/setup.bash
    ~~~
6. Now you can run the code

## Running the code

There are a variety of possible ways to compile and use the code.
It is already setup to provide simulation capabilities via Gazebo as well as hardware interfaces for use on the vehicles.

### Gazebo simulation environment

To run the code in simulation you use the following launch files:

```
roslaunch odroid empty_gazebo.launch
roslaunch odroid odroid_gazebo.launch
```

This will create a Gazebo simulation and the appropriate nodes for communication.

###  Indoor testing at MOCA

This launch file is setup for indoor testing in the motion capture lab.

```
roslaunch odroid odroid_moca.launch
```

### Hardware procedure
Find IP of odroid on the same network: use [`nmap`](http://security.stackexchange.com/questions/36198/how-to-find-live-hosts-on-my-network)

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


## Odroid node class implementation for ROS

## UAV control

## Gazebo models

### Quadrotor

### Install

```
TODO: scripts
```

### System dependencies

### Networking

To get master rosnode communicate with slaves
  - `hostname -I`, to check host name for roscore
  - `export ROS_MASTER_URI=http://<ip_address>:11311`
  - `export ROS_IP=<ip_address>`
  - `rosrun urg_node urg_node _ip_address:="192.168.0.10"`
  - Use nmap command to debug some of the network communication issues

## Update versions

tag v0.4
- First successful flight test

tag v0.3
- Gazebo simulation quad controller integration complete

tag V0.2
- Synchronized message from IMU and Vicon for sensor callback

tag V0.1
- Initial working node for attitude control on a spherical joint

## TODO:
- set time between odroid and ground station
```
sudo apt-get install chrony
sudo apt-get install ntpdate
sudo ntpdate ntp.ubuntu.com
or
THIS WORKS: sudo date --set="$(ssh user@host date)"
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
    # export PATH="/home/<username>/anaconda3/bin:$PATH"
    ~~~
  * Install `pip` using the system package manager - `$ sudo apt-get install python-pip python3-pip`
  * Ensure you have `catkin_pkg` and `ros_pkg` installed via `pip install --user catkin_pkg ros_pkg`
  * Check which Python you're using by `$ python --version`. It should not point to Anaconda after you modify the `.bashrc` file. If it still does then you need to open a new terminal window to load the new configuration.

### `nmap`

This a command-line tool to scan your local network.
To find IP addresses of other devices on your local subnet use the following:

~~~
$ nmap -sP 192.168.2.1/24
~~~

This command assumes the local network is `192.168.2.*`.
If this is not the case you need to modify the command by finding the local gateway using `ifconfig`
