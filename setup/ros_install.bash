#!/bin/bash
# time synchronization
echo "Starting to install ROS and associated stuff"
sudo apt-get -y install chrony
sudo apt-get -y install ntpdate

# ROS install
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
sudo apt-get -y update
sudo apt-get -y install ros-kinetic-desktop-full
sudo apt-get -y install ros-kinetic-rosserial-arduino
sudo apt-get -y install ros-kinetic-teleop-twist-keyboard ros-kinetic-hector-sensors-gazebo ros-kinetic-message-to-tf ros-kinetic-hector-sensors-description
sudo rosdep init
rosdep update
echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
source ~/.bashrc

echo "Install Python related stuff"
sudo apt-get -y install python-rosinstall

sudo apt-get -y install python-pip python3-pip
pip3 install --user rospkg catkin_pkg
pip install --user numpy scipy matplotlib ipython jupyter pandas sympy nose

echo "Adding local Python to path"
export PATH="$PATH:$HOME/.local/bin"

echo "ALL DONE!!"
cd ../../..


echo "Now we'll build!"
echo "If the make fails then just restart the terminal and run catkin_make in this directory!"

catkin_make
