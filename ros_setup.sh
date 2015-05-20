##############################################################################################
#Author: Kiran Kumar Lekkala
#Date: 13 April 2015
#Description: Makefile to build and compile the necessary files for generating peer executable
##############################################################################################


sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu wheezy main" > /etc/apt/sources.list.d/ros-latest.list'

wget https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -O - | sudo apt-key add -
sudo apt-get update
sudo apt-get upgrade

sudo apt-get install python-setuptools python-pip python-yaml python-argparse python-distribute python-docutils python-dateutil python-setuptools python-six
sudo pip install rosdep rosinstall_generator wstool rosinstall

###Initializing rosdep

sudo rosdep init
rosdep update

###Installation

mkdir ~/ros_catkin_ws
cd ~/ros_catkin_ws


###Fetch the core packages so we can build them.

rosinstall_generator ros_comm --rosdistro indigo --deps --wet-only --exclude roslisp --tar > indigo-ros_comm-wet.rosinstall
wstool init src indigo-ros_comm-wet.rosinstall


mkdir ~/ros_catkin_ws/external_src
sudo apt-get install checkinstall cmake
sudo sh -c 'echo "deb-src http://mirrordirector.raspbian.org/raspbian/ testing main contrib non-free rpi" >> /etc/apt/sources.list'
sudo apt-get update
