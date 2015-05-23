##############################################################################################
#Author: Kiran Kumar Lekkala
#Date: 13 April 2015
#Description: Makefile to build and compile the necessary files for generating peer executable
##############################################################################################



################################################################Resolving Dependencies#########################################################

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

###Build libconsole-bridge-dev

cd ~/ros_catkin_ws/external_src
sudo apt-get build-dep console-bridge
apt-get source -b console-bridge
sudo dpkg -i libconsole-bridge0.2_*.deb libconsole-bridge-dev_*.deb


###Build liblz4-dev

cd ~/ros_catkin_ws/external_src
apt-get source -b lz4
sudo dpkg -i liblz4-*.deb

###liburdfdom-headers-dev

cd ~/ros_catkin_ws/external_src
git clone https://github.com/ros/urdfdom_headers.git
cd urdfdom_headers
cmake .
sudo checkinstall make install
	
###liburdfdom-dev:

cd ~/ros_catkin_ws/external_src
sudo apt-get install libboost-test-dev libtinyxml-dev
git clone https://github.com/ros/urdfdom.git
cd urdfdom
cmake .
sudo checkinstall make install

###collada-dom-dev:

cd ~/ros_catkin_ws/external_src
sudo apt-get install libboost-filesystem-dev libxml2-dev
wget http://downloads.sourceforge.net/project/collada-dom/Collada%20DOM/Collada%20DOM%202.4/collada-dom-2.4.0.tgz
tar -xzf collada-dom-2.4.0.tgz
cd collada-dom-2.4.0
cmake .
sudo checkinstall make install


####################Making the cat-kin workspace#######################

cd ~/ros_catkin_ws
rosdep install --from-paths src --ignore-src --rosdistro indigo -y -r --os=debian:wheezy


sudo ./src/catkin/bin/catkin_make_isolated --install -DCMAKE_BUILD_TYPE=Release --install-space /opt/ros/indigo

###################Adding released packages##########################

cd ~/ros_catkin_ws
rosinstall_generator ros_comm ros_control joystick_drivers --rosdistro indigo --deps --wet-only --exclude roslisp --tar > indigo-custom_ros.rosinstall

wstool merge -t src indigo-custom_ros.rosinstall
wstool update -t src


rosdep install --from-paths src --ignore-src --rosdistro indigo -y -r --os=debian:wheezy

sudo ./src/catkin/bin/catkin_make_isolated --install -DCMAKE_BUILD_TYPE=Release --install-space /opt/ros/indigo
