##############################################################################################
#Author: Kiran Kumar Lekkala
#Date: 13 April 2015
#Description: Makefile to build and compile the necessary files for generating peer executable
##############################################################################################


sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu wheezy main" > /etc/apt/sources.list.d/ros-latest.list'

wget https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -O - | sudo apt-key add -
sudo apt-get update
sudo apt-get upgrade


