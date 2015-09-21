##############################################################################################
#Author: Kiran Kumar Lekkala
#Date: 13 April 2015
#Description: Makefile to build and compile the necessary files for generating peer executable
##############################################################################################

mkdir ~/ fuerte_workspace

##Insert the following line at the end
export ROS_PACKAGE_PATH = $ROS_PACKAGE_PATH :~/
fuerte_workspace


cd ~/ fuerte_workspace
git clone https :// github . com / tum - vision /
ardrone_autonomy . git

###Build the driver
cd ardrone_autonomy
./ build_sdk . sh
rosmake
