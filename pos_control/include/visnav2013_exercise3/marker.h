/*
 * marker.h
 *
 *  Created on: Apr 18, 2012
 *      Author: Nikolas Engelhard
 */

#ifndef MARKER_H_
#define MARKER_H_


#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <Eigen/Core>

struct drone_marker {

 void publish_markers();

 drone_marker(){
  pub_markers = nh_.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 0 );
 }


 void addMarkerPose(const tf::StampedTransform& trafo){
  trafos.push_back(trafo);
 }

 void init(){trafos.clear();}

 std::vector<tf::StampedTransform> trafos;

 ros::NodeHandle nh_;
 ros::Publisher pub_markers;
 tf::TransformBroadcaster br;

};


struct EKF_marker {

 void publish_last_n_states(int n = -1);

 EKF_marker(){
   pub_markers = nh_.advertise<visualization_msgs::Marker>("visualization_marker", 1000);
  }

 void addFilterState(Eigen::Vector3f mu, Eigen::Matrix3f sigma, float height = 0){
  mus.push_back(mu); sigmas.push_back(sigma); heights.push_back(height);
 }

 void init(){mus.clear(); sigmas.clear(); heights.clear();}


 ros::NodeHandle nh_;
 ros::Publisher pub_markers;

private:
 std::vector<Eigen::Vector3f> mus;
 std::vector<Eigen::Matrix3f> sigmas;
 std::vector<float> heights;

};



#endif /* MARKER_H_ */
