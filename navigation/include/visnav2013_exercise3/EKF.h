/*
 * EKF.h
 *
 *  Created on: May 7, 2012
 *      Author: Nikolas Engelhard
 */

#ifndef EKF_H_
#define EKF_H_

#include <ros/ros.h>
#include <Eigen/Core>
#include <Eigen/LU>

#include <visnav2013_exercise3/marker.h>

struct ExtendedKalmanFilter {

 Eigen::Vector3f state; // x, y, yaw
 Eigen::Matrix3f sigma; // uncertainty of state

 Eigen::Matrix3f Q; // process noise
 Eigen::Matrix3f R; // observation noise

 void predictionStep(const Eigen::Vector3f& odometry); // x_{t+1} = g(x_t,u) and update uncertainty
 void correctionStep(const Eigen::Vector3f& measurement, const Eigen::Vector3f& global_marker_pose);  // compare expected and measured values, update state and uncertainty

 void initFilter();

 EKF_marker marker;


};


#endif /* EKF_H_ */
