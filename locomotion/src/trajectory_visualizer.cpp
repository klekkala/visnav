#include <visnav2013_exercise/trajectory_visualizer.h>

namespace visnav2013_exercise
{

TrajectoryVisualizer::TrajectoryVisualizer(ros::NodeHandle& nh)
{
  marker_publisher_ = nh.advertise<visualization_msgs::MarkerArray>("/visualization_marker_array", 1);

  visualization_msgs::Marker trajectory_marker;
  trajectory_marker.header.frame_id = "/world";
  trajectory_marker.id = 1;
  trajectory_marker.ns = "ex1_ardrone_odometry";
  trajectory_marker.action = visualization_msgs::Marker::ADD;
  trajectory_marker.lifetime = ros::Duration(0.0);
  trajectory_marker.color.r = 1;
  trajectory_marker.color.g = 0;
  trajectory_marker.color.b = 0;
  trajectory_marker.color.a = 1;
  trajectory_marker.scale.x = 0.01;
  trajectory_marker.scale.y = 0.01;
  trajectory_marker.scale.z = 0.01;
  trajectory_marker.type = visualization_msgs::Marker::LINE_STRIP;


  pose_marker_prototype_.header.frame_id = "/world";
  pose_marker_prototype_.id = 2;
  pose_marker_prototype_.ns = "ex1_ardrone_odometry";
  pose_marker_prototype_.action = visualization_msgs::Marker::ADD;
  pose_marker_prototype_.lifetime = ros::Duration(0.0);
  pose_marker_prototype_.color.r = 1;
  pose_marker_prototype_.color.g = 0;
  pose_marker_prototype_.color.b = 0;
  pose_marker_prototype_.color.a = 1;
  pose_marker_prototype_.scale.x = 0.2;
  pose_marker_prototype_.scale.y = 0.2;
  pose_marker_prototype_.scale.z = 0.1;
  pose_marker_prototype_.type = visualization_msgs::Marker::ARROW;

  markers_.markers.push_back(trajectory_marker);
}

bool isMarkerFurtherAwayThan(const visualization_msgs::Marker& marker, const tf::Transform& pose, const float d)
{
  tf::Transform marker_pose;
  tf::poseMsgToTF(marker.pose, marker_pose);

  return pose.getOrigin().distance(marker_pose.getOrigin()) > d;
}

TrajectoryVisualizer& TrajectoryVisualizer::addPose(const tf::Transform& pose)
{
  geometry_msgs::Point p;
  p.x = pose.getOrigin().x();
  p.y = pose.getOrigin().y();
  p.z = pose.getOrigin().z();

  markers_.markers.front().points.push_back(p);

  // only add an arrow marker every 10cm, otherwise RVIZ gets laggy
  if(markers_.markers.size() == 1 || isMarkerFurtherAwayThan(markers_.markers.back(), pose, 0.1f))
  {
    pose_marker_prototype_.id = markers_.markers.back().id + 1;
    tf::poseTFToMsg(pose, pose_marker_prototype_.pose);

    markers_.markers.push_back(pose_marker_prototype_);
  }

  return *this;
}

void TrajectoryVisualizer::publish()
{
  marker_publisher_.publish(markers_);
}

} /* namespace visnav2013_exercise */
