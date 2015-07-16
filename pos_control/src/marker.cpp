#include <visnav2013_exercise3/marker.h>

using namespace std;
using namespace Eigen;


void EKF_marker::publish_last_n_states(int n){

 visualization_msgs::MarkerArray marker_list;
 visualization_msgs::Marker marker;

 marker.header.frame_id = "/world";

 marker.action = visualization_msgs::Marker::ADD;
 marker.ns = "ekf_marker_ns";
 marker.header.stamp = ros::Time::now();
 marker.pose.orientation.w = 1.0;
 marker.scale.x = 0.05; // width of line in m
 marker.color.a = 1.0;

 assert(mus.size() == sigmas.size());

 const uint segs = 16;

 int path_length = (n<0)?mus.size():n;
 int id = 0;

 for (int i = mus.size()-1; i >= mus.size()-path_length && i>0; i-=1){


  // create circle
  Matrix<float,2,segs> xy;
  for (uint j=0; j<segs; ++j){
   xy(0,j) = cos(2*M_PI/segs*j);
   xy(1,j) = sin(2*M_PI/segs*j);
  }

  // R = sigmas(i)(1:2,1:2) // left upper part
  Matrix2f R; R << sigmas[i](0,0), sigmas[i](0,1) , sigmas[i](1,0) , sigmas[i](1,1);

  xy = R*xy;

  float z = heights[i];

  std_msgs::ColorRGBA col;
  geometry_msgs::Point p;


  // show direction of drone:
  marker.type = visualization_msgs::Marker::LINE_STRIP;
  marker.points.clear(); marker.colors.clear();
  marker.id = id++;;
  marker.color.b = 0; marker.color.g = 0; marker.color.r = 1;
  p.x = mus[i](0);   p.y = mus[i](1); p.z = z;
  marker.points.push_back(p);
  p.x = mus[i](0)-0.5*sin(mus[i](2));   p.y = mus[i](1)+0.5*cos(mus[i](2)); p.z = z;
  marker.points.push_back(p);

  pub_markers.publish(marker);


  // show uncertainty ellipse
  marker.points.clear(); marker.colors.clear();
  marker.type = visualization_msgs::Marker::LINE_STRIP;
  marker.color.b = 1; marker.color.g = 0; marker.color.r = 0;
  marker.id = id++;

  for (uint j=0; j<segs; ++j){
    p.x = xy(0,j)+mus[i](0); p.y = xy(1,j)+mus[i](1); p.z = z;
   marker.points.push_back(p);
  }
  marker.points.push_back(marker.points[0]);

  pub_markers.publish(marker);


 }

}



void drone_marker::publish_markers(){

 if (pub_markers.getNumSubscribers() == 0)
  return;


 visualization_msgs::MarkerArray marker_list;
 visualization_msgs::Marker marker;

 marker.header.frame_id = "/world";

 marker.type = visualization_msgs::Marker::ARROW;
 marker.action = visualization_msgs::Marker::ADD;

 marker.scale.x = 0.2;
 marker.scale.y = 0.05;
 marker.scale.z = 0.05;

 marker.color.a = 1.0;
 marker.color.r = 1.0;


  for (uint i=0; i<trafos.size(); ++i){
   marker.id = i;
   marker.header.stamp = trafos[i].stamp_;

   marker.pose.position.x = trafos[i].getOrigin().getX();
   marker.pose.position.y = trafos[i].getOrigin().getY();
   marker.pose.position.z = trafos[i].getOrigin().getZ();

   marker.pose.orientation.x = trafos[i].getRotation().getX();
   marker.pose.orientation.y = trafos[i].getRotation().getY();
   marker.pose.orientation.z = trafos[i].getRotation().getZ();
   marker.pose.orientation.w = trafos[i].getRotation().getW();

   marker_list.markers.push_back(marker);
  }

 //ROS_INFO("Publishing %zu markers", marker_list.markers.size());
 pub_markers.publish(marker_list);

 // update tf frame:
 if (trafos.size() > 0){
  //  ROS_INFO("updating trafo");
  br.sendTransform(trafos[trafos.size()-1]);
 }

}
