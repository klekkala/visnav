
#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>

#include <std_msgs/Bool.h>
#include <geometry_msgs/Twist.h>
#include <visualization_msgs/Marker.h>

#include <visnav2013_exercise3/State.h>
#include <visnav2013_exercise3/PidParameterConfig.h>

class PidController
{
public:
  float c_proportional;
  float c_integral;
  float c_derivative;

  PidController()
  {
    c_proportional = c_integral = c_derivative = 0;
    reset();
  }

  float getCommand(const ros::Time& t, float error)
  {
    // derivative
    float derror = 0;
    // TODO: implement numerical differentiation
    
    return getCommand(t, error, derror);
  }
  
  float getCommand(const ros::Time& t, float error, float derror)
  {
    // TODO: implement PID control law
    return 0.0;
  }

  // resets the internal state
  void reset()
  {
  }
private:
};

class ArdroneController
{
private:
  ros::NodeHandle& nh;
  ros::Subscriber sub_pose, sub_enabled;
  ros::Publisher pub_vel;
  ros::Publisher pub_cmd_marker;

  dynamic_reconfigure::Server<visnav2013_exercise3::PidParameterConfig> reconfigure_server;
  visnav2013_exercise3::PidParameterConfig current_cfg;
  
  geometry_msgs::Twist twist;
  visnav2013_exercise3::State state;

  PidController pid_x, pid_y, pid_yaw;

  bool enabled;
  float goal_x, goal_y, goal_yaw;

public:
  ArdroneController(ros::NodeHandle& nh) :
      nh(nh),
      reconfigure_server(),
      enabled(false)
  {
    pub_vel = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    pub_cmd_marker = nh.advertise<visualization_msgs::Marker>("visualization_marker", 10);
    
    reconfigure_server.setCallback(boost::bind(&ArdroneController::onConfig, this, _1, _2));

    twist.linear.x = twist.linear.y = twist.linear.z = 0;
    twist.angular.x = twist.angular.y = twist.angular.z = 0;
    setGoalPose(0, 0, 0);

    sub_enabled = nh.subscribe<std_msgs::Bool>("/ardrone/enable_controller", 1, boost::bind(&ArdroneController::onEnableController, this, _1));
    sub_pose = nh.subscribe<visnav2013_exercise3::State>("/ardrone/filtered_pose", 1, boost::bind(&ArdroneController::onFilteredPose, this, _1));
  }

  void setPidParameters(visnav2013_exercise3::PidParameterConfig &config)
  {
    pid_x.c_proportional = pid_y.c_proportional = config.c_prop_trans;
    pid_x.c_integral = pid_y.c_integral = config.c_int_trans;
    pid_x.c_derivative = pid_y.c_derivative = config.c_deriv_trans;

    pid_yaw.c_proportional = config.c_prop_yaw;
    pid_yaw.c_integral = config.c_int_yaw;
    pid_yaw.c_derivative = config.c_deriv_yaw;
  }

  void setGoalPose(float x, float y, float yaw)
  {
    goal_x = x;
    goal_y = y;
    goal_yaw = yaw;
  }
  
  void setEnabled(bool v)
  {
    enabled = v;
    
    if(!enabled)
    {
      pid_x.reset();
      pid_yaw.reset();
    }
  }

  void onTimerTick(const ros::TimerEvent& e)
  {
    calculateContolCommand(e.current_real);
    
    if(enabled)
      pub_vel.publish(twist);
      
    sendCmdMarker(e.current_real);
  }

  void onConfig(visnav2013_exercise3::PidParameterConfig& cfg, uint32_t level)
  {
    current_cfg = cfg;
    setEnabled(cfg.enable);
    setPidParameters(cfg);
  }

  void onEnableController(const std_msgs::Bool::ConstPtr& msg)
  {
    setEnabled(msg->data);
    current_cfg.enable = msg->data;
    reconfigure_server.updateConfig(current_cfg);
  }

  void onFilteredPose(const visnav2013_exercise3::State::ConstPtr& pose_msg)
  {
    state = *pose_msg;
  }

  // control in xy and yaw
  void calculateContolCommand(const ros::Time& t)
  {
    // TODO: implement error computation and calls to pid controllers to get the commands

    // use this yaw to rotate commands from global to local frame
    float yaw = -(state.yaw + M_PI_2);
    
    twist.linear.x = 0.0; // = ??
    twist.linear.y = 0.0; // = ??

    float u_yaw = 0.0; // = ??

    // normalize angular control command
    twist.angular.z = atan2(sin(u_yaw), cos(u_yaw));
  }

  void sendCmdMarker(const ros::Time& t)
  {
    visualization_msgs::Marker marker;

    marker.header.frame_id = "/world";

    marker.action = visualization_msgs::Marker::ADD;
    marker.ns = "ardrone_controller";
    marker.header.stamp = t;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.05; // width of line in m
    marker.scale.y = 0.1; // width of line in m
    geometry_msgs::Point p, q;
    p.x = state.x;
    p.y = state.y;
    p.z = state.z;

    float length = 2.0;

    marker.color.a = 1.0;
    marker.color.g = 1.0;
    marker.type = visualization_msgs::Marker::ARROW;
    marker.points.clear();
    marker.id = 0;

    float yaw = -(state.yaw + M_PI_2);
    q.x = p.x + length * (cos(yaw) * twist.linear.x + sin(yaw) * twist.linear.y);
    q.y = p.y + length * (-sin(yaw) * twist.linear.x + cos(yaw) * twist.linear.y);
    q.z = p.z + length * twist.linear.z;

    marker.points.push_back(p);
    marker.points.push_back(q);
    pub_cmd_marker.publish(marker);

    // show rotation: attach arrow to end of pose-marker-arrow and rotate by 90deg
    // length is proportional to the rotation speed
    marker.points.clear();
    p.x = state.x + cos(yaw) * 0.5; //0.5 = length of pose-marker-arrow
    p.y = state.y - sin(yaw) * 0.5;
    q.x = p.x + 1 * (cos(yaw - M_PI / 2) * twist.angular.z); // 10: arbitrary constant (angular.z is in rad, marker in meter)
    q.y = p.y + 1 * (-sin(yaw - M_PI / 2) * twist.angular.z);
    q.z = p.z + length * twist.linear.z;
    marker.points.push_back(p);
    marker.points.push_back(q);
    marker.id = 1;
    marker.color.g = 0;
    marker.color.b = 1.0;

    pub_cmd_marker.publish(marker);
  }

};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ardrone_controller");
  ros::NodeHandle nh;

  ArdroneController controller(nh);

  ros::Timer timer = nh.createTimer(ros::Duration(0.02), boost::bind(&ArdroneController::onTimerTick, &controller, _1));

  ros::spin();
  
  return 0;
}
