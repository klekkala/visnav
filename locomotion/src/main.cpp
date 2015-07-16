#include <ros/ros.h>
#include <tf/LinearMath/Transform.h>
#include <ardrone_autonomy/Navdata.h>

#include <visnav2013_exercise/trajectory_visualizer.h>

/**
 * ARDroneOdometry, computes the position of the AR.Drone from its measurements.
 *
 * Extend the ARDroneOdometry::onNavdata(...) method with your solution.
 * Feel free to adapt the code to your needs ;)
 */
class ARDroneOdometry
{
private:
  ros::Subscriber navdata_subscriber_;
  visnav2013_exercise::TrajectoryVisualizer visualizer_;

  tf::Transform pose_;
  double avg_height_sum_, avg_height_count_, traveled_distance_;
  ros::Time last_time_;
public:
  ARDroneOdometry(ros::NodeHandle& nh) :
    visualizer_(nh),
    last_time_(0)
  {
    pose_.setIdentity();

    // subscribe to the '/ardrone/navdata' topic
    navdata_subscriber_ = nh.subscribe<ardrone_autonomy::Navdata>("/ardrone/navdata", 1, boost::bind(&ARDroneOdometry::onNavdata, this,  _1));

    ROS_INFO("Subscribed to '/ardrone/navdata' topic.");
  }

  /**
   * Subscriber method, called for every received ardrone_autonomy::Navdata message.
   */
  void onNavdata(const ardrone_autonomy::NavdataConstPtr& navdata)
  {
    ROS_INFO_STREAM("received navdata @" << navdata->header.stamp << " with vx: " << navdata->vx << " vy: " << navdata->vy << " height: " << navdata->altd << " yaw: " << navdata->rotZ);

    ros::Time time = navdata->header.stamp;

    tf::Matrix3x3 attitude, yaw;
    attitude.setRPY(convertAngleARDroneToTf(navdata->rotX), convertAngleARDroneToTf(navdata->rotY), convertAngleARDroneToTf(navdata->rotZ));
    yaw.setRPY(0, 0, convertAngleARDroneToTf(navdata->rotZ));

    if(!last_time_.isZero())
    {
      tf::Vector3 old_position = pose_.getOrigin();
      tf::Vector3 body_velocity(navdata->vx, navdata->vy, 0);
      double dt = (time - last_time_).toSec();

      // integrate velocity
      tf::Vector3 dposition = dt * body_velocity * 0.001;
      pose_.getOrigin() += yaw * dposition;

      traveled_distance_ += pose_.getOrigin().distance(old_position);
      avg_height_sum_ += pose_.getOrigin().z();
      avg_height_count_ += 1.0;

      ROS_INFO_STREAM("traveled distance: " << traveled_distance_ << "m average height: " << avg_height_sum_ / avg_height_count_ << "m");
    }

    pose_.setBasis(attitude);
    pose_.getOrigin().setZ(navdata->altd * 0.001);

    visualizer_.addPose(pose_).publish();

    last_time_ = time;
  }

  /**
   * AR.Drone angles are in degrees and in the interval [-180° +180°],
   * in contrast, TF uses angles in radians and in the interval [0 2*pi]
   */
  float convertAngleARDroneToTf(const float& angle)
  {
    return angle / 180.0f * M_PI + M_PI;
  }
};


int main(int argc, char **argv) {
  // initialize the ROS node
  ros::init(argc, argv, "Ex1ARDroneOdometry");

  // create a NodeHandle to subscribe/advertise topics
  ros::NodeHandle nh;

  // create our main class
  ARDroneOdometry odometry(nh);

  // wait until shutdown, e.g., until someone presses Ctrl+C
  ros::spin();

  return 0;
}
