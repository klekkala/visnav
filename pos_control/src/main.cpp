#include <Eigen/Core>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <ardrone_autonomy/Navdata.h>

#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/tfMessage.h>

#include <visnav2013_exercise3/State.h>
#include <visnav2013_exercise3/marker.h>
#include <visnav2013_exercise3/EKF.h>

using namespace std;
using namespace Eigen;

struct Ardrone_localizer
{

  // communication
  ros::NodeHandle nh_;
  ros::Subscriber sub_nav;
  ros::Subscriber sub_cmd;
  ros::Subscriber sub_tf;
  ros::Publisher pub_pose;
  ros::Time lastPublished;

  // pose of markers in world frame
  Vector3f global_pose_4x4_95, global_pose_4x4_1;

  // EKF
  ExtendedKalmanFilter kalman_filter;

  // for visualisation
  EKF_marker ekf_marker;
  drone_marker markers;
  tf::Vector3 lastMarkerPos;

  // init for differentiation
  ros::Time last_stamp;
  bool got_first_nav;

  double last_yaw;

  // for calibration.
  Vector3f lastPos1;
  Vector3f lastPos95;
  tf::Vector3 sumRelPos;
  int numRelPos;
  double lastPos1t, lastPos95t;

  void tfCB(const tf::tfMessageConstPtr msg)
  {

    // get observations and correct
    for (int i = 0; i < msg->transforms.size(); i++)
    {
      if (msg->transforms[i].header.frame_id == "/ardrone_base_bottomcam" && (msg->transforms[i].child_frame_id == "/4x4_1" || msg->transforms[i].child_frame_id == "/4x4_95"))
      {

        // make rotation matrix
        tf::Matrix3x3 R;
        R.setRotation(
            tf::Quaternion(msg->transforms[i].transform.rotation.x, msg->transforms[i].transform.rotation.y, msg->transforms[i].transform.rotation.z, msg->transforms[i].transform.rotation.w));
        tf::Vector3 t = tf::Vector3(msg->transforms[i].transform.translation.x, msg->transforms[i].transform.translation.y, msg->transforms[i].transform.translation.z);

        // get roll, pitch from observation
        double roll, pitch, yaw;
        R.getEulerYPR(yaw, pitch, roll);

        // "remove" roll, pitch.
        tf::Matrix3x3 Ry;
        Ry.setEulerYPR(-yaw, 0, 0);
        tf::Vector3 tNorm = Ry * R.inverse() * t;

        // make measurement
        Vector3f measurement;
        measurement(0) = -tNorm.x();
        measurement(1) = -tNorm.y();
        measurement(2) = yaw;

        // add measurement
        if (msg->transforms[i].child_frame_id == "/4x4_1")
        {
          kalman_filter.correctionStep(measurement, global_pose_4x4_1);
          lastPos1t = msg->transforms[i].header.stamp.toSec();
          lastPos1 = measurement;
        }
        else
        {
          kalman_filter.correctionStep(measurement, global_pose_4x4_95);
          lastPos95t = msg->transforms[i].header.stamp.toSec();
          lastPos95 = measurement;
        }
      }
    }

    // if both markers appear at the same time, print their relative pose for calibration. EXPERIMENTAL!
    if (lastPos95t - lastPos1t > -0.01 && lastPos95t - lastPos1t < 0.01)
    {
      tf::Matrix3x3 Ry;
      Ry.setEulerYPR(lastPos95[2], 0, 0);

      tf::Vector3 relPos = Ry * tf::Vector3(lastPos1[0] - lastPos95[0], lastPos1[1] - lastPos95[1], 0);

      numRelPos++;
      sumRelPos[0] += relPos[0];
      sumRelPos[1] += relPos[1];
      sumRelPos[2] += lastPos95[2] - lastPos1[2];

      printf("relative orientation: %f %f %f (avg: %f %f %f)\n", relPos[0], relPos[1], lastPos95[2] - lastPos1[2], sumRelPos[0] / numRelPos, sumRelPos[1] / numRelPos, sumRelPos[2] / numRelPos);
    }
  }

  void navCB(const ardrone_autonomy::NavdataConstPtr& nav_msg)
  {
    if (last_stamp > nav_msg->header.stamp)
      got_first_nav = false;

    if (!got_first_nav)
    {
      // clear markers
      markers.init();

      // clear ekf
      kalman_filter.initFilter();

      // clear rest
      lastMarkerPos = tf::Vector3(0, 0, 0);
      sumRelPos = tf::Vector3(0, 0, 0);
      numRelPos = 0;
      lastPos1t = -1;
      lastPos95t = 1;
      lastPublished = ros::Time::now();

      // init
      got_first_nav = true;
      last_stamp = nav_msg->header.stamp;
      last_yaw = nav_msg->rotZ;

      return;
    }

    // calc time diff
    double dt_s = (nav_msg->header.stamp - last_stamp).toNSec() / (1000.0 * 1000.0 * 1000.0);
    last_stamp = nav_msg->header.stamp;

    // calc dx, dy
    // transforme to "our" drone CO
    float dx = -dt_s * nav_msg->vy / 1000; // in m/s
    float dy = dt_s * nav_msg->vx / 1000; // in m/s

    // predict EKF
    Eigen::Vector3f odometry;
    odometry(0) = dx; // local position update
    odometry(1) = dy; // local position update
    odometry(2) = (nav_msg->rotZ - last_yaw) / 180 * M_PI; // treat absolute value as incremental update

    last_yaw = nav_msg->rotZ;

    // update pose of robot according to odometry measurement
    // this also increases the uncertainty of the filter
    kalman_filter.predictionStep(odometry);

    float vx_global = cos(kalman_filter.state(2)) * odometry(0) - sin(kalman_filter.state(2)) * odometry(1);
    float vy_global = sin(kalman_filter.state(2)) * odometry(0) + cos(kalman_filter.state(2)) * odometry(1);

    tf::Vector3 pos = tf::Vector3(kalman_filter.state[0], kalman_filter.state[1], nav_msg->altd / 1000.0);

    visnav2013_exercise3::State s;
    s.header = nav_msg->header;
    s.x = pos.x();
    s.y = pos.y();
    s.z = pos.z();
    s.vx = vx_global;
    s.vy = vy_global;
    s.yaw = kalman_filter.state[2];
    pub_pose.publish(s);

    // only publish at 30hz....
    if ((ros::Time::now() - lastPublished).toSec() > 0.01)
    {

      // add to ekf_marker
      ekf_marker.addFilterState(kalman_filter.state, kalman_filter.sigma, nav_msg->altd / 1000.0);

      // make pos
      tf::Vector3 pos = tf::Vector3(kalman_filter.state[0], kalman_filter.state[1], nav_msg->altd / 1000.0);
      tf::Transform transform;
      transform.setOrigin(pos);
      transform.setRotation(tf::Quaternion(0, 0, M_PI / 2 + kalman_filter.state[2]));

      // add to marker array?
      if (lastMarkerPos.distance(pos) > 0.1)
      {
        //markers.addMarkerPose(tf::StampedTransform(transform, nav_msg->header.stamp, "/world", "/ardrone"));
        lastMarkerPos = pos;
      }

      // publish
      ekf_marker.publish_last_n_states(1);
      markers.publish_markers();

      lastPublished = ros::Time::now();
    }
  }

  Ardrone_localizer()
  {

    got_first_nav = false;
    kalman_filter.initFilter();

    // pose of Marker in global coordinates
    global_pose_4x4_95 = Vector3f(0, 0, 0); // in meters
    global_pose_4x4_1 = Vector3f(0.04, 1.12, 0);

    sub_tf = nh_.subscribe("/tf", 100, &Ardrone_localizer::tfCB, this);
    sub_nav = nh_.subscribe("/ardrone/navdata", 100, &Ardrone_localizer::navCB, this);
    pub_pose = nh_.advertise<visnav2013_exercise3::State>("/ardrone/filtered_pose", 10);

    lastMarkerPos = tf::Vector3(0, 0, 0);
    sumRelPos = tf::Vector3(0, 0, 0);
    numRelPos = 0;

    lastPos1t = -1;
    lastPos95t = 1;

    lastPublished = ros::Time::now();
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ardrone_visualization");

  // For testing your correction step
  // ExtendedKalmanFilter EKF;
  // EKF.testFilter();
  // return 0;

  Ardrone_localizer localizer;

  tf::Transform tag_pose_4x4_95, tag_pose_4x4_1;
  tag_pose_4x4_95.setOrigin(tf::Vector3(localizer.global_pose_4x4_95[0], localizer.global_pose_4x4_95[1], localizer.global_pose_4x4_95[2]));
  tag_pose_4x4_1.setOrigin(tf::Vector3(localizer.global_pose_4x4_1[0], localizer.global_pose_4x4_1[1], localizer.global_pose_4x4_1[2]));
  tag_pose_4x4_95.setRotation(tf::Quaternion(0, 0, 0));
  tag_pose_4x4_1.setRotation(tf::Quaternion(0, 0, 0));

  tf::TransformBroadcaster br;

  ros::Rate r(30);
  while (localizer.nh_.ok())
  {
    for (int i = 0; i < 50; i++)
      ros::spinOnce();
    br.sendTransform(tf::StampedTransform(tag_pose_4x4_95, ros::Time::now(), "/world", "/marker_95"));
    br.sendTransform(tf::StampedTransform(tag_pose_4x4_1, ros::Time::now(), "/world", "/marker_1"));
    r.sleep();
  }

  return 0;
}
