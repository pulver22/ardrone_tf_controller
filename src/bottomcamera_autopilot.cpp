#include "ros/ros.h"
#include "ros/package.h"
#include <ros/node_handle.h>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Twist.h>
#include "std_msgs/String.h"
#include <ar_pose/ARMarker.h>
#include <ar_pose/ARMarkers.h>
//#include "../catkin_ws2/devel/include/ar_pose/ARMarker.h"
//#include "../catkin_ws2/devel/include/ar_pose/ARMarkers.h"
#include <tf2/transform_datatypes.h>
#include <tf2_ros/transform_listener.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <boost/lexical_cast.hpp>
//#include <devel/include/ar_pose/ARMarker.h>
#include "std_msgs/Float32.h"
#include "../include/offset.h"

bool marker_found = true;
int last_msg = 0;
const double PI = 3.141593;

using namespace std;

// Save the timestamp of the last time the marker has been seen
void MarkerCallback(const ar_pose::ARMarker::ConstPtr msg)
{
  ROS_INFO("Marker detected!");
  last_msg = msg->header.stamp.sec;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pose_subscriber");
  ROS_INFO("TF subscriber correctly running...");
  ros::NodeHandle nh;
  tf2_ros::Buffer tf_buffer;
  tf2_ros::TransformListener tf_listener(tf_buffer);
  ros::Rate rate(1.0);
  ros::Publisher cmd_pub = nh.advertise<std_msgs::String>("/uga_tum_ardrone/com", 100);
  ros::Subscriber marker_sub = nh.subscribe("/ar_pose_marker", 1, MarkerCallback);
  ros::Publisher linear_offset_pub = nh.advertise<geometry_msgs::Vector3>("/linear_offset", 100);
  ros::Publisher rotational_offset_pub = nh.advertise<std_msgs::Float32>("/rotation_offset", 100);
  geometry_msgs::Vector3 offset_msg;
  std_msgs::Float32 yaw_msg;

  // Define the commands to be sent to the controller
  std_msgs::String clear, auto_init, take_off, go_to, move_by_rel, land, reference, max_control, initial_reach_dist,
      stay_within_dist, stay_time;
  clear.data = "c clearCommands";
  auto_init.data = "c autoInit 500 800";
  reference.data = "setReference $POSE$";
  max_control.data = "setMaxControl 1";
  initial_reach_dist.data = "setInitialReachDist 0.1";
  stay_within_dist.data = "setStayWithinDist 0.1";
  stay_time.data = "setStayTime 0.5";
  take_off.data = "c takeoff";
  land.data = "c land";

  // Save the translation and rotational offsets on three axis
  Offset offset;
  offset.SetRoll(0);
  offset.SetPitch(0);
  offset.SetGaz(0);
  offset.SetYaw(0);

  // Keep counting of the algorithm's number of iterations
  int count = 0;

  // Compensatory factor for movements along 3 axis
  tfScalar roll, pitch, yaw;

  // Parameters for identifing when the drone reaches the target
  float target_x, target_y, target_z, target_yaw;
  float epsilon;

  // Load parameters from param server

  nh.getParam("/ardrone_tf_controller/target_X", target_x);      // expressed in meters (def: 0.1)
  nh.getParam("/ardrone_tf_controller/target_Y", target_y);      // expressed in meters (def: 0.1)
  nh.getParam("/ardrone_tf_controller/target_Z", target_z);      // expressed in meters (def: 0.4)
  nh.getParam("/ardrone_tf_controller/target_yaw", target_yaw);  // expressed in degress (def: 5.0)
  nh.getParam("/ardrone_tf_controller/epsilon",
              epsilon);  // error variable on rotation expressed in radiants (def: 0.78)

  bool was_reverse = false;

  while (nh.ok())
  {
    // Get transform from the drone and the frontcamera
    geometry_msgs::TransformStamped transformStamped;

    try
    {
      transformStamped = tf_buffer.lookupTransform("ar_marker", "ardrone_base_bottomcam", ros::Time(0));
    }
    catch (tf2::TransformException &ex)
    {
      ROS_WARN("%s", ex.what());
      ros::Duration(1.0).sleep();
      continue;
    }

    // Get rotation expressed in quaternion, transform it in a rotation matrix
    // and then retrieve roll pitch and yaw
    tf::Quaternion q(transformStamped.transform.rotation.x, transformStamped.transform.rotation.y,
                     transformStamped.transform.rotation.z, transformStamped.transform.rotation.w);
    tf::Matrix3x3 m(q);
    m.getRPY(roll, pitch, yaw);

    // Save actual offsets on three axis and yaw
    // Consider the drone looking ahead only if its orientation is between
    // [-45,+45] degrees
    if ((yaw > -epsilon) && (yaw < epsilon))
    {
      // Multiplyer changes coordinates frame if the drone was previously revers
      // wrt to the marker
      int multiplyer = 1;
      if (was_reverse == true)
      {
        multiplyer = -1;
      }
      offset.SetRoll(-multiplyer * transformStamped.transform.translation.x);
      offset.SetPitch(-multiplyer * transformStamped.transform.translation.y);
      offset.SetGaz(-abs(transformStamped.transform.translation.z));
      offset.SetYaw((float)yaw * 180 / PI);
    }
    else
    {
      offset.SetRoll(-transformStamped.transform.translation.x);
      offset.SetPitch(-transformStamped.transform.translation.y);
      offset.SetGaz(-abs(transformStamped.transform.translation.z));
      offset.SetYaw(((float)yaw * 180 / PI));
      was_reverse = true;
    }

    // Publish the offsets
    offset_msg.x = (offset.GetRoll());
    offset_msg.y = (offset.GetPitch());
    offset_msg.z = (offset.GetGaz());
    linear_offset_pub.publish(offset_msg);
    yaw_msg.data = offset.GetYaw();
    rotational_offset_pub.publish<>(yaw_msg);
    // ------------------------------------

    // Modify current_offset in proximity of the target
    offset.ReduceOffsetToZero(offset, target_x, target_y, target_z, target_yaw);

    // Create the navigation command
    move_by_rel.data = "c moveByRel " + boost::lexical_cast<std::string>(offset.GetRoll()) + " " +
                       boost::lexical_cast<std::string>(offset.GetPitch()) + " " +
                       boost::lexical_cast<std::string>(offset.GetGaz()) + " " +
                       boost::lexical_cast<std::string>(offset.GetYaw());

    if (move_by_rel.data.compare("c moveByRel 0 0 0 0") == 0)
    {
      ROS_INFO("Destination reached");
      cmd_pub.publish<>(move_by_rel);
      cmd_pub.publish<>(land);
      // ros::shutdown();
    }
    else
    {
      // If the time stamp of the last message is equal to the current time, it
      // means the marker has been correctly detected,
      // otherwise it has been lost

      if (last_msg == 0)
      {
        // do nothing
      }
      else
      {
        int currentTimeInSec = (int)ros::Time::now().sec;
        if (currentTimeInSec == last_msg)
        {
          cmd_pub.publish<>(clear);
          // Initialize the controller just before the flight
          if (count == 1)
          {
            ROS_INFO("Initialization");
            cmd_pub.publish<>(auto_init);
            cmd_pub.publish<>(reference);
            cmd_pub.publish<>(max_control);
            cmd_pub.publish<>(initial_reach_dist);
            cmd_pub.publish<>(stay_within_dist);
            cmd_pub.publish<>(stay_time);
            // cmd_pub.publish<> ( takeoff );
          }
          else
          {
            cmd_pub.publish<>(move_by_rel);

            // cout << "alive" << endl;
            cout << move_by_rel.data << endl;
          }
          cout << endl;
          // ros::Duration ( 1.0 ).sleep();
        }
        else
        {
          ROS_WARN("Marker is lost!");
          ROS_ERROR("CurrentTime is %d while last_msg is %d", currentTimeInSec, last_msg);
          cmd_pub.publish<>(clear);
          cmd_pub.publish<>(auto_init);
          cmd_pub.publish<>(reference);
          cmd_pub.publish<>(max_control);
          cmd_pub.publish<>(initial_reach_dist);
          cmd_pub.publish<>(stay_within_dist);
          cmd_pub.publish<>(stay_time);

          move_by_rel.data = "c moveByRel 0 0 0.2 0";
          cmd_pub.publish<>(move_by_rel);
          cout << move_by_rel.data << endl;
          cout << endl;
          // ros::Duration ( 1.0 ).sleep();
        }
      }
    }
    count++;
    rate.sleep();
    ros::spinOnce();
  }

  return 0;
}
