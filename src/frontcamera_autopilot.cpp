#include "ros/ros.h"
#include "ros/package.h"
#include <ros/node_handle.h>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Twist.h>
#include "std_msgs/String.h"
#include <ar_pose/ARMarker.h>
#include <tf2/transform_datatypes.h>
#include <tf2_ros/transform_listener.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <boost/lexical_cast.hpp>
#include "sensor_msgs/ChannelFloat32.h"
#include "std_msgs/Float32.h"
#include "../include/offset.h"

int last_msg = 0;
const double PI = 3.141593;
unsigned int ros_header_timestamp_base = 0;

using namespace std;

float getCompensatoryFactor ( double current_offset, double last_offset, int count, bool &axisCrossed, double target,
                              int type );

// Save the timestamp of the last time the marker has been seen
void markerCallback ( const ar_pose::ARMarker::ConstPtr msg )
{
  ROS_INFO ( "Marker detected!" );
  last_msg = msg->header.stamp.sec;
}

int main ( int argc, char **argv )
{
  ros::init ( argc, argv, "pose_subscriber" );
  ROS_INFO ( "TF subscriber correctly running..." );
  ros::NodeHandle nh;
  tf2_ros::Buffer tf_buffer;
  tf2_ros::TransformListener tf_listener ( tf_buffer );
  ros::Rate rate ( 1.0 );
  // For uga_tum_ardrone -> spring damped controller
  ros::Publisher cmd_pub = nh.advertise<std_msgs::String> ( "/uga_tum_ardrone/com", 100 );
  // For tum_ardrone -> PID controller
  //ros::Publisher cmd_pub = nh.advertise<std_msgs::String>("/tum_ardrone/com", 100);
  ros::Subscriber marker_sub = nh.subscribe ( "/ar_pose_marker", 1, markerCallback );
  ros::Publisher linear_offset_pub = nh.advertise<geometry_msgs::Vector3> ( "/linear_offset", 100 );
  ros::Publisher rotational_offset_pub = nh.advertise<std_msgs::Float32> ( "/rotation_offset", 100 );
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
  Offset current_offset;
  current_offset.SetRoll ( 0 );
  current_offset.SetPitch ( 0 );
  current_offset.SetGaz ( 0 );
  current_offset.SetYaw ( 0 );
  Offset last_view_offset, compensatory_offset;
  last_view_offset.SetOffset ( current_offset );

  // Keep counting of the algorithm's number of iterations
  int count = 0;

  // Compensatory factors for movements along 3 axis
  float k_roll, k_pitch, k_gaz;

  tfScalar roll, pitch, yaw;

  // Parameters for identifing when the drone reaches the target
  float target_x, target_y, target_z, target_yaw;
  float epsilon;

  // Set to true if drone has overcome the X axis
  bool x_axis_crossed = false;
  bool y_axis_crossed = false;
  bool z_axis_crossed = false;

  bool branch_unsafe = false;

  // Load parameters from param server
  nh.getParam ( "/ardrone_tf_controller/target_X", target_x );   // expressed in meters ( default: 0.1 )
  nh.getParam ( "/ardrone_tf_controller/target_Y", target_y );   // expressed in meters ( default: 0.1 )
  nh.getParam ( "/ardrone_tf_controller/target_Z", target_z );   // expressed in meters ( default: 0.4 )
  nh.getParam ( "/ardrone_tf_controller/target_yaw", target_yaw ); // expressed in degress ( default: 5.0 )
  nh.getParam ( "/ardrone_tf_controller/epsilon", epsilon );     // error variable on
  // rotation expressed
  // in radiants (
  // default: 0.78 )

  while ( nh.ok() )
    {
      // Get transform from the drone and the frontcamera
      geometry_msgs::TransformStamped transform_stamped;
      try
        {
          transform_stamped = tf_buffer.lookupTransform ( "ar_marker", "ardrone_base_frontcam", ros::Time ( 0 ) );
        }
      catch ( tf2::TransformException &ex )
        {
          ROS_WARN ( "%s", ex.what() );
          ros::Duration ( 1.0 ).sleep();
          continue;
        }

      // Get rotation expressed in quaternion, transform it in a rotation matrix
      // and then retrieve roll pitch and yaw
      tf::Quaternion q ( transform_stamped.transform.rotation.x, transform_stamped.transform.rotation.y,
                         transform_stamped.transform.rotation.z, transform_stamped.transform.rotation.w );
      tf::Matrix3x3 m ( q );
      m.getRPY ( roll, pitch, yaw );
			//cout << " yaw: " << yaw << endl;

      // Save actual offsets on three axis and yaw
      if ( transform_stamped.transform.translation.z > 0 )
        {
          current_offset.SetRoll ( -transform_stamped.transform.translation.x );
          current_offset.SetPitch ( transform_stamped.transform.translation.z );
          current_offset.SetGaz ( -transform_stamped.transform.translation.y );
          current_offset.SetYaw ( ( float ) pitch * 180 / PI );
        }
      else
        {
          // FIXME: probably wrong
          ROS_WARN ( " Branch unsafe!" );
          branch_unsafe = true;
          current_offset.SetRoll ( -transform_stamped.transform.translation.x );
          current_offset.SetPitch ( -transform_stamped.transform.translation.z );
          current_offset.SetGaz ( transform_stamped.transform.translation.y );
          current_offset.SetYaw ( ( float ) pitch * 180 / PI );
        }

      // Publish the offsets
      offset_msg.x = ( current_offset.GetRoll() );
      offset_msg.y = ( current_offset.GetPitch() );
      offset_msg.z = ( current_offset.GetGaz() );
      linear_offset_pub.publish ( offset_msg );
      yaw_msg.data = current_offset.GetYaw();
      rotational_offset_pub.publish<> ( yaw_msg );
      // ------------------------------------

      // Create a copy of the current_offset to evaluare the compensatory factor
      compensatory_offset.SetOffset ( current_offset );

      // Modify current_offset in proximity of the target
      current_offset.ReduceOffsetToZero ( current_offset, target_x, target_y, target_z, target_yaw );

      // Create the navigation command
      move_by_rel.data = "c moveByRel " + boost::lexical_cast<std::string> ( current_offset.GetRoll() ) + " " +
                         boost::lexical_cast<std::string> ( current_offset.GetPitch() ) + " " +
                         boost::lexical_cast<std::string> ( current_offset.GetGaz() ) + " " +
                         boost::lexical_cast<std::string> ( current_offset.GetYaw() );




      // If the time stamp of the last message is equal to the current
      // time, it means the marker has been correctly detected,otherwise it has
      // been lost

      k_roll = k_pitch = k_gaz = 1;

      if ( last_msg == 0 )
        {
          // do nothing
        }
      else
        {
          int currentTimeInSec = ( int ) ros::Time::now().sec;
          // Marker has been reached
          //TODO: Move this somewehre after the marker has been seen, and keep position only if the marker continue to be seen
          if ( move_by_rel.data.compare ( "c moveByRel 0 0 0 0" ) == 0 )
            {
              ROS_INFO ( "Destination reached" );
              cmd_pub.publish<> ( move_by_rel );
              continue;
              // cmd_pub.publish<> ( land );
              // ros::shutdown();
            }

          // FIXME: Test this time-depending comparison - maybe it's a good idea
          // to give 3 seconds for network issue
          if ( currentTimeInSec <= ( last_msg + 3 ) )
            {
              cmd_pub.publish<> ( clear );
              // Initialize the controller just before the flight
              if ( count == 1 )
                {
                  ROS_INFO ( "Initialization" );
                  cmd_pub.publish<> ( auto_init );
                  cmd_pub.publish<> ( reference );
                  cmd_pub.publish<> ( max_control );
                  cmd_pub.publish<> ( initial_reach_dist );
                  cmd_pub.publish<> ( stay_within_dist );
                  cmd_pub.publish<> ( stay_time );
                  // cmd_pub.publish<> ( takeoff );
                }
              else
                {
                  // TODO: Implement the update for k_pitch (probably not necessary)
                  // and k_gaz
                  // Update the compensatory factor for drift on roll
                  // 1 -> roll
                  // 2 -> pitch
                  // 3 -> gaz
                  k_roll = getCompensatoryFactor ( compensatory_offset.GetRoll(), last_view_offset.GetRoll(), count,
                                                   x_axis_crossed, target_x, 1 );
                  k_pitch = getCompensatoryFactor ( compensatory_offset.GetPitch(), last_view_offset.GetPitch(), count,
                                                    y_axis_crossed, target_y, 2 );
                  //k_gaz = getCompensatoryFactor ( compensatory_offset.GetGaz(), last_view_offset.GetGaz(), count,
                  //                                z_axis_crossed, target_z, 3 );

                  // Update the navigation command
                  move_by_rel.data = "c moveByRel " + boost::lexical_cast<std::string> ( k_roll * current_offset.GetRoll() ) +
                                     " " + boost::lexical_cast<std::string> ( k_pitch * current_offset.GetPitch() ) + " " +
                                     boost::lexical_cast<std::string> ( k_gaz * current_offset.GetGaz() ) + " " +
                                     boost::lexical_cast<std::string> ( current_offset.GetYaw() );

                  cmd_pub.publish<> ( move_by_rel );
                  cout << move_by_rel.data << endl;
                  // Update the last offset
                  last_view_offset.SetOffset ( compensatory_offset );
                }

              cout << endl;
            }
          else
            {
              ROS_ERROR ( "Marker is lost!" );
              ROS_ERROR ( "CurrentTime is %d while last_msg is %d", currentTimeInSec, last_msg );
              cmd_pub.publish<> ( clear );
              /*
              cmd_pub.publish<>(auto_init);
              cmd_pub.publish<>(reference);
              cmd_pub.publish<>(max_control);
              cmd_pub.publish<>(initial_reach_dist);
              cmd_pub.publish<>(stay_within_dist);
              cmd_pub.publish<>(stay_time);
              */

              k_roll = k_pitch = k_gaz = 1.0;
              // Redirect the drone towards the last position i which the drone was
              // seen
              // FIXME: Check the logic of this code
              move_by_rel.data = "c moveByRel " + boost::lexical_cast<std::string> ( -k_roll * last_view_offset.GetRoll() ) + " "+
                                 boost::lexical_cast<std::string> ( -k_pitch * last_view_offset.GetPitch() ) + " " +
                                 boost::lexical_cast<std::string> ( -k_gaz * last_view_offset.GetGaz() ) + " " +
                                 boost::lexical_cast<std::string> ( -last_view_offset.GetYaw() );

              cmd_pub.publish<> ( move_by_rel );
              cout << move_by_rel.data << endl;
              cout << endl;
            }
        }

      bool x_axis_crossed = false;
      bool y_axis_crossed = false;
      bool z_axis_crossed = false;
      count++;
      rate.sleep();
      ros::spinOnce();
    }

  return 0;
}

// Update the value of the compensatory factor for the drift
float getCompensatoryFactor ( double current_offset, double last_offset, int count, bool &axis_crossed, double target,
                              int type )
{
  float tmp_compensatory_factor = 1.0;
  int ordinata = 1.0;
  float angular_coefficient;

  switch ( type )
    {
      // Roll
    case ( 1 ) :
      angular_coefficient = 40.0;
      break;
      // Pitch
    case ( 2 ) :
			angular_coefficient = 1.0;
      break;
      // Gaz
    case ( 3 ) :
      angular_coefficient = 1.0;
      break;
    }

  // count = 1 Initialization
  // count = 2 First detection (last_view_offset = 0)
  // TODO: If the drone has overcome the virtual axis, keep the compensatory
  // factor high while he has an offset higher the target_X
  if ( last_offset * current_offset <= 0 && count > 2 )
    {
      if ( axis_crossed == false )
        axis_crossed = true;
    }
  // cout << "Boolean variable status: " << x_axis_crossed << endl;

  if ( axis_crossed == true )
    {
      switch ( type )
        {
        case ( 1 ) :
          ROS_WARN ( "Virtual X axis crossed" );
          break;
        case ( 2 ) :
          ROS_WARN ( "Virtual Y axis crossed" );
          break;
        case ( 3 ) :
          ROS_WARN ( "Virtual Z axis crossed" );
          break;
        }
      // FIXME: Modify tmpCompensatoryFactor to decrease as linear function
      tmp_compensatory_factor = max ( 1.0, angular_coefficient * ( abs ( current_offset ) - abs ( target ) ) + ordinata );
    }

  cout << "Inside function: " << type << " : " << tmp_compensatory_factor << endl;
  return tmp_compensatory_factor;
}
