#include <ros/ros.h>
#include <ros/node_handle.h>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Twist.h>
#include <ar_pose/ARMarker.h>
#include <tf2/transform_datatypes.h>
#include <tf2_ros/transform_listener.h>


using namespace std;
using namespace cv;

/*
void printPose ( const ar_pose::ARMarker::ConstPtr &msg )
{
        tf::Pose marker_pose_in_camera;


        marker_pose_in_camera.setOrigin (tf::Vector3 (msg->pose.pose.position.x,
                                         msg->pose.pose.position.y,
                                         msg->pose.pose.position.z));


        cout<<"-------------"<<endl;
        cout<<"x :" << msg->pose.pose.position.x<<endl;
        cout<<"y :" << msg->pose.pose.position.y<<endl;
        cout<<"z :" << msg->pose.pose.position.z<<endl;
        cout<<"-------------"<<endl;
}*/



int main ( int argc, char **argv )
{
  ros::init ( argc, argv,"pose_subscriber" );
  ROS_INFO ( "TF subscriber correctly running..." );
  ros::NodeHandle nh;
  //ros::Subscriber pose_sub=nh.subscribe<> ( "ar_pose_marker",1000,printPose );
  ros::Publisher vel_pub =nh.advertise<geometry_msgs::Twist> ( "/cmd_vel", 10 );
  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tf_listener ( tfBuffer );
  ros::Rate rate ( 2.0 );
  ros::Publisher errors_pub = nh.advertise<geometry_msgs::Vector3> ( "/position_errors",10 );

  while ( nh.ok() )
    {
      geometry_msgs::TransformStamped transformStamped;
      geometry_msgs::Vector3 errors_msg;
      try
        {
          transformStamped = tfBuffer.lookupTransform ( "ar_marker","ardrone_base_frontcam",ros::Time ( 0 ) );
        }
      catch ( tf2::TransformException &ex )
        {
          ROS_WARN ( "%s",ex.what() );
          ros::Duration ( 1.0 ).sleep();
          continue;
        }

      // Save the translation and rotational offsets on three axis
      float linear_offset_X, linear_offset_Y, linear_offset_Z;
      float rotational_offset_Z;
      linear_offset_X = transformStamped.transform.translation.x;
      linear_offset_Y = transformStamped.transform.translation.y;
      linear_offset_Z = transformStamped.transform.translation.z;
      rotational_offset_Z = transformStamped.transform.rotation.z;

      // Adjust distance on the global X axis - z for marker and camera
      // Adjust distance on the global y axis - x for marker and camera
      // Adjust distance on the global z axis - y for marker and camera


      // Create the twist message
      geometry_msgs::Twist vel_msg;

      // PID gains
      double kp_roll_pitch = 0.001;
      double ki_roll_pitch = 0.001;
      double kd_roll_pitch = 0.001;

      double kp_yaw = 0.001;
      double ki_yaw = 0.001;
      double kd_yaw = 0.001;

      double kp_gaz = 0.001;
      double ki_gaz = 0.001;
      double kd_gaz = 0.001;

      // Errors
      double linear_error_x = linear_offset_Z;
      double linear_error_y = linear_offset_X;
      double linear_error_z = linear_offset_Y;
      double rotational_error_z = rotational_offset_Z;

      //Time [s]
      static int64 last_t = 0.0;
      double dt = ( cv::getTickCount() - last_t ) / cv::getTickFrequency();
      last_t = cv::getTickCount();

      // Integral terms
      static double linear_integral_x = 0.0, linear_integral_y = 0.0, linear_integral_z = 0.0, rotational_integral_z = 0.0;
      if ( dt > 0.1 )
        {
          //Reset
          linear_integral_x = 0.0;
          linear_integral_y = 0.0;
          linear_integral_z = 0.0;
          rotational_integral_z = 0.0;
        }
      linear_integral_x += linear_error_x * dt;
      linear_integral_y += linear_error_y * dt;
      linear_integral_z += linear_error_z * dt;
      rotational_integral_z += rotational_error_z * dt;

      // Derivative terms
      static double linear_previous_error_x = 0.0, linear_previous_error_y = 0.0, linear_previous_error_z = 0.0, rotational_previous_error_z = 0.0;
      if ( dt > 0.1 )
        {
          // Reset
          linear_previous_error_x = 0.0;
          linear_previous_error_y = 0.0;
          linear_previous_error_z = 0.0;
          rotational_previous_error_z = 0.0;
        }
      double linear_derivative_x = ( linear_error_x - linear_previous_error_x ) / dt;
      double linear_derivative_y = ( linear_error_y - linear_previous_error_y ) / dt;
      double linear_derivative_z = ( linear_error_z - linear_previous_error_z ) / dt;
      double rotational_derivative_z = ( rotational_error_z - rotational_previous_error_z ) / dt;
      linear_previous_error_x = linear_error_x;
      linear_previous_error_y = linear_error_y;
      linear_previous_error_z = linear_error_z;
      rotational_previous_error_z = rotational_error_z;

      cout << "error_x : " << linear_error_x <<", error_y : " << linear_error_y << ", error_z : " << linear_error_z << ", rotational_error : " << rotational_error_z << endl;
      if ( linear_error_x > 0.6 || ( linear_error_y > 0.05 || linear_error_y < -0.05 ) )
        {
          kp_roll_pitch = 0.5; //  0.35
          ki_roll_pitch = 0;
          kd_roll_pitch = 0.32;  //0.35
        }

      if ( rotational_error_z > 0.1 || rotational_error_z < - 0.1 )
        {
          kp_yaw = 0.02; //0.5
          ki_yaw = 0.0; //0.0
          kd_yaw = 0.0; //0.1
        }

      if ( linear_error_z < -0.05 ||linear_error_z > 0.05 )
        {
          kp_gaz = 0.6; //0.5
          ki_gaz = 0.01; //0.1
          kd_gaz = 0.2; //0.3
        }

      vel_msg.linear.x = kp_roll_pitch * linear_error_x + ki_roll_pitch * linear_integral_x + kd_roll_pitch * linear_derivative_x;
      vel_msg.linear.y = kp_roll_pitch * linear_error_y + ki_roll_pitch * linear_integral_y + kd_roll_pitch * linear_derivative_y;
      vel_msg.linear.z = - ( kp_gaz * linear_error_z + ki_gaz * linear_integral_z + kd_gaz * linear_derivative_z );

      vel_msg.angular.z = - ( kp_yaw * rotational_error_z + ki_yaw * rotational_integral_z + kd_yaw * rotational_derivative_z );

      errors_msg.x = linear_error_x;
      errors_msg.y = linear_error_y;
      errors_msg.z = linear_error_z;


      //cout << "kp = " << kp << " , ki = " << ki << " , kd = " << kd << endl;
      cout << "(vx, vy, vz) : " << "(" << vel_msg.linear.x << ", " << vel_msg.linear.y <<", " << vel_msg.linear.z<< ")" << endl;
      cout << "Rotational speed on z : " << vel_msg.angular.z << endl;
      cout << "------------------------------------" << endl;


      vel_pub.publish ( vel_msg );
      errors_pub.publish<> ( errors_msg );

      vel_msg.linear.x = vel_msg.linear.y = 0;
      vel_msg.angular.z = 0.0;

      rate.sleep();
    }



  ros::spin();
  return 0;

}
