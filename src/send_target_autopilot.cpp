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
#include <boost/lexical_cast.hpp>


unsigned int ros_header_timestamp_base = 0;

using namespace std;


int main ( int argc, char **argv )
{
        ros::init ( argc, argv,"pose_subscriber" );
        ROS_INFO ( "TF subscriber correctly running..." );
        ros::NodeHandle nh;
        //ros::Subscriber pose_sub=nh.subscribe<> ( "ar_pose_marker",1000,printPose );
        //ros::Publisher vel_pub =nh.advertise<geometry_msgs::Twist> ( "/cmd_vel", 10 );
        tf2_ros::Buffer tfBuffer;
        tf2_ros::TransformListener tfListener ( tfBuffer );
        ros::Rate rate ( 1.0 );
        //ros::Publisher errors_pub = nh.advertise<geometry_msgs::Vector3> ( "/position_errors",10 );
        ros::Publisher cmd_pub = nh.advertise<std_msgs::String> ( "/uga_tum_ardrone/com",10 );
        //DroneController controller;
        //DronePosition target;
        int count = 1;

        while ( nh.ok() ) {
                geometry_msgs::TransformStamped transformStamped;
                geometry_msgs::Vector3 errors_msg;
                try {
                        transformStamped = tfBuffer.lookupTransform ( "ar_marker","ardrone_base_frontcam",ros::Time ( 0 ) );
                } catch ( tf2::TransformException &ex ) {
                        ROS_WARN ( "%s",ex.what() );
                        ros::Duration ( 1.0 ).sleep();
                        continue;
                }

                // Save the translation and rotational offsets on three axis
                float linear_offset_X, linear_offset_Y, linear_offset_Z;
                float rotational_offset_Z;

                // Adjust distance on the global X axis - z for marker and camera
                // Adjust distance on the global y axis - x for marker and camera
                // Adjust distance on the global z axis - y for marker and camera
                linear_offset_X = transformStamped.transform.translation.x;
                linear_offset_Y = transformStamped.transform.translation.y;
                linear_offset_Z = transformStamped.transform.translation.z;
                rotational_offset_Z = transformStamped.transform.rotation.z;


                std_msgs::String clear, autoinit,takeoff,move,land, reference, maxControl, initialReachDist, stayWithinDist, stayTime;
                
		
                
		clear.data = "c clearCommands";

                autoinit.data = "c autoInit 500 800";
                reference.data = "setReference $POSE$";
                maxControl.data = "setMaxControl 1";
                initialReachDist.data = "setInitialReachDist 0.2";
                stayWithinDist.data = "setStayWithinDist 0.5";
                stayTime.data = "setStayTime 2";

                takeoff.data = "c takeoff";
		
		/* TODO
		 * Fix axis correspondences: /uga_tum_ardrone/com apparently work with some weird reference system different from base_link
		 */
                move.data = "c goto " + boost::lexical_cast<std::string>(linear_offset_X) + " " + boost::lexical_cast<std::string>(linear_offset_Y) + " " + 
			boost::lexical_cast<std::string>(linear_offset_Z) + " " + boost::lexical_cast<std::string>(rotational_offset_Z) ;
                land.data = "c land";

                if ( count ==  1 ) {

                        cmd_pub.publish<> ( clear );

                        cmd_pub.publish<> ( autoinit );
                        cmd_pub.publish<> ( reference );
                        cmd_pub.publish<> ( maxControl );
                        cmd_pub.publish<> ( initialReachDist );
                        cmd_pub.publish<> ( stayWithinDist );
                        cmd_pub.publish<> ( stayTime );
                        cmd_pub.publish<> ( takeoff );	// mandatory for taking off (despite in the gui the drone takes off also using only the autoInit)
                        cmd_pub.publish<> ( move );
                        cmd_pub.publish<> ( land );
                        
                        count ++;
			//cout << linear_offset_X << " " << linear_offset_Y << " " << linear_offset_Z << " " << rotational_offset_Z << endl;
                        //cout << move.data << endl;
                }

                if ( count == 2 ) {
                        ROS_INFO ( "All commands are already sent!" );
                        count ++;
                }
                





                rate.sleep();
        }



        ros::spin();
        return 0;

}
