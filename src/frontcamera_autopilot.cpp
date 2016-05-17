#include "ros/ros.h"
#include "ros/package.h"
#include <ros/node_handle.h>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Twist.h>
#include "std_msgs/String.h"
#include <ar_pose/ARMarker.h>
//#include "../catkin_ws2/devel/include/ar_pose/ARMarker.h"
//#include "../catkin_ws2/devel/include/ar_pose/ARMarkers.h"
#include <tf2/transform_datatypes.h>
#include <tf2_ros/transform_listener.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <boost/lexical_cast.hpp>
#include "sensor_msgs/ChannelFloat32.h"
#include "std_msgs/Float32.h"




int last_msg = 0;
const double PI = 3.141593;
unsigned int ros_header_timestamp_base = 0;

using namespace std;

void marker_callback ( const ar_pose::ARMarker::ConstPtr msg )
{

        ROS_INFO ( "Marker detected!" );
        last_msg = msg->header.stamp.sec;
	// Marker's position wrt camera
	//float x = msg->pose.pose.position.x;
	//float y = msg->pose.pose.position.y;
	//float z = msg->pose.pose.position.z;
	//ROS_INFO("X: %f , Y: %f , Z: %f", x,y,z);
}

int main ( int argc, char **argv )
{
        ros::init ( argc, argv,"pose_subscriber" );
        ROS_INFO ( "TF subscriber correctly running..." );
        ros::NodeHandle nh;
        tf2_ros::Buffer tfBuffer;
        tf2_ros::TransformListener tfListener ( tfBuffer );
        ros::Rate rate ( 1.0 );
        ros::Publisher cmd_pub = nh.advertise<std_msgs::String> ( "/uga_tum_ardrone/com",100 );
        ros::Subscriber marker_sub = nh.subscribe ( "/ar_pose_marker",1, marker_callback );
	ros::Publisher linear_offset_pub = nh.advertise<geometry_msgs::Vector3>("/linear_offset",100);
	ros::Publisher rotational_offset_pub = nh.advertise<std_msgs::Float32>("/rotation_offset",100);
        
        // Save the translation and rotational offsets on three axis
        float linear_offset_X, linear_offset_Y, linear_offset_Z;
        linear_offset_X = linear_offset_Y = linear_offset_Z = 0;
        float rotational_offset_Z;
	geometry_msgs::Vector3 offset;
	std_msgs::Float32 yaw_error;
	
	int multiplier = 1;
	float last_offset_X = 0;
	

        tfScalar  roll, pitch, yaw;


        float target_X, target_Y,target_Z, target_yaw;
        float epsilon;
	
	nh.getParam("/ardrone_tf_controller/target_X",target_X);	// expressed in meters (default: 0.1)
	nh.getParam("/ardrone_tf_controller/target_Y",target_Y);	// expressed in meters (default: 0.1)
	nh.getParam("/ardrone_tf_controller/target_Z",target_Z);	// expressed in meters (default: 0.4)
	nh.getParam("/ardrone_tf_controller/target_yaw",target_yaw);	// expressed in degress (default: 5.0)
	nh.getParam("/ardrone_tf_controller/epsilon",epsilon);		// error variable on rotation expressed in radiants (default: 0.78)
	

        while ( nh.ok() ) {
                geometry_msgs::TransformStamped transformStamped;
                try {
                        transformStamped = tfBuffer.lookupTransform ( "ar_marker","ardrone_base_frontcam",ros::Time ( 0 ) );
                        //transformStamped = tfBuffer.lookupTransform( "ar_marker", "base_link", ros::Time(0));
                } catch ( tf2::TransformException &ex ) {
                        ROS_WARN ( "%s",ex.what() );
                        ros::Duration ( 1.0 ).sleep();
                        continue;
                }

		
		/* Get rotation expressed in quaternion, transform it in a rotation matrix and then retrieve roll pitch and yaw
		 */
                tf::Quaternion q ( transformStamped.transform.rotation.x, transformStamped.transform.rotation.y, transformStamped.transform.rotation.z, transformStamped.transform.rotation.w );
                tf::Matrix3x3 m ( q );
                m.getRPY ( roll, pitch, yaw );
		
		/* TODO: Fix drift on X due to rotation (as for bottomcam)
		 */
		

		if(transformStamped.transform.translation.z > 0){
			ROS_WARN("Drone above the marker's horizon");
			multiplier = -1;
		}
		
		linear_offset_X =  multiplier * transformStamped.transform.translation.x;	// WORKING
                linear_offset_Y = - multiplier * transformStamped.transform.translation.z;	// WORKING
                linear_offset_Z =  multiplier * transformStamped.transform.translation.y;
                rotational_offset_Z = ( ( float ) pitch * 180 / PI );
		
			/*
                if ( transformStamped.transform.translation.z > 0 ) {
                        linear_offset_X = - transformStamped.transform.translation.x;	// WORKING
                        linear_offset_Y =  transformStamped.transform.translation.z;	// WORKING
			linear_offset_Z = - transformStamped.transform.translation.y;
			rotational_offset_Z = ( ( float ) pitch * 180 / PI );

                } else {
                        linear_offset_X =  transformStamped.transform.translation.x;	// WORKING
                        linear_offset_Y = - transformStamped.transform.translation.z;	// WORKING
			linear_offset_Z =  transformStamped.transform.translation.y;
                        rotational_offset_Z = ( ( float ) pitch * 180 / PI );
                }
                */
                
                if(linear_offset_Z > 0){
			ROS_WARN ("Drone is ascending!!");
                }
                
                /*
                if ((linear_offset_X * last_offset_X < 0) ){
				ROS_WARN("Multiplier changed!");
				linear_offset_X = - linear_offset_X;
		}*/
                        
                last_offset_X = linear_offset_X;
                
                /* Plot the offsets calculated with TF
		 */
		offset.x = (linear_offset_X);
		offset.y = (linear_offset_Y);
		offset.z = (linear_offset_Z)	;
		linear_offset_pub.publish(offset);
		yaw_error.data = rotational_offset_Z;
		rotational_offset_pub.publish<>(yaw_error);
		// ------------------------------------
		
		
                std_msgs::String clear, autoinit,takeoff,goTo, moveBy ,land, reference, maxControl, initialReachDist, stayWithinDist, stayTime;

                if ( abs ( linear_offset_X ) < target_X ) {
                        linear_offset_X = 0;
                }

                if ( abs ( linear_offset_Y ) < target_Y ) {
                        linear_offset_Y = 0;
                }

                if ( abs ( linear_offset_Z ) < target_Z ) {
                        linear_offset_Z = 0;
                }

                if ( abs ( rotational_offset_Z ) < target_yaw ) {
                        rotational_offset_Z = 0;
                }

                clear.data = "c clearCommands";

                autoinit.data = "c autoInit 500 800";
                reference.data = "setReference $POSE$";
                maxControl.data = "setMaxControl 1";
                initialReachDist.data = "setInitialReachDist 0.1";
                stayWithinDist.data = "setStayWithinDist 0.1";
                stayTime.data = "setStayTime 0.5";

                takeoff.data = "c takeoff";


                //goTo.data = "c goto " + boost::lexical_cast<std::string> ( linear_offset_X ) + " " + boost::lexical_cast<std::string> ( linear_offset_Y ) + " " +
                //            boost::lexical_cast<std::string> ( linear_offset_Z ) + " " + boost::lexical_cast<std::string> ( rotational_offset_Z ) ;

                moveBy.data = "c moveByRel " + boost::lexical_cast<std::string> ( linear_offset_X ) + " " + boost::lexical_cast<std::string> ( linear_offset_Y ) + " " +
                              boost::lexical_cast<std::string> ( linear_offset_Z ) + " " + boost::lexical_cast<std::string> ( rotational_offset_Z ) ;
                land.data = "c land";

		int currentTimeInSec = ( int ) ros::Time::now().sec;

                if ( moveBy.data.compare ( "c moveByRel 0 0 0 0" ) == 0 ) {
                        ROS_INFO ( "Destination reached" );
                        //cmd_pub.publish<> ( land );
                        ros::shutdown();
                }

                /* If the time stamp of the last message is equal to the current time, it means the marker has been correctly detected,
                 * otherwise it has been lost
                 */
		
		
		if ( last_msg == 0 ) {
			// do nothing
                } else {

                        if ( currentTimeInSec == last_msg ) {
                                cmd_pub.publish<> ( clear );
                                cmd_pub.publish<> ( autoinit );
                                cmd_pub.publish<> ( reference );
                                cmd_pub.publish<> ( maxControl );
                                cmd_pub.publish<> ( initialReachDist );
                                cmd_pub.publish<> ( stayWithinDist );
                                cmd_pub.publish<> ( stayTime );
                                //cmd_pub.publish<> ( takeoff );
                                cmd_pub.publish<> ( moveBy );

                                //cout << "alive" << endl;
                                cout << moveBy.data << endl;
                                cout <<  endl;
                        } else {
                                ROS_ERROR ( "Marker is lost!" );
                                cmd_pub.publish<> ( clear );
                                cmd_pub.publish<> ( autoinit );
                                cmd_pub.publish<> ( reference );
                                cmd_pub.publish<> ( maxControl );
                                cmd_pub.publish<> ( initialReachDist );
                                cmd_pub.publish<> ( stayWithinDist );
                                cmd_pub.publish<> ( stayTime );
                                //cmd_pub.publish<> ( takeoff );
                                moveBy.data = "c moveByRel 0 -0.5 0 0";
                                cmd_pub.publish<> ( moveBy );
                                cout << moveBy.data << endl;
                                cout <<  endl;
                        }
                }
                linear_offset_X = linear_offset_Y = linear_offset_Z = 0;
                rate.sleep();
                ros::spinOnce();
        }

        return 0;

}
