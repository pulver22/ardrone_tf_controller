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
#include "../include/offset.h"





int last_msg = 0;
const double PI = 3.141593;
unsigned int ros_header_timestamp_base = 0;

using namespace std;

void updateOffset ( Offset* offset, double target_X, double target_Y, double target_Z, double target_yaw );
int getCompensatoryFactor( Offset* current_offset, Offset* last_offset, int count );

void markerCallback ( const ar_pose::ARMarker::ConstPtr msg )
{

        ROS_INFO ( "Marker detected!" );
        last_msg = msg->header.stamp.sec;
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
        ros::Subscriber marker_sub = nh.subscribe ( "/ar_pose_marker",1, markerCallback );
        ros::Publisher linear_offset_pub = nh.advertise<geometry_msgs::Vector3> ( "/linear_offset",100 );
        ros::Publisher rotational_offset_pub = nh.advertise<std_msgs::Float32> ( "/rotation_offset",100 );
        geometry_msgs::Vector3 offset_msg;
        std_msgs::Float32 yaw_msg;

        // Save the translation and rotational offsets on three axis
        Offset current_offset;
        current_offset.setRoll ( 0 );
        current_offset.setPitch ( 0 );
        current_offset.setGaz ( 0 );
        current_offset.setYaw ( 0 );
        Offset last_view_offset;
        last_view_offset.setOffset ( current_offset );
        int count = 0;
        int K_compensatory;

        tfScalar  roll, pitch, yaw;


        float target_X, target_Y,target_Z, target_yaw;
        float epsilon;

        nh.getParam ( "/ardrone_tf_controller/target_X",target_X );	// expressed in meters (default: 0.1)
        nh.getParam ( "/ardrone_tf_controller/target_Y",target_Y );	// expressed in meters (default: 0.1)
        nh.getParam ( "/ardrone_tf_controller/target_Z",target_Z );	// expressed in meters (default: 0.4)
        nh.getParam ( "/ardrone_tf_controller/target_yaw",target_yaw );	// expressed in degress (default: 5.0)
        nh.getParam ( "/ardrone_tf_controller/epsilon",epsilon );		// error variable on rotation expressed in radiants (default: 0.78)


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


                if ( transformStamped.transform.translation.z > 0 ) {
                        current_offset.setRoll ( - transformStamped.transform.translation.x );	// WORKING
                        current_offset.setPitch ( transformStamped.transform.translation.z );	// WORKING
                        current_offset.setGaz ( - transformStamped.transform.translation.y );
                        current_offset.setYaw ( ( float ) pitch * 180 / PI );

                } else {
                        ROS_WARN ( " Branch unsafe!" );
                        current_offset.setRoll ( transformStamped.transform.translation.x );	// WORKING
                        current_offset.setPitch ( - transformStamped.transform.translation.z );	// WORKING
                        current_offset.setGaz ( - transformStamped.transform.translation.y );
                        current_offset.setYaw ( ( float ) pitch * 180 / PI );
                }


                /* Plot the offsets calculated with TF
                 */
                offset_msg.x = ( current_offset.getRoll() );
                offset_msg.y = ( current_offset.getPitch() );
                offset_msg.z = ( current_offset.getGaz() )	;
                linear_offset_pub.publish ( offset_msg );
                yaw_msg.data = current_offset.getYaw();
                rotational_offset_pub.publish<> ( yaw_msg );
                // ------------------------------------


                std_msgs::String clear, autoinit,takeoff,goTo, moveBy ,land, reference, maxControl, initialReachDist, stayWithinDist, stayTime;

                // Modify current_offset in proximity of the target
				Offset* current_offset_ptr = &current_offset;
				Offset* last_view_offset_ptr = &last_view_offset;
                updateOffset ( current_offset_ptr,target_X,target_Y,target_Z,target_yaw ) ;

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

                moveBy.data = "c moveByRel " + boost::lexical_cast<std::string> ( current_offset.getRoll() ) + " " + boost::lexical_cast<std::string> ( current_offset.getPitch() ) + " " +
                              boost::lexical_cast<std::string> ( current_offset.getGaz() ) + " " + boost::lexical_cast<std::string> ( current_offset.getYaw() ) ;
                land.data = "c land";

                int currentTimeInSec = ( int ) ros::Time::now().sec;

                if ( moveBy.data.compare ( "c moveByRel 0 0 0 0" ) == 0 ) {
                        ROS_INFO ( "Destination reached" );
                        cmd_pub.publish<> ( moveBy );
                        //cmd_pub.publish<> ( land );
                        //ros::shutdown();
                }

                /* If the time stamp of the last message is equal to the current time, it means the marker has been correctly detected,
                 * otherwise it has been lost
                 */

                K_compensatory = 1;

                if ( last_msg == 0 ) {
                        // do nothing
                } else {

                        if ( currentTimeInSec == last_msg ) {
                                cmd_pub.publish<> ( clear );
                                if ( count == 1 ) {
                                        ROS_INFO ( "Initialization" );
                                        cmd_pub.publish<> ( autoinit );
                                        cmd_pub.publish<> ( reference );
                                        cmd_pub.publish<> ( maxControl );
                                        cmd_pub.publish<> ( initialReachDist );
                                        cmd_pub.publish<> ( stayWithinDist );
                                        cmd_pub.publish<> ( stayTime );
                                        //cmd_pub.publish<> ( takeoff );
                                } else {

                                        cout << "Last X offset : " << last_view_offset.getRoll() << ", current X offset : " << current_offset.getRoll() << endl;
                                        cout << "count : " << count << endl;
                                        K_compensatory = getCompensatoryFactor(current_offset_ptr, last_view_offset_ptr, count);

                                        moveBy.data = "c moveByRel " + boost::lexical_cast<std::string> ( K_compensatory * current_offset.getRoll() ) + " " + boost::lexical_cast<std::string> ( current_offset.getPitch() ) + " " +
                                                      boost::lexical_cast<std::string> ( current_offset.getGaz() ) + " " + boost::lexical_cast<std::string> ( current_offset.getYaw() ) ;

                                        cmd_pub.publish<> ( moveBy );
                                        //cout << "alive" << endl;
                                        cout << moveBy.data << endl;
										last_view_offset.setOffset ( current_offset );
                                }
                                
                                cout <<  endl;

                        } else {

                                /* TODO: this solution doesn't work always. Sometimes the marker is detected but the timestamp don't are equal: like one message has been lost
                                 *
                                 */
                                ROS_ERROR ( "Marker is lost!" );
                                ROS_ERROR ( "CurrentTime is %d while last_msg is %d", currentTimeInSec, last_msg );
                                cmd_pub.publish<> ( clear );
                                if ( count == 1 ) {
                                        cmd_pub.publish<> ( autoinit );
                                        cmd_pub.publish<> ( reference );
                                        cmd_pub.publish<> ( maxControl );
                                        cmd_pub.publish<> ( initialReachDist );
                                        cmd_pub.publish<> ( stayWithinDist );
                                        cmd_pub.publish<> ( stayTime );
                                        //cmd_pub.publish<> ( takeoff );
                                }
                                //moveBy.data = "c moveByRel " + boost::lexical_cast<std::string> ( 2 * current_offset.getRoll() ) + " " + boost::lexical_cast<std::string> ( 2 * current_offset.getPitch() ) + " " +
                                //              boost::lexical_cast<std::string> ( 2 * current_offset.getGaz() ) + " " + boost::lexical_cast<std::string> ( 2 * current_offset.getYaw() ) ;

                                moveBy.data = "c moveByRel 0 -0.5 0 0";
                                cmd_pub.publish<> ( moveBy );
                                cout << moveBy.data << endl;
                                cout <<  endl;
                        }
                }
                count++;
				//delete current_offset_ptr;
				//delete last_view_offset_ptr;
                rate.sleep();
                ros::spinOnce();
        }

        return 0;

}

void updateOffset ( Offset* offset, double target_X, double target_Y, double target_Z, double target_yaw )
{

        if ( abs ( offset->getRoll() ) < target_X ) {
                offset->setRoll ( 0 );
        }

        if ( abs ( offset->getPitch() ) < target_Y ) {
                offset->setPitch ( 0 );
        }

        if ( abs ( offset->getGaz() ) < target_Z ) {
                offset->setGaz ( 0 );
        }

        if ( abs ( offset->getYaw() ) < target_yaw ) {
                offset->setYaw ( 0 );
        }
 
}


int getCompensatoryFactor ( Offset* current_offset, Offset* last_offset, int count )
{
        int tmpCompensatoryFactor = 1;
        if ( last_offset->getRoll() * current_offset->getRoll() <=0 && count > 1 ) {

                if ( current_offset->getRoll() > 0 ) {
                        ROS_WARN ( "Virtual X axis crossed #1" );
                        tmpCompensatoryFactor =  10;
                } else if ( current_offset->getRoll() < 0 ) {
                        ROS_WARN ( "Virtual X axis crossed #2" );
                        tmpCompensatoryFactor = -10;
				}

        }
        return tmpCompensatoryFactor;
}
