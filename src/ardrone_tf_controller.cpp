#include <ros/ros.h>
#include <ros/node_handle.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <ar_pose/ARMarker.h>
#include <tf2/transform_datatypes.h>
#include <tf2_ros/transform_listener.h>


using namespace std;

void printPose ( const ar_pose::ARMarker::ConstPtr &msg )
{
        /*tf::Pose marker_pose_in_camera;


        marker_pose_in_camera.setOrigin (tf::Vector3 (msg->pose.pose.position.x,
                                         msg->pose.pose.position.y,
                                         msg->pose.pose.position.z));

        */
        cout<<"-------------"<<endl;
        cout<<"x :" << msg->pose.pose.position.x<<endl;
        cout<<"y :" << msg->pose.pose.position.y<<endl;
        cout<<"z :" << msg->pose.pose.position.z<<endl;
        cout<<"-------------"<<endl;
}



int main ( int argc, char **argv )
{
        ros::init ( argc, argv,"pose_subscriber" );
        ROS_INFO ( "TF subscriber correctly running..." );
        ros::NodeHandle nh;
        //ros::Subscriber pose_sub=nh.subscribe<> ( "ar_pose_marker",1000,printPose );
        ros::Publisher drone_vel =nh.advertise<geometry_msgs::Twist> ( "/cmd_vel", 10 );
        tf2_ros::Buffer tfBuffer;
        tf2_ros::TransformListener tfListener ( tfBuffer );
        ros::Rate rate ( 2.0 );

        while ( nh.ok() ) {
                geometry_msgs::TransformStamped transformStamped;

                try {
                        transformStamped = tfBuffer.lookupTransform ( "ar_marker","ardrone_base_frontcam",ros::Time ( 0 ) );
                } catch ( tf2::TransformException &ex ) {
                        ROS_WARN ( "%s",ex.what() );
                        ros::Duration ( 1.0 ).sleep();
                        continue;
                }
                
                // Save the translation offset on three axis
		float offset_X, offset_Y, offset_Z;
		offset_X = transformStamped.transform.translation.x;
		offset_Y = transformStamped.transform.translation.y;
		offset_Z = transformStamped.transform.translation.z;
		
                cout<<" TF transform between marker and camera frames"<<endl;
                cout<<"X: "<<offset_X<<endl;
                cout<<"Y: "<<offset_Y<<endl;
                cout<<"Z: "<<offset_Z<<endl;
		
		// Create the twist message
                geometry_msgs::Twist vel_msg;
		
		// Adjust distance on the global X axis - z for marker and camera
		if(offset_Z > 0.6){
			vel_msg.linear.x = 0.5;
		}else if(offset_Z < -0.6){
			vel_msg.linear.x = -0.5;
		}else vel_msg.linear.x = 0.0;
		
		
		// Adjust distance on the global y axis - x for marker and camera
		if(offset_X > 0.3){
			vel_msg.linear.y = 0.5;
		}else if (offset_X < -0.3){
			vel_msg.linear.y = -0.5;
		}else vel_msg.linear.y = 0.0;
		
		
		// Adjust distance on the global z axis - y for marker and camera
		if(offset_Y > 0.3){
			vel_msg.linear.z = 0.5;
		}else if (offset_Y < -0.3){
			vel_msg.linear.z = -0.5;
		}else vel_msg.linear.z = 0.0;
		
		/*TODO
		 * Controllers on position work alone, not all together (not stable)
		 * Controllers on rotations to be implemented
		 */
                drone_vel.publish ( vel_msg );

                rate.sleep();
        }



        ros::spin();
        return 0;

}
