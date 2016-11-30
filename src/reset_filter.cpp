#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include "ros/ros.h"

using namespace std;

int main( int argc, char **argv)
{
    ros::init( argc, argv, "");
    ros::NodeHandle nh;
    ros::Publisher reset_filter_pub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/husky/set_pose", 1000);

    geometry_msgs::PoseWithCovarianceStamped pose_with_cov_st;

    // set message's header
    pose_with_cov_st.header.stamp = ros::Time::now();
    pose_with_cov_st.header.frame_id = "/odom";

    // set position
    pose_with_cov_st.pose.pose.position.x = 0;
    pose_with_cov_st.pose.pose.position.y = 0;
    pose_with_cov_st.pose.pose.position.z = 0;

    // set orientation
    pose_with_cov_st.pose.pose.orientation.x = 0;
    pose_with_cov_st.pose.pose.orientation.y = 0;
    pose_with_cov_st.pose.pose.orientation.z = 0;
    pose_with_cov_st.pose.pose.orientation.w = 1;

    // publish message to reset UKF filter
    reset_filter_pub.publish(pose_with_cov_st);
    ROS_INFO_STREAM("Message to reset UKF filter sent");
}

