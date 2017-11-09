/*
  The MIT License (MIT)
  Copyright (c) 2017 Riccardo Polvara

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
  #MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
  #CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
  #SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

  Main class for the deep reinforced landing node.
*/
#include <math.h>
#include <stdlib.h>
#include <string>
#include <gazebo_msgs/GetModelState.h>
#include <gazebo_msgs/SetModelState.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <ros/node_handle.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <tf/transform_datatypes.h>
#include "ardrone_tf_controller/ResetPosition.h"
//#include "ardrone_tf_controller/SendCommand.h"
#include "ardrone_tf_controller/GetRelativePose.h"
using namespace std;

const double PI = 3.14159265359;

class USVController
{
  private:

    ros::NodeHandle nh_;

    /////////////////////////////////////////////
    //               PUBLISHERS                //
    /////////////////////////////////////////////
    // Publishers for moving the USV
    ros::Publisher cmd_pub_;
    // Publisher for setting/resetting USV's pose
    ros::Publisher reset_model_pub_;

    /////////////////////////////////////////////
    //                SERVICES                 //
    /////////////////////////////////////////////
    // Service for getting the USV's pose wrt to quadrotor's one
    ros::ServiceServer service_relative_pose_;
    // Service for getting the reset request...
    ros::ServiceServer service_reset_;

    // --- and the call the service offered by gazebo
    ros::ServiceClient set_state_client_;

    // Client for getting USV's position
    ros::ServiceClient get_state_client_;

    /////////////////////////////////////////////
    //         CALLBACKS AND SERVICES          //
    /////////////////////////////////////////////
        bool getRelativePose(ardrone_tf_controller::GetRelativePose::Request &req,
                          ardrone_tf_controller::GetRelativePose::Response &res);

    /////////////////////////////////////////////
    //                  DATA                   //
    /////////////////////////////////////////////

    // Server for getting USV's pose and various related variables
    gazebo_msgs::GetModelState srv_;
    geometry_msgs::Pose quadrotor_pose_, usv_pose_, marker_pose_, quadrotor_to_marker_pose_;
    geometry_msgs::Pose start_pose_;
    geometry_msgs::Twist start_twist_;

    // USV's control related variables
    geometry_msgs::Twist velocity_cmd_;

    bool reset_;
  protected:

  public:

    USVController();
    ~USVController();

    bool getReset();
    void setReset(bool reset);

    /*
      Generate a pose for the USV.
      @return the pose of the USV expressed as position (x,y,z) and orientation (x,y,w,z)
    */
    gazebo_msgs::SetModelState getUpdatedModelState(double roll, double pitch, double yaw);
    void getRelativeModelState();

    /*
      Assign the USV a new pose in thew world
      @param set_model_state is the new pose expressed as position(x,y,z) and orientation (x,y,w,z)
    */ 
    void setModelState(gazebo_msgs::SetModelState set_model_state);

    ros::Publisher getCmdPub();
    geometry_msgs::Twist getVelocityCmd();
};

USVController::USVController()
{
  cmd_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel",1);
  reset_model_pub_ = nh_.advertise<gazebo_msgs::ModelState>("/gazebo/set_model_state",1);

  get_state_client_ = nh_.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");
  set_state_client_ = nh_.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
  service_relative_pose_ = nh_.advertiseService("ardrone_tf_controller/get_relative_pose", &USVController::getRelativePose, this);

}

USVController::~USVController()
{
}


bool USVController::getRelativePose(ardrone_tf_controller::GetRelativePose::Request &req,
                                            ardrone_tf_controller::GetRelativePose::Response &res)
{
  // NOTE: the relative position is calculated within the mathod for the reward (setReward). Therefore that method need to be 
  //called before this one in order to have the relative pose of the quadrotor to the marker
  res.pose.position.x = quadrotor_to_marker_pose_.position.x;
  res.pose.position.y = quadrotor_to_marker_pose_.position.y;
  res.pose.position.z = quadrotor_to_marker_pose_.position.z;
  return true;
}

bool USVController::getReset()
{
  return reset_;
}

void USVController::setReset(bool reset)
{
  reset_ = reset;
}

gazebo_msgs::SetModelState USVController::getUpdatedModelState(double new_roll, double new_pitch, double new_yaw)
{
  //TODO: Get actual position and change roll/pitch accordingly
  gazebo_msgs::SetModelState set_model_state;
  tfScalar current_roll, current_pitch, current_yaw;
  tf::Matrix3x3 m;
  tf::Quaternion orientation;

  srv_.request.model_name = "usv_model";
  if(get_state_client_.call(srv_))
  {
    usv_pose_.position.x = srv_.response.pose.position.x;
    usv_pose_.position.y = srv_.response.pose.position.y;
    usv_pose_.position.z = srv_.response.pose.position.z;
    usv_pose_.orientation.x = srv_.response.pose.orientation.x;
    usv_pose_.orientation.y = srv_.response.pose.orientation.y;
    usv_pose_.orientation.z = srv_.response.pose.orientation.z;
    usv_pose_.orientation.w = srv_.response.pose.orientation.w;
  
    tf::Quaternion orientation(usv_pose_.orientation.x, usv_pose_.orientation.y, usv_pose_.orientation.z, usv_pose_.orientation.w);
    m.setRotation(orientation);
    m.getRPY(current_roll, current_pitch, current_yaw);
    current_roll = current_roll * 180 / PI;
    current_pitch = current_pitch * 180 / PI;
    current_yaw = current_yaw * 180 / PI;
    std::cout << "Current USV position: x=" << usv_pose_.position.x << ", y=" << usv_pose_.position.y << ", z= " << usv_pose_.position.z << std::endl;
    std::cout << "Current USV orientation (degrees): R=" << current_roll << ", P=" << current_pitch << " , Y=" << current_yaw << endl;  
  }
  else
  {
    ROS_ERROR("Service has not been called!");
  }
  
  // Define the new rotation on the USV based on the argument
  m.setEulerYPR(new_yaw, new_pitch, new_roll);
  m.getRotation(orientation);
  
  set_model_state.request.model_state.model_name = "usv_model";
  set_model_state.request.model_state.pose.position.x = usv_pose_.position.x;
  set_model_state.request.model_state.pose.position.y = usv_pose_.position.y;
  set_model_state.request.model_state.pose.position.z = usv_pose_.position.z;
  set_model_state.request.model_state.pose.orientation.x = orientation.getX();
  set_model_state.request.model_state.pose.orientation.y = orientation.getY();
  set_model_state.request.model_state.pose.orientation.z = orientation.getZ();
  set_model_state.request.model_state.pose.orientation.w = orientation.getW();

  return set_model_state;
}

void USVController::getRelativeModelState()
{
  srv_.request.model_name = "quadrotor";
  if (get_state_client_.call(srv_))
  {  // NB: quadrotor's altitude can be used to understand if it still flying or landed
    quadrotor_pose_.position.x = srv_.response.pose.position.x;
    quadrotor_pose_.position.y = srv_.response.pose.position.y;
    quadrotor_pose_.position.z = srv_.response.pose.position.z;
  }
  else
  {
    ROS_ERROR("Service has not been called");
  }

  srv_.request.model_name = "usv_model";
  if (get_state_client_.call(srv_))
  {
    marker_pose_.position.x = srv_.response.pose.position.x;
    marker_pose_.position.y = srv_.response.pose.position.y;
    marker_pose_.position.z = srv_.response.pose.position.z;
  }
  else
  {
    ROS_ERROR("Service has not been called");
  }

  //Calculate the quadrotor pose wrt the marker's one
  quadrotor_to_marker_pose_.position.x = quadrotor_pose_.position.x - marker_pose_.position.x;
  quadrotor_to_marker_pose_.position.y = quadrotor_pose_.position.y - marker_pose_.position.y;
  quadrotor_to_marker_pose_.position.z = quadrotor_pose_.position.z - marker_pose_.position.z;
}

void USVController::setModelState(gazebo_msgs::SetModelState set_model_state)
{
  set_state_client_.call(set_model_state);
}

ros::Publisher USVController::getCmdPub()
{
  return cmd_pub_;
}

geometry_msgs::Twist USVController::getVelocityCmd()
{
  return velocity_cmd_;
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "usv_controller_node");
  USVController usv_controller_node;
  gazebo_msgs::SetModelState tmp_model_state;
  
  float roll, pitch, yaw, time;
  
  if (argc > 3)
  {
    roll = atof(argv[1]) * PI / 180;
    pitch = atof(argv[2]) * PI / 180;
    yaw = atof(argv[3]) * PI / 180;
    time = atof(argv[4]);
  }
  else
  {
    std::cout << "You need to pass three values (degrees) as argument for RPY. Exiting now!" << std::endl;
    exit(EXIT_FAILURE);
  }

  ros::Rate rate(1/time);

  while(ros::ok())
  {

    // Calculate the relative pose of the quadrotor to the USV at every iteration
    usv_controller_node.getRelativeModelState();

    // Get the actual pose of the USV and generate a new one
    tmp_model_state = usv_controller_node.getUpdatedModelState(roll, pitch, yaw);
    usv_controller_node.setModelState(tmp_model_state);

    ros::spinOnce();
    rate.sleep();
  }
}
