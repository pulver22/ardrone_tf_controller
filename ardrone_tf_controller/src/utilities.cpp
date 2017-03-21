#include <string>
#include <unordered_map>

#include "../include/utilities.h"
#include <geometry_msgs/Point.h>
#include <tf/LinearMath/Matrix3x3.h>

Utilities::Utilities()
{
}

Utilities::~Utilities()
{
}

//Sum the EKF prediction to the last observation
Offset Utilities::UpdateUGVPosition(int current_time_stamp, int old_time_stamp, std::unordered_map<int, std::string> *ts_map)
{

  //std::cout << "CT: " << ts_map->begin()->first << ", OT: " << old_time_stamp << std::endl;

  // Get actual and old UGV/s position from the hashtable
  Offset new_offset, old_ugv_position, new_ugv_position;

  std::string old_ugv_encoder = ( *(ts_map->find(old_time_stamp))).second;
  old_ugv_position.SetOffset( new_offset.FromEncodingToOffset(old_ugv_encoder) );

  // NOTE: hashtable filled top-down, at the top we always find the most recent value
  std::string new_ugv_encoder = (ts_map->begin())->second;
  // FIXME: if we want to use find(time_stamp) we should forsward our KF estimation in the future, since the actual one is still not present
  //std::cout << (ts_map->find(current_time_stamp))->first << std::endl;
  new_ugv_position.SetOffset(new_offset.FromEncodingToOffset(new_ugv_encoder));


  //new_offset.SetRoll( new_ugv_position.GetRoll() - old_ugv_position.GetRoll() );
  //new_offset.SetPitch( new_ugv_position.GetPitch() - old_ugv_position.GetPitch() );

  // Rotate 90 degrees when passing from EKF to ar_pose frame

  float pitch = new_ugv_position.GetPitch() - old_ugv_position.GetPitch();
  if (std::abs(pitch) < 0.005) pitch = 0;
  new_offset.SetRoll( pitch );
  //std::cout << "Roll: " << new_ugv_position.GetPitch() << " : " << old_ugv_position.GetPitch() << " : " << pitch << std::endl;



  float roll = ( new_ugv_position.GetRoll() - old_ugv_position.GetRoll() );
  if (std::abs(roll) < 0.005) roll = 0;
  new_offset.SetPitch( roll );
  //std::cout << "Pitch: " << new_ugv_position.GetRoll() << " : " << old_ugv_position.GetRoll() << " : " << roll << std::endl;

  new_offset.SetGaz( new_ugv_position.GetGaz() - old_ugv_position.GetGaz() );
  new_offset.SetYaw( (new_ugv_position.GetYaw() - old_ugv_position.GetYaw()) * 180 / PI );

  //std::cout << "Yaw: " << new_ugv_position.GetYaw() << " : " << old_ugv_position.GetYaw() << " : " << new_offset.GetYaw() << std::endl;

  std::cout <<"UGV position (x,y,z,yaw): " <<  new_offset.GetRoll() << " " << new_offset.GetPitch() << " " << new_offset.GetGaz() << " " << new_offset.GetYaw() << std::endl;

  return new_offset;

}
