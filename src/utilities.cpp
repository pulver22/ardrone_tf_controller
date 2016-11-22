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

Offset Utilities::UpdateUGVPosition(int current_time_stamp, int old_time_stamp, std::unordered_map<int, std::string> *ts_map)
{
  // Get actual and old UGV/s position from the hashtable
  Offset new_offset, new_ugv_position, old_ugv_position;

  std::unordered_map<int, std::string>::const_iterator iterator = ts_map->find(current_time_stamp);
    ROS_WARN("ALIVE");
    if ( iterator == ts_map->end() )
        std::cout << "not found";
      else
        std::cout << (*iterator).first << " is " << (*iterator).second;

  std::string new_ugv_encoder;
  new_ugv_encoder.assign( iterator->second );
  std::cout << new_ugv_encoder << std::endl;
  new_ugv_position.FromEncodingToOffset(new_ugv_encoder);

  std::string old_ugv_encoder = ( *(ts_map->find(old_time_stamp))).second;
  std::cout << old_ugv_encoder << std::endl;
  old_ugv_position.FromEncodingToOffset(old_ugv_encoder);

  new_offset.SetRoll( new_ugv_position.GetRoll() - old_ugv_position.GetRoll() );
  new_offset.SetPitch( new_ugv_position.GetPitch() - old_ugv_position.GetPitch() );
  new_offset.SetGaz( new_ugv_position.GetGaz() - old_ugv_position.GetGaz() );
  new_offset.SetYaw( new_ugv_position.GetYaw() - old_ugv_position.GetYaw() );

  return new_offset;

}
