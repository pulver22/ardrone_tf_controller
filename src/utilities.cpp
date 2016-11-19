#include <string>

#include "../include/utilities.h"
#include <geometry_msgs/Point.h>
#include <tf/LinearMath/Matrix3x3.h>

Utilities::Utilities()
{
}

Utilities::~Utilities()
{
}

std::string Utilities::FromOffsetToEncoding(geometry_msgs::Point position, tfScalar yaw)
{
    std::string encoding;

    encoding = std::to_string(position.x) + "/" + std::to_string(position.y) + "/" + std::to_string(position.z) + "/"  + std::to_string(yaw) ;

    return encoding;
}

Offset Utilities::FromEncodingToOffset(std::string encoding)
{

  //std::cout << encoding << std::endl;

  std::string s;
  Offset offset;
  char delimiter('/');
  std::string::size_type pos = encoding.find(delimiter);

  offset.SetRoll(std::stod(encoding.substr(0, pos)));

  s = encoding.substr(pos + 1, encoding.size());
  pos = s.find(delimiter);
  offset.SetPitch(std::stod(s.substr(0, pos)));

  s = s.substr(pos + 1, encoding.size());
  pos = s.find(delimiter);
  offset.SetGaz(std::stod(s.substr(0, pos)));

  s = s.substr(pos + 1, encoding.size());
  pos = s.find(delimiter);
  offset.SetYaw(std::stod(s.substr(0, pos)));

  //std::cout << offset.GetRoll() << " " << offset.GetPitch() << " " << offset.GetGaz() << " " << offset.GetYaw() << std::endl;

}
