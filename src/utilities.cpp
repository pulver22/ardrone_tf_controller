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

std::string Utilities::getEncoding(geometry_msgs::Point position, tfScalar yaw)
{
    std::string encoding;

    encoding = std::to_string(position.x) + "/" + std::to_string(position.y) + "/" + std::to_string(position.z) + "/"  + std::to_string(yaw) ;

    return encoding;
}
