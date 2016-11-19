#ifndef UTILITIES_H
#define UTILITIES_H

#include <string>
#include <std_msgs/String.h>
#include <geometry_msgs/Point.h>
#include <tf/LinearMath/Matrix3x3.h>

#include "../include/offset.h"

class Utilities
{
public:
    Utilities();
    ~Utilities();
    std::string FromOffsetToEncoding (geometry_msgs::Point position, tfScalar yaw);
    Offset FromEncodingToOffset(std::string encoding);


};
#endif // UTILITIES_H
