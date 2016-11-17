#ifndef UTILITIES_H
#define UTILITIES_H

#include <string>
#include <std_msgs/String.h>
#include <geometry_msgs/Point.h>
#include <tf/LinearMath/Matrix3x3.h>

class Utilities
{
public:
    Utilities();
    ~Utilities();
    std::string getEncoding (geometry_msgs::Point position, tfScalar yaw);

};
#endif // UTILITIES_H
