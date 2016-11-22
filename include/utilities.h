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
    Offset UpdateUGVPosition(int current_time_stamp, int old_time_stamp,std::unordered_map<int, std::string> *ts_map);

};
#endif // UTILITIES_H
