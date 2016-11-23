#ifndef OFFSET_H
#define OFFSET_H

#include <tf2_ros/transform_listener.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <geometry_msgs/Point.h>

const double PI = 3.141593;


class Offset
{
public:
  Offset();
  Offset(double roll, double pitch, double gax, double yaw);
  ~Offset();
  double GetRoll();
  double GetPitch();
  double GetGaz();
  double GetYaw();
  void SetRoll(double roll);
  void SetPitch(double pitch);
  void SetGaz(double gaz);
  void SetYaw(double yaw);
  void SetOffset(Offset offset);
  void ReduceOffsetToZero(Offset &offset, double target_X, double target_Y, double target_Z, double target_yaw);
  void FromTfToOffset(Offset *offset, geometry_msgs::TransformStamped tf, tfScalar yaw, float epsilon, bool front_camera, int multiplier, bool was_reverse, bool initialization_after_tf_lost, bool critical_phase, int branch);
  void CentreUAV(Offset *offset, float target_x, float target_y);
  void CentreFOV(Offset *offset, float target_x, float target_y, double camera_alignment_x);
  void RotateOnly(Offset *offset, float target_x, float target_y, bool *tf_lost_compensatory);
  std::string FromOffsetToEncoding (geometry_msgs::Point position, tfScalar yaw);
  Offset FromEncodingToOffset(std::string encoding);
  Offset operator=( const Offset offset_2);
  Offset operator+=( const Offset offset_2);

protected:
  double roll_;
  double pitch_;
  double gaz_;
  double yaw_;
};

#endif
