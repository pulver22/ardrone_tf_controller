#include "../include/offset.h"
#include <cmath>  // abs
#include <iostream>

Offset::Offset()
{
}

Offset::~Offset()
{
}

Offset::Offset(double roll, double pitch, double gaz, double yaw)
{
  this->roll_ = roll;
  this->pitch_ = pitch;
  this->gaz_ = gaz;
  this->yaw_ = yaw;
}

double Offset::GetRoll()
{
  return this->roll_;
}

double Offset::GetPitch()
{
  return this->pitch_;
}

double Offset::GetGaz()
{
  return this->gaz_;
}

double Offset::GetYaw()
{
  return this->yaw_;
}

void Offset::SetRoll(double roll)
{
  this->roll_ = roll;
}

void Offset::SetPitch(double pitch)
{
  this->pitch_ = pitch;
}

void Offset::SetGaz(double gaz)
{
  this->gaz_ = gaz;
}

void Offset::SetYaw(double yaw)
{
  this->yaw_ = yaw;
}

void Offset::SetOffset(Offset offset)
{
  this->roll_ = offset.GetRoll();
  this->pitch_ = offset.GetPitch();
  this->gaz_ = offset.GetGaz();
  this->yaw_ = offset.GetYaw();
}

void Offset::ReduceOffsetToZero(Offset &offset, double target_X, double target_Y, double target_Z, double target_yaw)
{
  if (std::abs(offset.GetRoll()) < target_X)
    {
      offset.SetRoll(0);
    }

  if (std::abs(offset.GetPitch()) < target_Y)
    {
      offset.SetPitch(0);
    }

  if (std::abs(offset.GetGaz()) < target_Z)
    {
      offset.SetGaz(0);
    }

  if (std::abs(offset.GetYaw()) < target_yaw)
    {
      offset.SetYaw(0);
    }
}

void Offset::FromTfToOffset(Offset *offset, geometry_msgs::TransformStamped tf, tfScalar yaw, float epsilon, bool front_camera, int multiplier, bool was_reverse, bool initialization_after_tf_lost, bool critical_phase, int branch)
{
  switch(branch)
    {
    case(1):

      if ( ( was_reverse == true && initialization_after_tf_lost == false) || critical_phase == true )
        {
          ROS_WARN ( "Reversed! --> Changing multiplier" );
          //multiplier = -1;
        }
      (*offset).SetRoll ( -multiplier * tf.transform.translation.x );
      (*offset).SetPitch ( -multiplier * tf.transform.translation.y );
      (*offset).SetGaz ( -abs ( tf.transform.translation.z ) );
      (*offset).SetYaw ( ( float ) yaw * 180 / PI );
      break;

    case(2):
      //multiplier = 1;

      (*offset).SetRoll ( multiplier * tf.transform.translation.x );
      (*offset).SetPitch ( multiplier * tf.transform.translation.y );

      if ( front_camera == true && abs ( yaw ) < abs ( PI/2 + epsilon ) )
        {
          (*offset).SetRoll ( - multiplier * tf.transform.translation.y );
          (*offset).SetPitch ( multiplier * tf.transform.translation.x );
        }
      (*offset).SetGaz ( -abs ( tf.transform.translation.z ) );
      (*offset).SetYaw ( ( ( float ) yaw * 180 / PI ) );

    }


}

// Increase the offsets to center the UAV and prevent it's not able to track the moving marker
void Offset::CentreFOV(Offset *offset, float target_x, float target_y, double camera_alignment_x)
{
  if ( abs(offset->GetRoll()) > target_x || abs(offset->GetPitch()) > target_y)
    {
      //cout << camera_alignment_x << endl;
      if ( camera_alignment_x > 0.4 )
        {
          offset->SetYaw ( 10 );
        }
      else if ( camera_alignment_x < -0.4 )
        {
          offset->SetYaw ( -10 );
        }
      else offset->SetYaw ( 0 );
    }
}

// If the distance on X and Y is higher than a threshold, when using front camera just translate keeping the marker at the centre of the camera's frame!
void Offset::CentreUAV(Offset *offset, float target_x, float target_y)
{
  if(offset->GetRoll() > target_x)
    {
      offset->SetRoll(offset->GetRoll() + 0.5);
    } else{
      offset->SetRoll(offset->GetRoll() - 0.5);
    }

  if(offset->GetPitch() > target_y)
    {
      offset->SetPitch(offset->GetPitch() + 0.5);
    } else{
      offset->SetPitch(offset->GetPitch() - 0.5);
    }
}

void Offset::RotateOnly(Offset *offset, float target_x, float target_y, bool *tf_lost_compensatory)
{
  if( offset->GetYaw() < -5 || offset->GetYaw() > 5)
    {
      //cout <<"Yaw: "<< current_offset.GetYaw() << endl;
      ROS_WARN( " TF found with bottomcamera! First rotate and then approach!!" );
      if ( offset->GetRoll() > target_x )
        {
          //cout << "Branch 1" << endl;
          offset->SetRoll( 0 );
        }

      if ( offset->GetPitch() > target_y)
        {
          //cout << "Branch 2" << endl;
          offset->SetPitch( 0 );
        }
    }
  else
    {
      //initialization_after_tf_lost = false;
      *tf_lost_compensatory = true;
      ROS_WARN("TF compensatory set!!");
    }
}


std::string Offset::FromOffsetToEncoding(geometry_msgs::Point position, tfScalar yaw)
{
    std::string encoding;

    encoding = std::to_string(position.x) + "/" + std::to_string(position.y) + "/" + std::to_string(position.z) + "/"  + std::to_string(yaw) ;

    return encoding;
}

Offset Offset::FromEncodingToOffset(std::string encoding)
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
