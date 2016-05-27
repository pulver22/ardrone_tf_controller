#include "../include/offset.h"
#include <stdlib.h>     /* abs */
#include <iostream>



Offset::Offset()
{
}

Offset::~Offset()
{
}

Offset::Offset ( double roll, double pitch, double gaz, double yaw )
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



void Offset::SetRoll ( double roll )
{
  this->roll_ = roll;
}


void Offset::SetPitch ( double pitch )
{
  this->pitch_ = pitch;
}

void Offset::SetGaz ( double gaz )
{
  this->gaz_ = gaz;
}


void Offset::SetYaw ( double yaw )
{
  this->yaw_ = yaw;
}


void Offset::SetOffset ( Offset offset )
{
  this->roll_ = offset.GetRoll();
  this->pitch_ = offset.GetPitch();
  this->gaz_ = offset.GetGaz();
  this->yaw_ = offset.GetYaw();
}

void Offset::ReduceOffsetToZero ( Offset& offset, double target_X, double target_Y, double target_Z, double target_yaw )
{
	std::cout << offset.GetRoll() << " " << target_X << std::endl;
  if ( abs ( offset.GetRoll() ) < target_X )
    {
				std::cout << offset.GetRoll() << " " << target_X << std::endl;
			std::cout << "entered" << std::endl;
      offset.SetRoll ( 0 );
    }

  if ( abs ( offset.GetPitch() ) < target_Y )
    {
      offset.SetPitch ( 0 );
    }

  if ( abs ( offset.GetGaz() ) < target_Z )
    {
      offset.SetGaz ( 0 );
    }

  if ( abs ( offset.GetYaw() ) < target_yaw )
    {
      offset.SetYaw ( 0 );
    }

}
