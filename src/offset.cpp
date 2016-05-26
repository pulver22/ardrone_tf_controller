#include "../include/offset.h"
#include <stdlib.h>     /* abs */



Offset::Offset()
{
}

Offset::~Offset()
{
}

Offset::Offset (double roll, double pitch, double gaz, double yaw ) 
{
	this->roll_ = roll;
	this->pitch_ = pitch;
	this->gaz_ = gaz;
	this->yaw_ = yaw;
}

double Offset::GetRoll(){
	return this->roll_;
}

double Offset::GetPitch(){
	return this->pitch_;
}

double Offset::GetGaz(){
	return this->gaz_;
}

double Offset::GetYaw(){
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
        if ( abs ( this->GetRoll() ) < target_X ) {
                this->SetRoll ( 0 );
        }

        if ( abs ( this->GetPitch() ) < target_Y ) {
                this->SetPitch ( 0 );
        }

        if ( abs ( this->GetGaz() ) < target_Z ) {
                this->SetGaz ( 0 );
        }

        if ( abs ( this->GetYaw() ) < target_yaw ) {
                this->SetYaw ( 0 );
        }

}
