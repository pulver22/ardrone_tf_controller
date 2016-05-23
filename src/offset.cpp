#include "../include/offset.h"


Offset::Offset()
{
}

double Offset::getRoll(){
	return this->roll;
}

double Offset::getPitch(){
	return this->pitch;
}

double Offset::getGaz(){
	return this->gaz;
}

double Offset::getYaw(){
	return this->yaw;
}



void Offset::setRoll ( double roll )
{
	this->roll = roll;
}


void Offset::setPitch ( double pitch )
{
	this->pitch = pitch;
}

void Offset::setGaz ( double gaz )
{
	this->gaz = gaz;
}


void Offset::setYaw ( double yaw )
{
	this->yaw = yaw;
}


void Offset::setOffset ( Offset offset )
{
	this->roll = offset.getRoll();
	this->pitch = offset.getPitch();
	this->gaz = offset.getGaz();
	this->yaw = offset.getYaw();
}
