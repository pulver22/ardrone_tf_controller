#ifndef OFFSET_H
#define OFFSET_H


class Offset{
public:
	
	Offset();
	double getRoll();
	double getPitch();
	double getGaz();
	double getYaw();
	void setRoll(double roll);
	void setPitch(double pitch);
	void setGaz(double gaz);
	void setYaw(double yaw);
	void setOffset(Offset offset);
	
protected:
	
	double roll;
	double pitch;
	double gaz;
	double yaw;
	

};

#endif