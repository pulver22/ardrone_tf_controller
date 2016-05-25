#ifndef OFFSET_H
#define OFFSET_H


class Offset{
public:
	
	Offset();
	Offset(double roll, double pitch, double gax, double yaw);
	Offset(const Offset& other);
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