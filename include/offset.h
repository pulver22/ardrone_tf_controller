#ifndef OFFSET_H
#define OFFSET_H


class Offset{
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
	void ReduceOffsetToZero ( Offset &offset, double target_X, double target_Y, double target_Z, double target_yaw );
	
protected:
	
	double roll_;
	double pitch_;
	double gaz_;
	double yaw_;
	

};

#endif