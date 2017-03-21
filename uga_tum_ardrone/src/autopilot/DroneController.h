#pragma once
 /**
 *  This file is part of uga_tum_ardrone.
 *
 *  Copyright 2012 Jakob Engel <jajuengel@gmail.com> (Technical University of Munich)
 *  Portions Copyright 2015 Kenneth Bogert <kbogert@uga.edu> and Sina Solaimanpour <sina@uga.edu> (THINC Lab, University of Georgia)
 *  For more information see <https://vision.in.tum.de/data/software/uga_tum_ardrone>.
 *
 *  uga_tum_ardrone is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  uga_tum_ardrone is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with uga_tum_ardrone.  If not, see <http://www.gnu.org/licenses/>.
 */
#ifndef __DRONECONTROLLER_H
#define __DRONECONTROLLER_H




#include "TooN/se3.h"
#include <queue>
#include "geometry_msgs/Twist.h"
#include "uga_tum_ardrone/filter_state.h"

class ControlNode;

struct ControlCommand
{
	inline ControlCommand() {roll = pitch = yaw = gaz = 0;}
	inline ControlCommand(double roll, double pitch, double yaw, double gaz)
	{
		this->roll = roll;
		this->pitch = pitch;
		this->yaw = yaw;
		this->gaz = gaz;
	}
	double yaw, roll, pitch, gaz;
};


struct DronePosition
{
public:
	double yaw;
	TooN::Vector<3> pos;
	inline DronePosition(TooN::Vector<3> pos, double yaw)
		: yaw(yaw), pos(pos) {}
	inline DronePosition(){ yaw=0; pos=TooN::makeVector(0,0,0);}
};

class DroneController
{
private:
	ControlCommand lastSentControl;

	// currentTarget.
	DronePosition target;
	bool targetValid;

	// used for integral term
	TooN::Vector<4> targetNew;	// 0=target has been reached before
								// 1=target is new

	// need to keep track of integral terms
	TooN::Vector<4> i_term;
	TooN::Vector<4> last_err;
	TooN::Vector<4> speedAverages;

	double lastTimeStamp;
	double targetSetAtClock;
	ControlCommand hoverCommand;



	// filled with info (on update)
	bool  ptamIsGood;
	double scaleAccuracy;
	void calcControl(TooN::Vector<4> new_err, TooN::Vector<4> d_error, double yaw, double pitch, double roll);

public:

	// generates and sends a new control command to the drone, based on the currently active command ant the drone's position.
	ControlCommand update(uga_tum_ardrone::filter_stateConstPtr);

	ControlNode* node;

	// for logging, gets filled with recent infos on control.
	TooN::Vector<30> logInfo;

	// adds a waypoint
	void setTarget(DronePosition newTarget);
	void clearTarget();
	DronePosition getCurrentTarget();
	ControlCommand getLastControl();

	// gets last error
	TooN::Vector<4> getLastErr();

	DroneController(void);
	~DroneController(void);




    // damped spring control parameters, settable by dynamic reconfigure

    double K_direct;        // spring strength for yaw and linear Z
    double K_rp;            // spring strength for roll and pitch

    double droneMassInKilos;
    double max_rp_radians;  // maximum roll and pitch of the drone in radians (not the ardrone_autonomy parameter)

	double rise_fac;
	double agressiveness;
	double max_gaz_rise;
	double max_gaz_drop;
	double max_yaw;
	double max_rp;
	double xy_damping_factor;
};
#endif /* __DRONECONTROLLER_H */

