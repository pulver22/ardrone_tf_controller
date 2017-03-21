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



#include "DroneController.h"
#include "gvars3/instances.h"
#include "../HelperFunctions.h"
#include "ControlNode.h"

int sgn(double val) {
    return (0 < val) - (val < 0);
}

DroneController::DroneController(void)
{
	target = DronePosition(TooN::makeVector(0.0,0.0,0.0),0.0);
	targetValid = false;
	last_err[2] = 0;
	lastTimeStamp = 0;

	hoverCommand.gaz = hoverCommand.pitch = hoverCommand.roll = hoverCommand.yaw = 0;

	node = NULL;
}


DroneController::~DroneController(void)
{
}

double angleFromTo2(double angle, double min, double sup)
{
	while(angle < min) angle += 360;
	while(angle >=  sup) angle -= 360;
	return angle;
}

// generates and sends a new control command to the drone,
// based on the currently active command ant the drone's position.
ControlCommand DroneController::update(uga_tum_ardrone::filter_stateConstPtr state)
{
	TooN::Vector<3> pose = TooN::makeVector(state->x, state->y, state->z);
	double yaw = state->yaw;
	TooN::Vector<4> speeds = TooN::makeVector(state->dx, state->dy, state->dz, state->dyaw);
	ptamIsGood = state->ptamState == state->PTAM_BEST || state->ptamState == state->PTAM_GOOD || state->ptamState == state->PTAM_TOOKKF;
	scaleAccuracy = state->scaleAccuracy;

	// calculate (new) errors.
	TooN::Vector<4> new_err = TooN::makeVector(
		target.pos[0] - pose[0],
		target.pos[1] - pose[1],
		target.pos[2] - pose[2],
		target.yaw - yaw
		);

	// yaw error needs special attention, it can always be pushed in between 180 and -180.
	// this does not affect speeds and makes the drone always take the quickest rotation side.
	new_err[3] = angleFromTo2(new_err[3],-180,180);
	TooN::Vector<4> d_err = TooN::makeVector(-speeds[0], -speeds[1], -speeds[2], -speeds[3]);

	if(targetValid)
		calcControl(new_err, d_err, yaw, state->pitch, state->roll);
	else
	{
		lastSentControl = hoverCommand;
	}

    if(node->logfileControl != NULL)
	{
		pthread_mutex_lock(&node->logControl_CS);
		if(node->logfileControl != NULL)
        {
			(*(node->logfileControl)) << lastTimeStamp << " ";

			for( int i = 0; i < logInfo.size(); i++)
                (*(node->logfileControl)) << logInfo[i] << " ";

            (*(node->logfileControl)) << "\n";
        }
		pthread_mutex_unlock(&node->logControl_CS);
	}
	last_err = new_err;
	return lastSentControl;
}


void DroneController::setTarget(DronePosition newTarget)
{
	target = newTarget;
	target.yaw = angleFromTo2(target.yaw,-180,180);
	targetSetAtClock = getMS()/1000.0;
	targetNew = TooN::makeVector(1.0,1.0,1.0,1.0);
	targetValid = true;
	last_err = i_term = TooN::makeVector(0,0,0,0);
    lastTimeStamp = getMS()/1000.0 - 0.03; // prevent deltaT from being 0

	char buf[200];
	snprintf(buf,200,"New Target: xyz = %.3f, %.3f, %.3f,  yaw=%.3f", target.pos[0],target.pos[1],target.pos[2],target.yaw);
	ROS_INFO("%s", buf);

	if(node != NULL)
		node->publishCommand(std::string("u l ") + buf);
}


DronePosition DroneController::getCurrentTarget()
{
	return target;
}

void DroneController::clearTarget()
{
	targetValid = false;
}

void i_term_increase(double& i_term, double new_err, double cap)
{
	if(new_err < 0 && i_term > 0)
		i_term = std::max(0.0, i_term + 2.5 * new_err);
	else if(new_err > 0 && i_term < 0)
		i_term = std::min(0.0, i_term + 2.5 * new_err);
	else
		i_term += new_err;

	if(i_term > cap) i_term =  cap;
	if(i_term < -cap) i_term =  -cap;
}

void DroneController::calcControl(TooN::Vector<4> new_err, TooN::Vector<4> new_velocity, double yaw, double pitch, double roll)
{

	float agr = agressiveness;
	if(!ptamIsGood) agr *= 0.75;
	agr *= scaleAccuracy;

    double K_rp_agr = K_rp * agr;
    double K_direct_agr = K_direct * agr;


	TooN::Vector<4> vel_term = new_velocity;
	TooN::Vector<4> p_term = new_err;	// p-term is error.

	// rotate error to drone CS, invert pitch
	double yawRad = yaw * 2 * 3.141592 / 360;
	double pitchRad = pitch * 2 * 3.141592 / 360;
	double rollRad = roll * 2 * 3.141592 / 360;
	vel_term[0] = cos(yawRad)*new_velocity[0] - sin(yawRad)*new_velocity[1];
	vel_term[1] = - sin(yawRad)*new_velocity[0] - cos(yawRad)*new_velocity[1];

	p_term[0] = cos(yawRad)*new_err[0] - sin(yawRad)*new_err[1];
	p_term[1] = - sin(yawRad)*new_err[0] - cos(yawRad)*new_err[1];


    // calculate the damping coefficient assuming critically damped
    double c_direct = 2 * sqrt( K_direct * 50 * droneMassInKilos );
    double c_direct_agr = 2 * sqrt( K_direct_agr * droneMassInKilos );
    double c_rp = xy_damping_factor * 2 * sqrt( K_rp_agr * droneMassInKilos );


    // find target forces given the current state
    double springForceRoll = K_rp_agr * p_term[0];
    double springForcePitch = K_rp_agr * p_term[1];
    double springForceYaw = K_direct * 50 * new_err[3];
    double springForceGaz = K_direct_agr * new_err[2];

    double dampingForceRoll = c_rp * -vel_term[0];
    double dampingForcePitch = c_rp * -vel_term[1];
    double dampingForceYaw = c_direct * -new_velocity[3];
    double dampingForceGaz = c_direct_agr * -new_velocity[2];

    double totalForceRoll = springForceRoll - dampingForceRoll;
    double totalForcePitch = springForcePitch - dampingForcePitch;
    double totalForceYaw = springForceYaw - dampingForceYaw;
    double totalForceGaz = springForceGaz - dampingForceGaz;

    totalForceGaz = std::max((-9.8 * droneMassInKilos) *0.8, totalForceGaz); // the drone can't actually accelerate down, gravity does this.
                            // and we still need the engines available to provide pitch/roll control, don't shut them down completely

    // Integrate for yaw and gaz control
	double deltaT = getMS()/1000.0 - lastTimeStamp; lastTimeStamp = getMS()/1000.0;

	if (deltaT > 0.2) // guard against crazy timestamps
        deltaT = 0.2;

    double deltaYaw = deltaT * (totalForceYaw / droneMassInKilos) / 2;
    double deltaGaz = deltaT * (totalForceGaz / droneMassInKilos) / 2;

    // the new gaz command
	lastSentControl.gaz = -new_velocity[2] + deltaGaz;	// gaz can be translated directly
	lastSentControl.gaz = std::min(max_gaz_rise,std::max(max_gaz_drop, (double)(lastSentControl.gaz)));

    double clippedZForce = std::max((-9.8 * droneMassInKilos)*0.8, (2 * droneMassInKilos * (lastSentControl.gaz + new_velocity[2])) / deltaT);

    // how much force do the motors actually generate? need to remove gravity from this
    double appliedEngineForce = (/*clippedZForce +*/ 9.8 * droneMassInKilos) / (fabs(cos(pitchRad) * cos(rollRad)));

    if (fabs(totalForceRoll) > fabs(appliedEngineForce))
        totalForceRoll = fabs(appliedEngineForce)*sgn(totalForceRoll);

    if (fabs(totalForcePitch) > fabs(appliedEngineForce))
        totalForcePitch = fabs(appliedEngineForce)*sgn(totalForcePitch);

    // The new rotation commands
	lastSentControl.roll = (1.57 - acos(totalForceRoll / appliedEngineForce)) / max_rp_radians;
	lastSentControl.pitch = (1.57 - acos(totalForcePitch / appliedEngineForce)) / max_rp_radians;
	lastSentControl.yaw = ((-new_velocity[3] + deltaYaw) * (2 * 3.141592 / 360)) / 1.66;	// yaw can be translated directly, command is 1.0 = 1.66 rads/second

	// clip
	lastSentControl.roll = std::min(max_rp,std::max(-max_rp,(double)(lastSentControl.roll)));
	lastSentControl.pitch = std::min(max_rp,std::max(-max_rp,(double)(lastSentControl.pitch)));
	lastSentControl.yaw = std::min(max_yaw,std::max(-max_yaw,(double)(lastSentControl.yaw)));


	logInfo = TooN::makeVector(
		springForceRoll, springForcePitch, springForceGaz, springForceYaw,
        dampingForceRoll, dampingForcePitch, dampingForceGaz, dampingForceYaw,
		deltaT, deltaYaw, deltaGaz, new_velocity[0], new_velocity[1], new_velocity[2],
		new_velocity[3], new_err[2], new_err[3], clippedZForce,
		lastSentControl.roll, lastSentControl.pitch, lastSentControl.gaz, lastSentControl.yaw,
		appliedEngineForce,yawRad,pitchRad,rollRad,
		target.pos[0],target.pos[1],target.pos[2],target.yaw
		);
}

TooN::Vector<4> DroneController::getLastErr()
{
	return last_err;
}
ControlCommand DroneController::getLastControl()
{
	return lastSentControl;
}
