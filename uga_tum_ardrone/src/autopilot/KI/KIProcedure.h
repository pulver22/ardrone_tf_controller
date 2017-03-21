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
#ifndef __KIPROCEDURE_H
#define __KIPROCEDURE_H
 
 

#include "../DroneController.h"
#include "uga_tum_ardrone/filter_state.h"

class ControlNode;
class DroneController;


class KIProcedure
{
protected:
	ControlNode* node;
	DroneController* controller;

public:
	std::string command;

	// called externally before first call to update().
	inline void setPointers(ControlNode* node, DroneController* cont)
	{
		this->node = node;
		controller = cont;
	}

	// is called with control-frequency, is supposed to each time generate and send a new
	// control command to the drone.
	// returns wether the goal of this KI has been reached (leads to the KI being destroyed and the next one being popped).
	virtual bool update(const uga_tum_ardrone::filter_stateConstPtr statePtr) = 0;

	// constructed shortly before first update.
	inline KIProcedure(void) {node = NULL; controller = NULL; command = "not set"; };
	virtual ~KIProcedure(void) {};
};

#endif /* __KIPROCEDURE_H */

