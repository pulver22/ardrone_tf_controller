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
#ifndef __MOUSEKEYHANDLER_H
#define __MOUSEKEYHANDLER_H
 
 
#pragma once
#include "cvd/image.h"

class MouseKeyHandler
{
public:
	// default constructors
	inline MouseKeyHandler() {};
	inline ~MouseKeyHandler(void) {};
	virtual inline void on_key_down(int key) {};
	virtual inline void on_mouse_move(CVD::ImageRef where, int state) {};
	virtual inline void on_mouse_down(CVD::ImageRef where, int state, int button) {};
	virtual inline void on_event(int event) {};
};
#endif /* __MOUSEKEYHANDLER_H */
