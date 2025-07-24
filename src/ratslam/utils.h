/*
 * openRatSLAM
 *
 * utils - General purpose utility helper functions mainly for angles and readings settings
 *
 * Copyright (C) 2012
 * David Ball (david.ball@qut.edu.au) (1), Scott Heath (scott.heath@uqconnect.edu.au) (2)
 *
 * RatSLAM algorithm by:
 * Michael Milford (1) and Gordon Wyeth (1) ([michael.milford, gordon.wyeth]@qut.edu.au)
 *
 * 1. Queensland University of Technology, Australia
 * 2. The University of Queensland, Australia
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef _RATSLAM_UTILS_H
#define _RATSLAM_UTILS_H

#define _USE_MATH_DEFINES
#include <math.h>

#include <iostream>
#include <float.h>


namespace ratslam {

// % Clip the input angle to between 0 and 2pi radians
inline double clip_rad_360(double angle)
{
    while (angle < 0)
        angle += 2.0 * M_PI;

    while (angle >= 2.0 * M_PI)
        angle -= 2.0 * M_PI;
 
    return angle;
}

// % Clip the input angle to between -pi and pi radians
inline double clip_rad_180(double angle)
{
    while (angle > M_PI)
        angle -= 2.0 * M_PI;

    while (angle <= -M_PI)
        angle += 2.0 * M_PI;
    
    return angle;
}

//% Get the signed delta angle from angle1 to angle2 handling the wrap from 2pi
//% to 0.
inline double get_signed_delta_rad(double angle1, double angle2)
{
    double dir = clip_rad_180(angle2 - angle1);

    double delta_angle = clip_rad_360(angle1) - clip_rad_360(angle2);
	delta_angle = fabs(delta_angle);

    if (delta_angle < 2.0 * M_PI - delta_angle)
    {
        if (dir > 0)
            return delta_angle;
        else
            return -delta_angle;
    }
    else
    {
        if (dir > 0)
            return 2.0 * M_PI - delta_angle;
        else
            return -(2.0 * M_PI - delta_angle);
    }
}

}; // namspace ratslam

#endif // _RATSLAM_UTILS_H