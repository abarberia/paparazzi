/*
 * Copyright (C) Team Wonder
 *
 * This file is part of paparazzi
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */
/**
 * @file "modules/TRAJECTORY/TRAJECTORY.h"
 * @author Team Wonder
 * Just pathing in TRAJECTORY
 */

#ifndef TRAJECTORY_H
#define TRAJECTORY_H


// settings
extern int TRAJECTORY_L;
extern int TRAJECTORY_D;
extern int AVOID_number_of_objects;
extern float AVOID_h1,AVOID_h2;
extern float AVOID_d;
extern float AVOID_safety_angle;
extern float current_time;
extern float TRAJECTORY_X;
extern float TRAJECTORY_Y;
extern int square_mode;
extern int mode;

// functions
extern void trajectory_init();
extern void trajectory_periodic();

#endif

