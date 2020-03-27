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


#include "modules/computer_vision/cv.h"
#include "modules/observer/observer.h"
// #include "generated/flight_plan.h"
#include "state.h"
#include <stdio.h>
#include <time.h>

#ifndef OBSERVER_NODE_H
#define OBSERVER_NODE_H


////////////////////////////////////////////////////////////////////////////////
// DEFINE FUNCTIONS

// Main function
struct image_t *observer_func(struct image_t *img);

// Image processing functions
void read_drone_state();
void create_img(struct image_t *input, struct image_t *output);
void copy2img(struct image_t *input, struct image_t *output);
void image_specfilt(struct image_t *input, struct image_t *output1, 
                      struct image_t *output2, uint8_t y_m, 
                      uint8_t y_M, uint8_t u_m, uint8_t u_M, 
                      uint8_t v_m, uint8_t v_M, uint8_t *mask);
void image_bgfilt(struct image_t *input, struct image_t *output, uint8_t y_m, 
                  uint8_t y_M, uint8_t u_m, uint8_t u_M, uint8_t v_m, 
                  uint8_t v_M, bool color, bool remove_ground);
void convolve(struct image_t *input, struct image_t *output);
void convolve_big(struct image_t *input, struct image_t *output);
void blur_image(struct image_t *input, struct image_t *output);
void blur_big(struct image_t *input, struct image_t *output);
void blur_mask(struct image_t *input, uint8_t *mask);
void detect_poles(uint16_t *poles);
void find_orange_objs();
void find_green_objs();  
void find_rand_objs(struct image_t *input);
void logic_curtain(uint16_t idx1, uint16_t idx2, struct image_t *input);
void logic_free(uint16_t idx1, uint16_t idx2);
void combine_measurements();
void delete_outliers();
void find_distances();

extern float final_objs[100][4];

// Init function
extern void observer_node_init(void);


////////////////////////////////////////////////////////////////////////////////
// DEFINE PARAMETERS

// Output image
// struct image_t *outimg;

// // Color filters
// const int n_cf = 2;
// const int n_cols = 6;
//                     // y_m  y_M  u_m  u_M  v_m  v_M 
// const extern uint8_t cf[n_cf][n_cols] = {{0,   255, 0,   255, 0,   255},
//                                   {100, 255, 100, 255, 100, 255}};



#endif