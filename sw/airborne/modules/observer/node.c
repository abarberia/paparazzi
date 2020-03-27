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


#include "modules/observer/node.h"


#ifndef OPENCVDEMO_FPS
#define OPENCVDEMO_FPS 0       ///< Default FPS (zero means run at camera fps)
#endif

#ifndef COLORFILTER_CAMERA
#define COLORFILTER_CAMERA front_camera
#endif

// Variables declaration
  // Drone positions
  float x_loc;
  float y_loc;
  float head;
  float vx_loc;
  float vy_loc;
  float v_tot;
  float rot_loc;

  // orange filter       y_m  y_M  u_m  u_M  v_m  v_M
  uint8_t orange_cf[6] = { 26, 164,  45, 130, 160, 192};
  uint8_t green_cf[6]  = { 70, 230,   0, 120,   0, 125};
  uint8_t blue_cf[6]   = { 67, 255, 120, 255,   0, 125};
  uint8_t black_cf[6]  = {116, 140, 116, 140,   0,  120};

  // orange mask
  uint8_t mask_o[520][240];
  // green mask
  uint8_t mask_g[520][240];
  // random mask
  uint8_t mask_r[520][240];

  // limits on green mask object detection:
  // do not check if outside these boundaries
  const float top_x_green = 3.5;
  const float top_y_green = 3.5;

  const float head_x_t_green = 2.2;
  const float head_x_b_green = 1;
  const float head_y_t_green = 0.6;
  const float head_y_b_green = 2.5; // capo is on 2.618
  // const float head_y_pi_green = 3.58;


  // bool to check (if close to the edge, don't look)
  bool check_green;

  bool started_rand;


  // floor of green mask
  uint16_t floors[520];

  // background filter
  uint8_t n_cf = 2;
  uint8_t bg_cf[2][6] = {{67, 255, 120, 255,  0, 125},  // blue
                      {70,   255,   0, 120,   0, 125}}; // green
  // uint8_t bg_cf[1][6] = {{70,   255,   0, 130,   0, 130}}; // green

  // small kernels
    // uint8_t kernel[3][3] = {{0, 1, 0},
    //                         {1, -4, 1},
    //                         {0, 1, 0}};

    // uint8_t kernel[3][3] = {{1, 0, -1},
    //                         {2, 0, -2},
    //                         {1, 0, -1}};
  //


  // 3x3 kernel
  uint8_t kernel[3][3] = {{0, 0, 0},
                          {1, 0, -1},
                          {0, 0, 0}};


  // big kernels
    // uint8_t big_kernel[7][7] = {{1, 1, 0, 0, 0, -1, -1},
    //                             {1, 1, 0, 0, 0, -1, -1},
    //                             {2, 2, 0, 0, 0, -2, -2},
    //                             {5, 5, 0, 0, 0, -5, -5},
    //                             {2, 2, 0, 0, 0, -2, -2},
    //                             {1, 1, 0, 0, 0, -1, -1},
    //                             {1, 1, 0, 0, 0, -1, -1}};

    // uint8_t big_kernel[7][7] = {{1, 3, 2, 5, 2, 3, 1},
    //                             {1, 3, 2, 5, 2, 3, 1},
    //                             {0, 0, 0, 0, 0, 0, 0},
    //                             {0, 0, 0, 0, 0, 0, 0},
    //                             {0, 0, 0, 0, 0, 0, 0},
    //                             {-1, -3, -2, -5, -2, -3, -1},
    //                             {-1, -3, -2, -5, -2, -3, -1}};

    // uint8_t big_kernel[7][7] = {{1, 1, 2, 5, 2, 1, 1},
    //                             {0, 0, 0, 0, 0, 0, 0},
    //                             {0, 0, 0, 0, 0, 0, 0},
    //                             {0, 0, 0, 0, 0, 0, 0},
    //                             {0, 0, 0, 0, 0, 0, 0},
    //                             {0, 0, 0, 0, 0, 0, 0},
    //                             {-1, -1, -2, -5, -2, -1, -1}};


    // const int8_t big_kernel[5][5] = {{1, 2, 5, 2, 1},
    //                             {0, 0, 0, 0, 0},
    //                             {0, 0, 0, 0, 0},
    //                             {0, 0, 0, 0, 0},
    //                             {-1, -2, -5, -2, -1}};
  //  

  const int8_t big_kernel[5][5] = {{1, 1, 0, -1, -1},
                                  {2, 2, 0, -2, -2},
                                  {3, 3, 0, -3, -3},
                                  {2, 2, 0, -2, -2},
                                  {1, 1, 0, -1, -1}};


  // uint8_t big_kernel[3][3] = {{0, 0, 0},
  //                             {1, 0, -1},
  //                             {0, 0, 0}};
                            

  // big kernel length
  const uint8_t ker_l = 5;
  const uint8_t ker_l2 = 2;

  const uint8_t sum_k = 36;

  // Blur length
  const uint8_t blur_l = 20;



  // Because image is tilted, so is the kernel -> big vertical blur
    // const uint8_t blur_kernel[3][7] = {{1, 1, 1, 1, 1, 1, 1},
    //                                    {1, 1, 1, 1, 1, 1, 1},
    //                                    {1, 1, 1, 1, 1, 1, 1}};

    // const uint8_t blur_kernel[3][21] = {{1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
    //                                     {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
    //                                     {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1}};


    // Blur dimensions

    // const uint8_t blur_w = 3;
    // const uint8_t blur_w2 = 1;

    // const uint8_t blur_h = 21;
    // const uint8_t blur_h2 = 10;
  //

  const uint8_t blur_w = 21;
  const uint8_t blur_w2 = 10;

  const uint8_t blur_h = 1;
  const uint8_t blur_h2 = 0;

  // detected poles (max 100 at a time) -> (left_px, right_px, distance)
    // poles 00-29 are used for orange
    // poles 30-59 are used for green
    // poles 60-99 are used for random edges
  //  

  uint8_t idx_o = 0;
  uint8_t idx_g = 30;
  uint8_t idx_r = 50;

  const float px_dist_scale = 107.232;


  uint16_t poles[100][3];
  uint16_t poles_comb[100][4]; // left_px, right_px, dist, type
                              // types are: 0 = undefined; 1 = orange pole
  int16_t poles_w_inertia[100][5]; // left_px, right_px, dist, times_seen, type
  float final_objs[100][4];
  uint8_t count_o;
  uint8_t count_g;
  uint8_t count_r;
  uint8_t count;
  uint8_t count_inertia;
  uint8_t final_count;
  uint8_t obj_merged[100];
  uint8_t idx_to_rm[100];
  uint8_t elem_to_rm;
  uint8_t rand_search_type;
  uint16_t px_change;

  uint16_t sums_o[520]; // cumulative sum
  uint16_t sums_r[520]; // cumulative sum
  uint16_t sums_r_smooth[520]; // smoothed cumulative sum

  // Distance hreshold for same object [pixels]
  const uint8_t threshold = 30;
  const uint8_t associate_threshold = 70;

  struct image_t processed;
  struct image_t color_mask;
  struct image_t blurred;

  uint32_t glob = 0;

  bool first_time = true;
//

////////////////////////////////////////////////////////////////////////////////
// MAIN FUNCTION
struct image_t *observer_func(struct image_t *img){

  // nav_set_heading_towards_waypoint(WP_STDBY);

  if (img->type == IMAGE_YUV422) {

    // Take the time
    clock_t begin = clock();

    // Copy input image to processed, convoluted and color masks
    if (first_time){
      create_img(img, &processed);
      create_img(img, &color_mask);
      create_img(img, &blurred);
      first_time = false;
    }

    // Clean poles
    for (uint16_t x = 0; x < 100; x++) {
      poles[x][0] = 0;
      poles[x][1] = 0;
      poles[x][2] = 0;
      poles_comb[x][0] = 0;
      poles_comb[x][1] = 0;
      poles_comb[x][2] = 0;
      obj_merged[x] = 0;
      idx_to_rm[x] = 0;
      final_objs[x][0] = 0;
      final_objs[x][1] = 0;
      final_objs[x][2] = 0;
    }

    // Get the drone's posititon and heading for object detector
    read_drone_state();

    // //printf("[%lf, %lf, %lf]\n", stateGetPositionEnu_f()->x, stateGetPositionEnu_f()->y, stateGetNedToBodyEulers_f()->psi);

    // Filter poles (orange) and find objects from mask
    image_specfilt(img, &processed, &color_mask, orange_cf[0], orange_cf[1], orange_cf[2],
                    orange_cf[3], orange_cf[4], orange_cf[5], &mask_o);


    // Filter ground (green) and find objects from mask
    image_specfilt(&processed, &processed, &color_mask, green_cf[0], green_cf[1], 
                green_cf[2], green_cf[3], green_cf[4], green_cf[5], &mask_g);


    // // Filter blue color and remove noise in ground
    image_bgfilt(&processed, &processed, blue_cf[0], blue_cf[1], blue_cf[2],
                 blue_cf[3], blue_cf[4], blue_cf[5], false, true);

    // // // Filter black color and remove noise in ground
    // image_bgfilt(&processed, &processed, black_cf[0], black_cf[1], black_cf[2],
    //              black_cf[3], black_cf[4], black_cf[5], false, false);


    // uint8_t *source = (uint8_t *)img->buf;

    // if (glob%10 == 0){
    //   // Go trough all the pixels
    //   for (uint16_t y = 0; y < img->h; y++) {
    //     for (uint16_t x = 0; x < img->w; x += 2) {
    //       printf("[%d, %d, %d] ", source[1], source[0], source[2]);
    //       source += 4;
    //     }
    //     printf("\n");
    //   }
    // }

    // glob++;

    // Blur the rest of the image to 
    blur_big(&processed, &blurred);

    convolve_big(&blurred, &processed);


    find_orange_objs();
    find_green_objs();
    find_rand_objs(img);

    combine_measurements();
    delete_outliers();
    find_distances();

    // This is just to show the mask
    // for (uint16_t x=0; x<520; x++){
    //   sum = 0;
    //   for (uint16_t y=0; y<240; y++){
    //     printf("%d ",mask_r[x][y]);
    //     sum += mask_r[x][y];
    //   }
    //   printf("\t %d\n", sum);
    // }



    // copy processed to img for output
    // copy2img(&processed, img);
    // copy2img(&blurred, img);



    clock_t end = clock();
    double time_spent = (double)(end - begin) / CLOCKS_PER_SEC;
    // printf("Time: %lf \n", time_spent);
    // printf("///////////////////////////////////////////////////////////////\n");
  }
  // return &color_mask;
  return img;
}

////////////////////////////////////////////////////////////////////////////////
void read_drone_state(){
  // Get position and transform
  // x defined looking from tables to cyberzoo
  // y defined to the right
  x_loc = - stateGetPositionEnu_f()->x*0.523599 + stateGetPositionEnu_f()->y*0.851965;
  y_loc =  stateGetPositionEnu_f()->x*0.851965 + stateGetPositionEnu_f()->y*0.523599;
  head = stateGetNedToBodyEulers_f()->psi + 0.523599;
  if (head > 3.141592){
    head -= 3.141592*2;
  }

  // Get velocity and transform
  vx_loc = - stateGetSpeedEnu_f()->x*0.523599 + stateGetSpeedEnu_f()->y*0.851965;
  vy_loc =  stateGetSpeedEnu_f()->x*0.851965 + stateGetSpeedEnu_f()->y*0.523599;
  v_tot = sqrt(vx_loc*vx_loc + vy_loc*vy_loc);

  // Get angular rate
  rot_loc = stateGetBodyRates_f()->r;
  // printf("ROT SPEED: %.2lf\n", rot_loc);
}

void create_img(struct image_t *input, struct image_t *output){
  // Set the variables
  output->type = input->type;
  output->w = input->w;
  output->h = input->h;

  output->buf_size = sizeof(uint8_t) * 2 * input->w * input->h;

  output->buf = malloc(input->buf_size);
}

void copy2img(struct image_t *input, struct image_t *output){

  uint8_t *source = (uint8_t *)input->buf;
  uint8_t *dest = (uint8_t *)output->buf;


  for (uint16_t y = 0; y < (output->h); y++) { // horizontal (for every column) 520
    for (uint16_t x = 0; x < (output->w); x += 2) { // vertical (for every pixel in every column) 240
      dest[0] = source[0];
      dest[1] = source[1];
      dest[2] = source[2];
      dest[3] = source[3];

      source += 4;
      dest += 4;
    }
  }
}

// Filter orange color and place black on the rest
void image_specfilt(struct image_t *input, struct image_t *output1, struct image_t *output2, uint8_t y_m, 
                     uint8_t y_M, uint8_t u_m, uint8_t u_M, uint8_t v_m, 
                     uint8_t v_M, uint8_t *mask){

  uint8_t *source = (uint8_t *)input->buf;
  uint8_t *dest1 = (uint8_t *)output1->buf;
  uint8_t *dest2 = (uint8_t *)output2->buf;

  // Copy the creation timestamp (stays the same)
  output1->ts = input->ts;
  output2->ts = input->ts;

  // Go trough all the pixels
  for (uint16_t y = 0; y < input->h; y++) { // horizontal (for every column) 520
    for (uint16_t x = 0; x < input->w; x += 2) { // vertical (for every pixel in every column) 240
      // Check if the color is inside the specified values
      if ( (source[1] >= y_m)
        && (source[1] <= y_M)
        && (source[0] >= u_m)
        && (source[0] <= u_M)
        && (source[2] >= v_m)
        && (source[2] <= v_M)
      ) {
        dest1[0] = 128;  // U
        dest1[1] = 255;  // Y
        dest1[2] = 128;  // V
        dest1[3] = 255;  // Y

        dest2[0] = 128;
        dest2[1] = 255;
        dest2[2] = 128;
        dest2[3] = 255;

        // Only add to the mask those elements with the filtered color
        *(mask+x+y*(output1->w)) = 1;
        *(mask+x+1+y*(output1->w)) = 1;
      } else {
          // dest1[0] = 128;       // U // Grayscale
          // dest1[1] = source[1]; // Y
          // dest1[2] = 128;       // V
          // dest1[3] = source[3]; // Y
          
        dest1[0] = source[0];  // U // Color
        dest1[1] = source[1];  // Y
        dest1[2] = source[2];  // V
        dest1[3] = source[3];  // Y

        dest2[0] = 128;
        dest2[1] = 0;
        dest2[2] = 128;
        dest2[3] = 0;

        *(mask+x+y*(output1->w)) = 0;
        *(mask+x+1+y*(output1->w)) = 0;
      }
      // Go to the next 2 pixels
      dest1 += 4;
      dest2 += 4;
      source += 4;
    }
  }
}

// Filter background color and place grayscale on the rest
void image_bgfilt(struct image_t *input, struct image_t *output, uint8_t y_m, 
                     uint8_t y_M, uint8_t u_m, uint8_t u_M, uint8_t v_m, 
                     uint8_t v_M, bool color, bool remove_ground){

  uint8_t *source = (uint8_t *)input->buf;
  uint8_t *dest = (uint8_t *)output->buf;

  // Copy the creation timestamp (stays the same)
  output->ts = input->ts;

  // Go trough all the pixels
  for (uint16_t y = 0; y < output->h; y++) {
    for (uint16_t x = 0; x < output->w; x += 2) {
      // Check if the color is inside the specified values

      if (remove_ground){
        // // Remove all ground below the floor
        if (x < floors[y]){
            source[1] = 255;
            source[3] = 255;
        }
      }

      if ( (source[1] >= y_m)
        && (source[1] <= y_M)
        && (source[0] >= u_m)
        && (source[0] <= u_M)
        && (source[2] >= v_m) 
        && (source[2] <= v_M)
      ) {
        dest[0] = 128;          // U
        dest[1] = 255;          // Y
        dest[2] = 128;          // V
        dest[3] = 255;          // Y
      } else {
        if (color) {
          // UYVY
          dest[0] = source[0];  // U
          dest[1] = source[1];  // Y
          dest[2] = source[2];  // V
          dest[3] = source[3];  // Y
        } else {
          dest[0] = 128;        // U
          dest[1] = source[1]*2;  // Y
          dest[2] = 128;        // V
          dest[3] = source[3]*2;  // Y
        }
      }
      // Go to the next 2 pixels
      dest += 4;
      source += 4;
    }
  }
}

// Try number 1 to blur an image vertically
void blur_image(struct image_t *input, struct image_t *output){

  uint8_t *source = ((uint8_t *)input->buf);
  uint8_t *dest = ((uint8_t *)output->buf);

  uint16_t val1;
  uint16_t val3;

  int8_t bot_l;
  int8_t top_l;

  for (uint16_t y = 0; y < (output->h); y++) { // horizontal (for every column) 520
    for (uint16_t x = 0; x < (output->w); x += 2) { // vertical (for every pixel in every column) 240
      val1 = 0;
      val3 = 0;

      if (x < 2*blur_l){
        bot_l = -x/2;
        top_l = blur_l+1;
      } else{
        if (x > (output->w)-2*blur_l-2){
          bot_l = -blur_l;
          top_l = (output->w - x)/2;
        } else {
          bot_l = -blur_l;
          top_l = blur_l+1;
        }
      }

      // Blur vertically the image (average of neighbouring pixels)
      for (int8_t i = bot_l; i < top_l; i++){
        val1 += (source + 2*(x+2*i) +  2*y*output->w)[1];
        val3 += (source + 2*(x+2*i) +  2*y*output->w)[3];
      }
      
      (dest + 2*x + 2*y*output->w)[0] = 128;
      (dest + 2*x + 2*y*output->w)[1] = val1/(top_l - bot_l);
      (dest + 2*x + 2*y*output->w)[2] = 128;
      (dest + 2*x + 2*y*output->w)[3] = val3/(top_l - bot_l);

      // dest[0] = (uint8_t)128;
      // dest[1] = (uint8_t)val1/(top_l - bot_l);
      // dest[2] = (uint8_t)128;
      // dest[3] = (uint8_t)val3/(top_l - bot_l);
      
      // //printf("%d ", (dest + 2*x + 2*y*input->w)[3]);
      // //printf("[%d, %d]", x, y);
      // //printf("%d ", dest[1]);
      // source += 4;
      // dest += 4;
    }
    // //printf("\n");
  }
}

// Try number 1 to convolve image
void convolve(struct image_t *input, struct image_t *output){

  uint8_t *source = ((uint8_t *)input->buf)+(2*input->w);
  uint8_t *dest = ((uint8_t *)output->buf)+(2*output->w);

  int16_t max_k1 = 0;
  int16_t min_k1 = 32000;
  int16_t max_k3 = 0;
  int16_t min_k3 = 32000;

  int16_t val1;
  int16_t val3;

  // //printf("first\n");
  for (uint16_t y = 1; y < (input->h)-1; y++) { // horizontal (for every column) 520
    dest += 4;
    source += 4;
    for (uint16_t x = 1; x < (input->w)-1; x += 2) { // vertical (for every pixel in every column) 240
      val1 = 0;
      val3 = 0;

      // for (uint8_t k_y = 0; k_y < k_size; k_y++) {
      //   for (uint8_t k_x = 0; k_x < k_size; k_x++) {
      //     // idx = source + (k_x-k_size/2)*4 + (k_y-k_size/2)*4*(output->w)
      //     val1 += kernel[k_x][k_y] * (source + (k_x-k_size/2)*4 + (k_y-k_size/2)*4*(output->w))[1];
      //     val2 += kernel[k_x][k_y] * (source + (k_x-k_size/2)*4 + (k_y-k_size/2)*4*(output->w))[3];
      //   }
      // }
      // dest[1] = val1;
      // dest[3] = val2;

      // val1 = (source-(4*input->w))[1] + (source+(4*input->w))[1] - 2*(source[1]);
      // val3 = (source-(4*input->w))[3] + (source+(4*input->w))[3] - 4*(source[3]);



      // dest[1] = abs(0*(source-4)[1] + 0*(source+4)[1] + 1*(source-(2*input->w))[1] - 1*(source+(2*input->w))[1] - 0*(source[1]));
      // dest[1] = abs(1*(source-4)[1] + 1*(source+4)[1] + 1*(source-(2*input->w))[1] + 1*(source+(2*input->w))[1] - 4*(source[1]));
      // dest[3] = abs(0*(source-4)[3] + 0*(source+4)[3] + 1*(source-(2*input->w))[3] - 1*(source+(2*input->w))[3] - 0*(source[3]));
      // dest[3] = abs(1*(source-4)[3] + 1*(source+4)[3] + 1*(source-(2*input->w))[3] + 1*(source+(2*input->w))[3] - 1*(source[3]));

      for (uint8_t i = 0; i < 3; i++){
        for (uint8_t j = 0; j < 3; j++){
          val1 += (source + (i-1)*4 + (j-1)*(2*input->w))[1] * kernel[j][i];
          val3 += (source + (i-1)*4 + (j-1)*(2*input->w))[3] * kernel[j][i];
        }
      }

      dest[0] = 128;
      dest[1] = abs(val1);
      dest[2] = 128;
      dest[3] = abs(val3);


      if (dest[1] > max_k1) {
        max_k1 = dest[1];
      } if(dest[1] < min_k1) {
        min_k1 = dest[1];
      } if (dest[3] > max_k3) {
        max_k3 = dest[3];
      } if (dest[3] < min_k3) {
        min_k3 = dest[3];
      }

      dest += 4;
      source += 4;
      // //printf("%d ", dest[1]);

    }
    // //printf("\n");
  }


  // //printf("second\n");
  dest = ((uint8_t *)output->buf)+(2*output->w);

  for (uint16_t y = 1; y < (input->h)-1; y++) { // horizontal (for every column) 520
    dest += 4;
    // source += 4;
    for (uint16_t x = 1; x < (input->w)-1; x += 2) { // vertical (for every pixel in every column) 240
      dest[1] = (dest[1]-min_k1) * 255 / (max_k1-min_k1+1);
      dest[3] = (dest[3]-min_k3) * 255 / (max_k3-min_k3+1);

      if (dest[1] > 60) {
        // *(mask+x+y*(output->w)) = 1;
        mask_r[y][x] = 1;
        dest[1] = 255;
      } else {
        // *(mask+x+y*(output->w)) = 0;
        mask_r[y][x] = 0;
        dest[1] = 0;
      }
      if (dest[3] > 60) {
        // *(mask+x+1+y*(output->w)) = 1;    
        mask_r[y][x+1] = 1;   
        dest[3] = 255; 
      } else {
        // *(mask+x+1+y*(output->w)) = 0;
        mask_r[y][x+1]=0;
        dest[3] = 0;
      }

      // //printf("%d ", mask_r[y][x]);
      dest += 4;
    }
    // //printf("\n");
  }
}

// Blur an image by averaging over pixels next to it (uses blur_w and blur_h)
void blur_big(struct image_t *input, struct image_t *output){

  uint8_t *source = ((uint8_t *)input->buf);
  uint8_t *dest = ((uint8_t *)output->buf);

  int32_t val1;
  int32_t val3;

  int8_t bot_l1; // x
  int8_t top_l1;
  int8_t bot_l2; // y
  int8_t top_l2;

  uint8_t sum_b;

  for (uint16_t y = 0; y < (input->h); y++) { // horizontal (for every column) 520
    for (uint16_t x = 0; x < (input->w); x += 2) { // vertical (for every pixel in every column) 240
      val1 = 0;
      val3 = 0;

      if (x < 2*blur_w2){
        bot_l1 = -x/2;
        top_l1 = blur_w2+1;
      } else{
        if (x > (output->w)-2*blur_w2-2){
          bot_l1 = -blur_w2;
          top_l1 = (output->w - x)/2;
        } else {
          bot_l1 = -blur_w2;
          top_l1 = blur_w2+1;
        }
      }

      if (y < blur_h2){
        bot_l2 = -y;
        top_l2 = blur_h2+1;
      } else{
        if (y > (output->h)-blur_h2-1){
          bot_l2 = -blur_h2;
          top_l2 = (output->h - y);
        } else {
          bot_l2 = -blur_h2;
          top_l2 = blur_h2+1;
        }
      }

      sum_b = (top_l2 - bot_l2) * (top_l1 - bot_l1);

      // //printf("[[%d, %d], [%d, %d]], ", bot_l1, top_l1, bot_l2, top_l2);
      // //printf("%d ", (dest + 2*x + 2*y*output->w)[1]);



      for (int8_t i = bot_l1; i < top_l1; i++){
        for (int8_t j = bot_l2; j < top_l2; j++){
          val1 += (source + 2*(x+2*i) + 2*(y+j)*output->w)[1];// * blur_kernel[i+blur_w2][j+blur_h2];
          val3 += (source + 2*(x+2*i) + 2*(y+j)*output->w)[3];// * blur_kernel[i+blur_w2][j+blur_h2];
          // //printf("[%d, %d]: ", i, j);
          // //printf("%d ", kernel[i][j]);
        }
      }

      // //printf("%d ", val1);

      (dest + 2*x + 2*y*output->w)[0] = 128;
      (dest + 2*x + 2*y*output->w)[1] = abs(val1 / sum_b);
      (dest + 2*x + 2*y*output->w)[2] = 128;
      (dest + 2*x + 2*y*output->w)[3] = abs(val3 / sum_b);

      // //printf("%d ", val1);
      // //printf("%d ", sum_b);
      // //printf("%d ", (dest + 2*x + 2*y*output->w)[1]);
    }
    // //printf("\n");
  }
  // //printf("\n");
}

// Convolve an image by multiplying pixels next to it with big_kernel
void convolve_big(struct image_t *input, struct image_t *output){

  uint8_t *source = ((uint8_t *)input->buf);
  uint8_t *dest = ((uint8_t *)output->buf);

  int16_t max_k1 = 0;
  int16_t min_k1 = 32000;
  int16_t max_k3 = 0;
  int16_t min_k3 = 32000;

  int32_t val1;
  int32_t val3;

  for (uint16_t y = ker_l2; y < (input->h)-ker_l2; y++) { // horizontal (for every column) 520
    for (uint16_t x = ker_l2; x < (input->w)-ker_l2; x += 2) { // vertical (for every pixel in every column) 240
      val1 = 0;
      val3 = 0;

      // printf("%d ", (source + 2*x +2*y*output->w)[1]);

      for (int8_t i = -ker_l2; i < ker_l2+1; i++){
        for (int8_t j = -ker_l2; j < ker_l2+1; j++){
          val1 += (source + 2*(x+2*i) + 2*(y+j)*output->w)[1] * big_kernel[i+ker_l2][j+ker_l2];
          val3 += (source + 2*(x+2*i) + 2*(y+j)*output->w)[3] * big_kernel[i+ker_l2][j+ker_l2];
        }
      }
      // //printf("%d ", abs(val1));

      (dest + 2*x + 2*y*output->w)[0] = 128;
      (dest + 2*x + 2*y*output->w)[1] = abs(val1/sum_k);
      (dest + 2*x + 2*y*output->w)[2] = 128;
      (dest + 2*x + 2*y*output->w)[3] = abs(val3/sum_k);


      if ((dest + 2*x + 2*y*output->w)[1] > max_k1) {
        max_k1 = (dest + 2*x + 2*y*output->w)[1];
      } if((dest + 2*x + 2*y*output->w)[1] < min_k1) {
        min_k1 = (dest + 2*x + 2*y*output->w)[1];
      } if ((dest + 2*x + 2*y*output->w)[3] > max_k3) {
        max_k3 = (dest + 2*x + 2*y*output->w)[3];
      } if ((dest + 2*x + 2*y*output->w)[3] < min_k3) {
        min_k3 = (dest + 2*x + 2*y*output->w)[3];
      }

      // //printf("%d ", (dest + 2*x + 2*y*output->w)[1]);
    }
    // //printf("\n");
  }

  // //printf("");


  for (uint16_t y = ker_l2; y < (input->h)-ker_l2; y++) { // horizontal (for every column) 520
    for (uint16_t x = ker_l2; x < (input->w)-ker_l2; x += 2) { // vertical (for every pixel in every column) 240

      (dest + 2*x + 2*y*output->w)[1] = ((dest + 2*x + 2*y*output->w)[1]-min_k1) * 255 / (max_k1-min_k1+1);
      (dest + 2*x + 2*y*output->w)[3] = ((dest + 2*x + 2*y*output->w)[3]-min_k3) * 255 / (max_k3-min_k3+1);

      if ((dest + 2*x + 2*y*output->w)[1] > 30) {
        mask_r[y][x] = 1;
        (dest + 2*x + 2*y*output->w)[1] = 255;
      } else {
        mask_r[y][x] = 0;
        (dest + 2*x + 2*y*output->w)[1] = 0;
      }
      if ((dest + 2*x + 2*y*output->w)[3] > 30) {
        mask_r[y][x+1] = 1;   
        (dest + 2*x + 2*y*output->w)[3] = 255; 
      } else {
        mask_r[y][x+1]=0;
        (dest + 2*x + 2*y*output->w)[3] = 0;
      }

      // //printf("%d ",(dest + 2*x + 2*y*output->w)[1]);
      // //printf("%d ", mask_r[y][x]);
    }
    // //printf("\n");
  }
}

// Try number 1 to blur mask
void blur_mask(struct image_t *input, uint8_t *mask){

  uint8_t *source = ((uint8_t *)input->buf)+(4*input->w);
  // uint8_t *dest = ((uint8_t *)output->buf)+(4*output->w);

  // int16_t max_k1 = 0;
  // int16_t min_k1 = 32000;
  // int16_t max_k3 = 0;
  // int16_t min_k3 = 32000;

  for (uint16_t y = 1; y < (input->h)-1; y++) { // horizontal (for every column) 520
    // dest += 4;
    source += 4;
    for (uint16_t x = 1; x < (input->w)-1; x += 2) { // vertical (for every pixel in every column) 240

      // for (uint8_t k_y = 0; k_y < k_size; k_y++) {
      //   for (uint8_t k_x = 0; k_x < k_size; k_x++) {
      //     // idx = source + (k_x-k_size/2)*4 + (k_y-k_size/2)*4*(output->w)
      //     val1 += kernel[k_x][k_y] * (source + (k_x-k_size/2)*4 + (k_y-k_size/2)*4*(output->w))[1];
      //     val2 += kernel[k_x][k_y] * (source + (k_x-k_size/2)*4 + (k_y-k_size/2)*4*(output->w))[3];
      //   }
      // }
      // dest[1] = val1;
      // dest[3] = val2;

      // val1 = (source-(4*input->w))[1] + (source+(4*input->w))[1] - 2*(source[1]);
      // val3 = (source-(4*input->w))[3] + (source+(4*input->w))[3] - 4*(source[3]);

      // //printf("%d ", source[1]/255);

      if ((source-4)[1] + (source+4)[1] + (source-(4*input->w))[1] + (source+(4*input->w))[1] + source[1] > (255*3)){
      //   //printf("111");
      //   dest[1] = 255;
        *(mask+x+y*(input->w)) = 1;
      } else {
      //   //printf("000");
      //   dest[1] = 0;
        *(mask+x+y*(input->w)) = 0;
      }

      if ((source-4)[1] + (source+4)[3] + (source-(4*input->w))[3] + (source+(4*input->w))[3] + source[3] > (255*3)){
      //   dest[3] = 255;
        *(mask+x+1+y*(input->w)) = 1;  
      } else {
      //   dest[3] = 0;
        *(mask+x+1+y*(input->w)) = 0;
      }


      // dest += 4;
      source += 4;
    }
    // //printf("\n");
    // dest += 4;
    // source += 4;
  }
}

// Find poles from orange mask
void find_orange_objs(){

  // Get number of appearances
  for (uint16_t x = 0; x < 520; x++){
    sums_o[x] = 0;
    for (uint16_t y = 0; y < 240; y++){
      sums_o[x] += mask_o[x][y];
    }
  }

  // Get derivative in number of appearances
  int16_t der1 = 0;
  count_o = 0;
  bool started = false;

  for (uint16_t x = 1; x < 519; x++){

    // LOGIC WITH THRESHOLD ON 1st DERIVATIVE
    der1 = sums_o[x+1] - sums_o[x-1];

    if (abs(der1) > 50) {
      if (der1 > 0) {
        poles[count_o][0] = x;
        started = true;
      } else {
        poles[count_o][1] = x;
        count_o++;
        started = false;
      }
      x += 8;
    }
  }

  // TODO
  // calculate distance to pole here?

  if (started){
    poles[count_o][1] = 519;
    count_o++;
  }


  // LOGIC WITH 2nd DERIVATIVE
      // der1 = sums[x+1] - sums[x-1];

      // if (abs(der1) > 15){    // if there are more than X 1's difference
      //   der2new = sums[x+1] + sums[x-1] - 2*sums[x];

      //   // if 2nd derivative have opposite sign, we have a pole!
      //   if (((der2new < 0) && (der2old > 0)) || ((der2new > 0) && (der2old < 0))){
  
      //     if (der1 > 0){ // if the pole begins
      //       poles[count][0] = x;
      //     } else {       // if the pole ends
      //       poles[count][1] = x;
      //       count ++;
      //     }

      //   } else {
      //     der2old = der2new;
      //   }
      // } else {
      //   der2old = 0;
      //   der2new = 0;
      // }

  // printf("Poles detected from orange mask\n");
  // for (uint16_t x = 0; x < 10; x++){
  //   printf("[%d, %d] \n", poles[x][0], poles[x][1]);
  // }
  // printf("\n");
}

// Find poles from green mask
void find_green_objs(){

  uint8_t sum;
  uint8_t top_l;

  // Get floor index
  for (int16_t x = 0; x < 520; x++){
    for (int16_t y =239; y > -1; y--){
      sum = 0;
      // the first white pixel
      if ((mask_g[x][y] == 1)){
        // define upper boundary seach
        if (y>15){ // if there are more than 8 pixels in the row left
          top_l = 15;
        } else { // else
          top_l = y;
        }
        for (uint8_t i=0; i<top_l; i++){
          sum += mask_g[x][y+i];
        }
        // if not just one random pixel
        if (sum > top_l - 2){
          floors[x] = y;
          break;
        }
      }
      if (y == 0){
        floors[x] = y;
      }
    }
  }


  // Get floor slope
  int8_t der1 = 0;
  count_g = 0;
  bool started = false;


  check_green = true;

  // if close to the top (x+)
  if (x_loc > top_x_green){
    // and not looking inside the circle
    if ((head < head_x_t_green) && (head > -head_x_t_green)){
      check_green = false;
    }
  } 

  // if close to the bottom (x-)
  if (x_loc < -top_x_green){
    // and not looking inside the circle
    if ((head > head_x_b_green) || (head < -head_x_b_green)){
      check_green = false;
    }
  }

  // if close to the right (y+)
  if (y_loc > top_y_green){
    // and not looking inside the circle
    if ((head > -head_y_t_green) || (head < -head_y_b_green)){
      check_green = false;
    }
  }

  // if close to the left (y-)
  if (y_loc < -top_y_green){
    // and not looking inside the circle
    if ((head < head_y_t_green) || (head > head_y_b_green)){
      check_green = false;
    }
  }


  // Only perform gradient based object search if inside circle and looking 
  // inside to avoid detecting the edge of the floor
  if (check_green){
    for (uint16_t x = 2; x < 518; x++){

      // LOGIC WITH THRESHOLD ON 1st DERIVATIVE
      der1 = floors[x+2] + floors[x+1] - floors[x-1] - floors[x-2];

      if (abs(der1) > 18) {
        if (der1 < 0){
          poles[idx_g+count_g][0] = x;
          started = true;
        } else {
          if ((poles[idx_g+count_g][0] != 0) || (count_g < 0.5)){
            poles[idx_g+count_g][1] = x;
            count_g++;
            started = false;
          }
        }
        x += 8;
      }
    }
    if (started){
      poles[idx_g+count_g][1] = 519;
      count_g++;
    }
  } else {
    // TODO implement logic where you don't search from given pixel where the corner is
    printf("NOT CHECKING !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");
    printf("TOO CLOSE TO THE BORDER FOR GREEN MASK\n");
  }


  // printf("Poles detected from green mask\n");
  // for (uint16_t x = idx_g; x < idx_g+9; x++){
  //   printf("[%d, %d] \n", poles[x][0], poles[x][1]);
  // }
  // printf("\n");
}

// THIS FUNCTION IS COPIED FROM find_orange_objs
// TODO -> IDENTIFY OBJECTS COMPARING WITH GREEN MASK LOCATION
//      -> is this even going to be a good idea?
//      -> otherwise too much noise


// Find poles from green mask
void find_rand_objs(struct image_t *input){
  // angle from pose to
  float head_c1 = atan2(( 4.35-y_loc), ( -5.35-x_loc)); // start of curtain (-, +) 
  float head_c2 = atan2((-4.35-y_loc), (  5.35-x_loc)); // end of curtain (+, -)
  count_r = 0;

  // printf("[H, C1]: [%.1lf, %.1lf]\n", head*57.29, head_c1*57.29);
  // printf("[H, C2]: [%.1lf, %.1lf]\n", head*57.29, head_c2*57.29);


  // what are you looking at: curtain, free, or mix
  if ((head + 0.6981 < head_c1) && (head - 0.6981 > head_c2)){
    // if pointing only at the curtain
    rand_search_type = 1;
  } else {
    // if pointing at crossing 1 (-, +)
    if ((head - 0.6981 < head_c1) && (head + 0.6981 > head_c1)){
      rand_search_type = 2;
      px_change = (head_c1 - head + 0.6981)*372.4;
    } else {
      // if pointing at crossing 2 (+, -)
      if ((head - 0.6981 < head_c2) && (head + 0.6981 > head_c2)){
        rand_search_type = 3;
        px_change = (head_c2 - head)*372.6*0.94 + 260; // 0.94 is the correction factor because mapping is not linear
      } else {
        // if pointing to the curtain-free sides
        rand_search_type = 4;
      }
    }
  }

  float avg_white_f = 0;

  // Get number of appearances (integral of white pixels per column)
  for (uint16_t x = 0; x < 520; x++){
    sums_r[x] = 0;
    for (uint16_t y = 0; y < 240; y++){
      sums_r[x] += mask_r[x][y];
    }

    if (rand_search_type == 2){
      if (x > px_change){
        avg_white_f = (avg_white_f*(x-px_change-1) + sums_r[x]) / (x-px_change);
      }
    }

    if (rand_search_type == 3){
      if (x < px_change){
        avg_white_f = (avg_white_f*x + sums_r[x]) / (x+1);
      }
    }

    if (rand_search_type == 4){
      avg_white_f = (avg_white_f*x + sums_r[x]) / (x+1);
    }
  }
  // printf("\n");

  uint16_t avg_white = (uint16_t)avg_white_f;
  int8_t min_smooth;
  int16_t max_smooth;
  uint16_t sum;

  if (avg_white < 20){
    avg_white = 20;
  }

  // printf("AVG WHITE: %d\n", avg_white);
  // printf("PX_CHANGE: %d\n", px_change);

  // Smooth the sums_r function to get clearer derivatives
  for (uint16_t x = 0; x < 520; x++){
    if (x < 2){
      min_smooth = -x;
    } else {
      min_smooth = -2;
    }
    if (x > 517){
      max_smooth = 520-x;
    } else{
      max_smooth = 3;
    }
    sum = 0;
    for (int8_t i = min_smooth; i<max_smooth; i++){
      sum += sums_r[x+i];
    }
    sums_r_smooth[x] = sum/(max_smooth - min_smooth);
  }

  // // print mask
    // if (rand_search_type == 1){
    //   printf("TYPE 1\n");
    //   uint16_t sum_tt;
    //   for (uint16_t x=0; x<520; x++){
    //     sum_tt = 0;
    //     for (uint16_t y=0; y<240; y++){

    //       if (y<sums_r_smooth[x]){
    //         printf("1 ");
    //       } else {
    //         printf("0 ");
    //       }
    //       // printf("%d ",mask_r[x][y]);
    //       sum_tt += mask_r[x][y];
    //     }
    //     printf("\t %d\n", sum_tt);
    //   }
    // }
    // if (rand_search_type == 2){
    //   printf("TYPE 2\n");
    //   uint16_t sum_tt;
    //   for (uint16_t x=0; x<520; x++){
    //     sum_tt = 0;
    //     for (uint8_t y=0; y<240; y++){

    //       if ((y<sums_r_smooth[x]) || (x == px_change)){
    //         printf("1 ");
    //       } else {
    //         if ((y>px_change) && (y == avg_white)){
    //           printf("1 ");
    //         } else{
    //           printf("0 ");
    //         }
    //       }
    //       // printf("%d ",mask_r[x][y]);
    //       sum_tt += mask_r[x][y];
    //     }
    //     printf("\t %d\n", sum_tt);
    //   }
    // }
    // if (rand_search_type == 3){
    //   printf("TYPE 3\n");
    //   uint16_t sum_tt;
    //   for (uint16_t x=0; x<520; x++){
    //     sum_tt = 0;
    //     for (uint16_t y=0; y<240; y++){

    //       if ((y<sums_r_smooth[x]) || (y == px_change)){
    //         printf("1 ");
    //       } else {
    //         if ((y<px_change) && (x == avg_white)){
    //           printf("1 ");
    //         } else{
    //           printf("0 ");
    //         }
    //       }
    //       // printf("%d ",mask_r[x][y]);
    //       sum_tt += mask_r[x][y];
    //     }
    //     printf("\t %d\n", sum_tt);
    //   }
    // }
    // if (rand_search_type == 4){
    //   printf("TYPE 4\n");
    //   uint16_t sum_tt;
    //   for (uint16_t x=0; x<520; x++){
    //     sum_tt = 0;
    //     for (uint16_t y=0; y<240; y++){

    //       if ((y<sums_r_smooth[x]) || (x == avg_white)){
    //         printf("1 ");
    //       } else {
    //         printf("0 ");
    //       }
    //       // printf("%d ",mask_r[x][y]);
    //       sum_tt += mask_r[x][y];
    //     }
    //     printf("\t %d\n", sum_tt);
    //   }
    // }
  //

  started_rand = false;

  // curtain all over logic
  if (rand_search_type == 1){
    // printf("LOGIC 1\n");
    logic_curtain(1, 519, input);
  }

  // curtain on the left, free on the right logic
  if (rand_search_type == 2){
    // printf("LOGIC 2\n");
    logic_curtain(1, px_change-1, input);
    logic_free(px_change, 520);
  }

  // free on the left, curtain on the right logic
  if (rand_search_type == 3){
    // printf("LOGIC 3\n");
    logic_free(1, px_change);
    logic_curtain(px_change+1, 520, input);
  }

  // free all over logic
  if (rand_search_type == 4){
    // printf("LOGIC 4\n");
    logic_free(1, 520);
  }

  if (started_rand){
    poles[idx_r+count_r][1] = 519;
    count_r++;
  }


  // printf("Edges detected from random gradients\n");
  // for (uint16_t x = idx_r; x < idx_r+10; x++){
  //   printf("[%d, %d] \n", poles[x][0], poles[x][1]);
  // }
  // printf("\n");

}

void logic_curtain(uint16_t idx1, uint16_t idx2, struct image_t *input){  

  int16_t der1_r = 0;
  int8_t lim_sum;
  uint16_t sum_cc_top;
  uint16_t sum_cc_bot;
  uint8_t *source = ((uint8_t *)input->buf);
  // printf("IDX1, IDX2]: [%d, %d]\n", idx1, idx2);
  // LOGIC WITH THRESHOLD ON 1st DERIVATIVE
  for (uint16_t x = idx1; x < idx2; x++){


    // Get derivative in number of appearances
    der1_r = sums_r_smooth[x+1] - sums_r_smooth[x-1];
    // printf("%d ", der1_r);
    // if the gradient is enough
    if (abs(der1_r) > 40) {

      // identify if start or end object by comparing the colors 
      // (backgroun -> darker)
      if (x < 5){
        lim_sum = -x;
      } else {
        if (x > idx2-5){
          lim_sum = -idx2+x;
        } else{
          lim_sum = -5;
        }
      }

      sum_cc_bot = 0;
      sum_cc_top = 0;
      // printf("\n||||||||||||||||||\n");
      // printf("%d, %d \n", x, lim_sum);


      for (int8_t x_sum = lim_sum; x_sum < 0; x_sum++){
        // printf("[cbot, ctop]: [%d, %d]\n", (source + 2*160 + 2*(x+x_sum)*input->w)[1], (source + 2*160 + 2*(x-x_sum)*input->w)[1]);
        sum_cc_bot += (source + 2*160 + 2*(x+x_sum)*input->w)[1]; // color diff at vertical pixel 70 (from top?)
        sum_cc_top += (source + 2*160 + 2*(x-x_sum)*input->w)[1];
      }
      // printf("[%d, %d]", sum_cc_bot, sum_cc_bot);
      // printf("\n^^^^^^^^^^^^^\n");

      // If bottom is darker, object starts
      if (sum_cc_bot < sum_cc_top){
        poles[idx_r+count_r][0] = x;
        started_rand = true;
      } else { // else, object is ending

        // check if this measurement is not from the previous object on the free side
        if (rand_search_type == 3){
          if (started_rand){
            poles[idx_r+count_r][1] = x;
            started_rand = false;
            count++;
          } else{
            if (count_r > 0){
              if (x-poles[idx_r+count_r-1][1] < 20){
                poles[idx_r+count_r-1][1] = (poles[idx_r+count_r-1][1] + x)/2;
                started_rand = false;
              }
            }
          }
        } else{
          poles[idx_r+count_r][1] = x;
          started_rand = false; 
          count_r++;
        }
      }
      x+=10;
    }
  }
  // printf("\n");
  
}

void logic_free(uint16_t idx1, uint16_t idx2){
  uint16_t obj_width;
  uint16_t x_run;

  for (uint16_t x = idx1; x < idx2; x++){

    // Calculate object only if there is a hole in the noise
    if (sums_r_smooth[x] < 15){
      obj_width = 0;

      // printf("possible hole: ");
      for (x_run = x; x_run < idx2; x_run++){
        // printf("%d ", x_run);
        if (sums_r_smooth[x_run] < 10){
          obj_width++;
        } else {
          break;
        }
      }
      // printf("\n");

      // If no more than 10 pixels for 10 pixels, consider it a hole
      if (obj_width > 15){

        // if object already started
        if (x == idx1){
          poles[idx_r+count_r][1] = x_run;
          started_rand = false;
          count_r++;
          x = x_run+5;
        } else { 
          //if object did not finish
          if (x_run == idx2-1){
            poles[idx_r+count_r][0] = x;
            started_rand = true;
          } else{ // else: if object is in the middle (starts and finishes)
            poles[idx_r+count_r][0] = x;
            poles[idx_r+count_r][1] = x_run;
            started_rand = false;
            count_r++;
            x = x_run+5;
          }
        }
      } else{
        if ((rand_search_type == 2) && (x - idx1 < 15)){
          if (started_rand){
            poles[idx_r+count_r][1] = px_change;
            started_rand = false;
          }
        }
      }
    }
  }

  printf("\n");

}

// Associate measurements to each other
void combine_measurements(){
  uint16_t dist;
  count = 0;

  // printf("Count orange: %d\n", count_o);
  // printf("Count green: %d\n", count_g);


  // Merge orange and green objects
  if (count_g == 0){ // if no green is detected
    for (uint8_t co = 0; co < count_o; co++){
      poles_comb[count][0] = poles[co][0];
      poles_comb[count][1] = poles[co][1];
      poles_comb[count][2] = poles[co][2];
      poles_comb[count][3] = 1;
      count++;
    }
  } else {
    if (count_o == 0){
      for (uint8_t cg = 0; cg < count_g; cg++){
        poles_comb[count][0] = poles[idx_g+cg][0];
        poles_comb[count][1] = poles[idx_g+cg][1];
        poles_comb[count][2] = poles[idx_g+cg][2];
        poles_comb[count][3] = 1;
        count++;
      }
    } else {  
      for (uint8_t co = 0; co < count_o; co++){
        for (uint8_t cg = 0; cg < count_g; cg++){
          dist = abs(poles[co][0] - poles[idx_g+cg][0]);
          if (dist < threshold){
            // Object is the same

            poles_comb[count][0] = (poles[co][0] + poles[idx_g+cg][0])/2;
            poles_comb[count][1] = (poles[co][1] + poles[idx_g+cg][1])/2;
            poles_comb[count][2] = (poles[co][2] + poles[idx_g+cg][2])/2;
            poles_comb[count][3] = 1;

            obj_merged[cg] = 1;
            count++;

            break;
          } 

          // if orange pole is not detected by green filter, add it to list
          if (cg == count_g-1) {
            poles_comb[count][0] = poles[co][0];
            poles_comb[count][1] = poles[co][1];
            poles_comb[count][2] = poles[co][2];
            poles_comb[count][3] = 1;
            count++;
          }
        }
      }

      // add green obj not detected by orange filter
      for (uint8_t cg = 0; cg < count_g; cg++){
        // printf("Obj [idx] merged: %d\n", !obj_merged[cg]);          
        if (!obj_merged[cg]){
          poles_comb[count][0] = poles[idx_g+cg][0];
          poles_comb[count][1] = poles[idx_g+cg][1];
          poles_comb[count][2] = poles[idx_g+cg][2];
          poles_comb[count][3] = 0;
          count++;      
          obj_merged[cg] = 0; // Useful not to compute distance from width if not orange pole
        }
      }
    }
  }


  // printf("Combined measurements\n");
  // for (uint16_t x = 0; x < 10; x++){
  //   printf("[%d, %d, %d] \n", poles_comb[x][0], poles_comb[x][1], poles_comb[x][3]);
  // }
  // printf("\n");

  // printf("Edged measurements\n");
  // for (uint16_t x = 0; x < 10; x++){
  //   printf("[%d, %d] \n", poles[idx_r+x][0], poles[idx_r+x][1]);
  // }
  // printf("\n");


  // // Clean obj_merged list
  // for (uint8_t x=0; x<100; x++){
  //   obj_merged[x] = 0;
  // }


  // Merge objects from edge detector
  if ((count_g == 0) && (count_o == 0)){
    for (uint8_t cr = 0; cr < count_r; cr++){
      poles_comb[count][0] = poles[idx_r+cr][0];
      poles_comb[count][1] = poles[idx_r+cr][1];
      poles_comb[count][2] = poles[idx_r+cr][2];
      poles_comb[count][3] = 0;
      count++;
      // printf("WRITING HERE 1\n");
    }
  } else {
    for (uint8_t cr = 0; cr < count_r; cr++){
      for (uint8_t c = 0; c < count; c++){

        dist = abs(poles[idx_r+cr][0] - poles_comb[c][0]);

        if (dist < threshold){

          // printf("WRITING HERE 2/n");
          // Object is the same, compute averaged values
          poles_comb[c][0] = (2*poles_comb[c][0] + poles[idx_r+cr][0])/3;
          poles_comb[c][1] = (2*poles_comb[c][1] + poles[idx_r+cr][1])/3;
          poles_comb[c][2] = (2*poles_comb[c][2] + poles[idx_r+cr][2])/3;
          break;
        }

        // if random object is not detected by green or orange filter, add it to list
        if (c == count-1) {
          // printf("WRITING HERE 3\n");
          poles_comb[count][0] = poles[idx_r+cr][0];
          poles_comb[count][1] = poles[idx_r+cr][1];
          poles_comb[count][2] = poles[idx_r+cr][2];
          poles_comb[count][3] = 0;
          count++;
        }
      }
    }
  }






}

// If an object is detected more than XX times in the last XX views, then take it as valid
void delete_outliers(){
  uint16_t dist;
  uint8_t new_count = 0;
  uint16_t min_dist;
  uint8_t min_idx;

  // //printf("count intertia: %d \n", count_inertia);
  // //printf("Intertial measurements 1\n");
  // for (uint16_t x = 0; x < 10; x++){
  //   //printf("[%d, %d, %d] \n", poles_w_inertia[x][0], poles_w_inertia[x][1], poles_w_inertia[x][3]);
  // }
  // //printf("\n");

  // if list obj_w_inertia is empty, add all seen cones
  if (count_inertia == 0){
    for (uint8_t c_old = 0; c_old < count; c_old++){

      // Add object location
      poles_w_inertia[c_old][0] = poles_comb[c_old][0];
      poles_w_inertia[c_old][1] = poles_comb[c_old][1];
      poles_w_inertia[c_old][2] = poles_comb[c_old][2];

      // Add # of times seen
      poles_w_inertia[c_old][3] = 2;

      // Add object type
      poles_w_inertia[c_old][4] = poles_comb[c_old][3];
    }
    new_count = count;
  } else{
    // if object is rotating or moving fast, then take all the measurements
    if ((v_tot > 1) || (rot_loc > 0.4)){
      // printf("rotating too fast, calm down\n");
      for (uint8_t c_old = 0; c_old < count; c_old++){

        // Add object location
        poles_w_inertia[c_old][0] = poles_comb[c_old][0];
        poles_w_inertia[c_old][1] = poles_comb[c_old][1];
        poles_w_inertia[c_old][2] = poles_comb[c_old][2];

        // Add # of times seen
        poles_w_inertia[c_old][3] = 9;

        // Add object type
        poles_w_inertia[c_old][4] = poles_comb[c_old][3];

      }
      new_count = count - count_inertia;

    } else{
      // o/w for all objects seen
      for (uint8_t c_old = 0; c_old < count; c_old++){
        min_dist = 1000;
        for (uint8_t c_inrt = 0; c_inrt < count_inertia; c_inrt++){

          dist = abs(poles_comb[c_old][0] - poles_w_inertia[c_inrt][0]);


          // //printf("Threshold: %d  of cone [%d, %d] with cone [%d, %d];\tNew count: %d\t\t", dist,  
          //                 poles_comb[c_old][0], poles_comb[c_old][1],
          //                 poles_w_inertia[c_inrt][0], poles_w_inertia[c_inrt][1], new_count);


          if (dist<min_dist){
            min_dist = dist;
            min_idx = c_inrt;
          }

        if ((c_inrt == count_inertia-1) && (min_dist > threshold)){
            // if object was not seen before
            // Add object location
            poles_w_inertia[count_inertia+new_count][0] = poles_comb[c_old][0];
            poles_w_inertia[count_inertia+new_count][1] = poles_comb[c_old][1];
            poles_w_inertia[count_inertia+new_count][2] = poles_comb[c_old][2];

            // Add # of times seen
            poles_w_inertia[count_inertia+new_count][3] = 2;

            // Add object type
            poles_w_inertia[count_inertia+new_count][4] = poles_comb[c_old][3];

            new_count++;
          }
        }

        // if object was seen before
        if (min_dist < associate_threshold){
          // Update the measurement to new location
          poles_w_inertia[min_idx][0] = poles_comb[c_old][0];
          poles_w_inertia[min_idx][1] = poles_comb[c_old][1];
          poles_w_inertia[min_idx][2] = poles_comb[c_old][2];
          // Update times seen
          poles_w_inertia[min_idx][3] += 2;
          // Update type
          poles_w_inertia[min_idx][4] = poles_comb[c_old][3];
        }
      }
    }
  }

  // printf("count intertia: %d \n", count_inertia);
  // printf("New count: %d\n", new_count);
  // printf("Intertial measurements 2\n");
  // for (uint16_t x = 0; x < 10; x++){
  //   printf("[%d, %d, %d] \n", poles_w_inertia[x][0], poles_w_inertia[x][1], poles_w_inertia[x][3]);
  // }
  // printf("\n");


  count_inertia += new_count;
  final_count = 0;
  elem_to_rm = 0;


  for (uint8_t c_inrt = 0; c_inrt < count_inertia; c_inrt++){
    poles_w_inertia[c_inrt][3]--;

    // if the obj was seen negative times
    if (poles_w_inertia[c_inrt][3] < 1){
      idx_to_rm[elem_to_rm] = c_inrt;
      elem_to_rm ++;

      // printf("REMOVEEE!!!\n");
      // printf("Obj idx: %d\t Obj to rm: %d\n", c_inrt, elem_to_rm-1);

    } else {

      if (poles_w_inertia[c_inrt][3] > 7){

        if (poles_w_inertia[c_inrt][3] > 30){
          poles_w_inertia[c_inrt][3] = 30;
        }

        final_objs[final_count][0] = poles_w_inertia[c_inrt][0];
        final_objs[final_count][1] = poles_w_inertia[c_inrt][1];
        final_objs[final_count][2] = poles_w_inertia[c_inrt][2];
        final_objs[final_count][3] = poles_w_inertia[c_inrt][4];

        final_count++;
      }
    }
  }

  uint8_t last_idx = count_inertia-1;
  uint8_t rm_idx_i;

  for (uint8_t c_rm = 0; c_rm < elem_to_rm; c_rm++){
    rm_idx_i = idx_to_rm[c_rm];
    for (uint8_t c_mv = rm_idx_i; c_mv < count_inertia-1; c_mv++){
      // //printf("Moving %d to %d\n", c_mv+1, c_mv);
      poles_w_inertia[c_mv][0] = poles_w_inertia[c_mv+1][0];
      poles_w_inertia[c_mv][1] = poles_w_inertia[c_mv+1][1];
      poles_w_inertia[c_mv][2] = poles_w_inertia[c_mv+1][2];
      poles_w_inertia[c_mv][3] = poles_w_inertia[c_mv+1][3];
      poles_w_inertia[c_mv][4] = poles_w_inertia[c_mv+1][4];
    }
    // //printf("Deleting obj %d\n", last_idx);
    poles_w_inertia[last_idx][0] = 0;
    poles_w_inertia[last_idx][1] = 0;
    poles_w_inertia[last_idx][2] = 0;
    poles_w_inertia[last_idx][3] = 0;
    poles_w_inertia[last_idx][4] = 0;
    last_idx--;
  }

  count_inertia -= elem_to_rm;



  // //printf("Intertial measurements 3\n");
  // for (uint16_t x = 0; x < 10; x++){
  //   //printf("[%d, %d, %d] \n", poles_w_inertia[x][0], poles_w_inertia[x][1], poles_w_inertia[x][3]);
  // }

  // //printf("\n");


}

void find_distances(){

  // //printf("Final count: %d\n", final_count);

  float dist1;
  float dist2;

  // angle from pose to
  float head_1 = atan2(( 3.85-y_loc), ( 3.85-x_loc)); // top r (+, +) 
  float head_2 = atan2(( 3.85-y_loc), (-3.85-x_loc)); // bot r (+, -)
  float head_3 = atan2((-3.85-y_loc), (-3.85-x_loc)); // bot l (-, -)
  float head_4 = atan2((-3.85-y_loc), ( 3.85-x_loc)); // top l (-, +))

  // // angle from pose to 
  // float head_1 = atan2(( 3.85-x_loc), ( 3.85-y_loc)); // top r (+, +) 
  // float head_2 = atan2(( 3.85-x_loc), (-3.85-y_loc)); // bot r (+, -)
  // float head_3 = atan2((-3.85-x_loc), (-3.85-y_loc)); // bot l (-, -)
  // float head_4 = atan2((-3.85-x_loc), ( 3.85-y_loc)); // top l (-, +))


  // //printf("HEADS: \n");
  // //printf("H1: %.2lf\n", head_1);
  // //printf("H2: %.2lf\n", head_2);
  // //printf("H3: %.2lf\n", head_3);
  // //printf("H4: %.2lf\n", head_4);

  // printf("Own loc: [%.2lf, %.2lf]\n", x_loc, y_loc);
  // printf("Our head: %.2lf\n", head);
  // printf("Final count: %d\n", final_count);

  // printf("||||||||||||||||||\n");
  for (uint8_t idx = 0; idx < final_count; idx++){
    final_objs[idx][2] = px_dist_scale/(final_objs[idx][1]-final_objs[idx][0]);
  }



    if (floors[avg_px] != 0){
      // Get angle to obj from linear mapping on the pixel location
      head_obj = head + (-40+4*avg_px/26) * 0.017453;
      // printf("Avg pxl: %d\n", avg_px);
      // printf("Head obj: %.2lf\n", head_obj);

      // if pointing to top edge
      if ((head_obj < head_1) && (head_obj > head_4)){
        // x_lim = 3.85-x_loc;
        // y_lim = 
        // printf("1\n");

        d_to_edge = (3.85-x_loc) / cos(head_obj);
      }
      // if pointing to the right edge
      if ((head_obj > head_1) && (head_obj < head_2)){
        d_to_edge = (3.85-y_loc) / sin(head_obj);
        // printf("2\n");
      }
      // if pointing to the bottom edge
      if ((head_obj > head_2) || (head_obj < head_3)){
        d_to_edge = (-3.85-x_loc) / cos(head_obj);
        // printf("3\n");
      }
      // if pointing to the left edge
      if ((head_obj > head_3) && (head_obj < head_4)){
        d_to_edge = (-3.85-y_loc) / sin(head_obj);
        // printf("4\n");
      }



    // printf("[%.1lf, %.1lf]\n", final_objs[idx][0], final_objs[idx][1]);

    // printf("d_to_edge: %.2lf m\n", d_to_edge);

    // printf("Floor @ obj: %d\n", floors[avg_px]);
    // printf("Floor next to obj: %d, %d\n", floors[(uint16_t)final_objs[idx][0]-15], floors[(uint16_t)final_objs[idx][1]+15]);


    ////////////////// Find distance from object thickness //////////////////
    // (assuming thickness of 0.3 m -> pole thickness)

    dist2 = px_dist_scale/(final_objs[idx][1]-final_objs[idx][0]);

    // printf("[D1, D2]: [%.1lf, %.1lf]\n", dist1, dist2);
    // printf("[1:orange, 0:other] = %d\n", (uint8_t)(final_objs[idx][3]));


    // Only compute distance from width if object was detected by orange filter
    if ((uint8_t)(final_objs[idx][3]) == 1){

      if (dist1 > 0.05){
        final_objs[idx][2] = (2*dist2 + dist1) / 3;
      } else {
        final_objs[idx][2] = dist2;
      }
    } else {
      final_objs[idx][2] = dist1;
    }
  }

  // printf("||||||||||||||||||\n");

  // printf("Final measurements\n");
  // for (uint16_t x = 0; x < 10; x++){
  //   printf("[%f, %f, %f] \n", final_objs[x][0], final_objs[x][1], final_objs[x][2]);
  // }

  // printf("\n");


}

void observer_node_init(void){
  cv_add_to_device(&COLORFILTER_CAMERA, observer_func, OPENCVDEMO_FPS);
}