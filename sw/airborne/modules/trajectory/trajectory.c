/*
 * Copyright (C) Team Wonder
 *
 * This file is part of paparazzi
 *
 * paparazzi is free software; TRAJECTORY_You can redistribute it and/or modifTRAJECTORY_Y
 * it under the terms of the GNU General Public License as published bTRAJECTORY_Y
 * the Free Software Foundation; either version 2, or (at TRAJECTORY_Your option)
 * anTRAJECTORY_Y later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANTRAJECTORY_Y WARRANTTRAJECTORY_Y; without even the implied warrantTRAJECTORY_Y of
 * MERCHANTABILITTRAJECTORY_Y or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * TRAJECTORY_You should have received a copTRAJECTORY_Y of the GNU General Public License
 * along with paparazzi; see the file COPTRAJECTORY_YING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */
/**
 * @file "modules/circle/circle.c"
 * @author Team Wonder
 * Just pathing in circle
 */

#include "modules/trajectory/trajectory.h"
#include "generated/flight_plan.h"
#include "firmwares/rotorcraft/navigation.h"
#include "state.h"
#include "modules/observer/node.h"




enum trajectory_mode_t {
  CIRCLE,
  SQUARE,
  LACE,
  INVERTED_LACE
};

enum trajectory_mode_t trajectory_mode = CIRCLE;
int TRAJECTORY_L = 1800; //for a dt f 0.0011, razor thin margins
// int TRAJECTORY_L = 1550; //for a dt f 0.0004
int TRAJECTORY_D = 0;
float TRAJECTORY_X=0;
float TRAJECTORY_Y=0;
float TRAJECTORY_SWITCHING_TIME=20;

float current_time = 0;
int square_mode = 1;
int lace_mode = 1;
int mode=1;


int AVOID_number_of_objects = 0;
float AVOID_h1,AVOID_h2;
float AVOID_d;
float AVOID_safety_angle = 10 * M_PI/180;
float AVOID_FOV = 80;
int AVOID_PERCENTAGE_THRESHOLD=20;
float AVOID_OF_angle = 5 * M_PI/180;
float AVOID_objects[100][3];
float AVOID_slow_dt = 0.0002;
float AVOID_normal_dt = 0.0011;

float dt=0.0011; // 0.6 m/s speed

void trajectory_init(void){}

void trajectory_periodic(void)
{

// Set heading according to previous goal


AVOID_number_of_objects = 0;
unpack_object_list();
count_objects();

current_time += dt;
//dt = AVOID_normal_dt;
int r = TRAJECTORY_L/2 - TRAJECTORY_D;   

switch (trajectory_mode){
  case CIRCLE:

    circle(current_time, &TRAJECTORY_X, &TRAJECTORY_Y, r);
    
    if (current_time > TRAJECTORY_SWITCHING_TIME){ //move to another mode
        current_time=0;
        TRAJECTORY_Y=0;
        TRAJECTORY_X=0;  
        square_mode=1; 
        trajectory_mode = SQUARE;
      }

    break;
  case SQUARE:

    square(dt, &TRAJECTORY_X, &TRAJECTORY_Y, r);

    if (current_time > TRAJECTORY_SWITCHING_TIME){ //move to another mode
      current_time=0;
      TRAJECTORY_Y=0;
      TRAJECTORY_X=0; 
      lace_mode=1;
      trajectory_mode = LACE;
    }

    break;
  case LACE:

    lace(dt, &TRAJECTORY_X, &TRAJECTORY_Y, r);

      if (current_time > TRAJECTORY_SWITCHING_TIME){ //move to another mode
        current_time=0;
        TRAJECTORY_Y=0;
        TRAJECTORY_X=0; 
        lace_mode=1;
        trajectory_mode = INVERTED_LACE;
      }

      break;
  case INVERTED_LACE:

    lace_inverted(dt, &TRAJECTORY_X, &TRAJECTORY_Y, r);

      if (current_time > TRAJECTORY_SWITCHING_TIME){ //move to another mode
        current_time=0;
        trajectory_mode = CIRCLE;
      }

      break;
  default:
    break;
  }

float x_rotated=TRAJECTORY_X*0.5+TRAJECTORY_Y*0.866025;
float y_rotated=-TRAJECTORY_X*0.866025+TRAJECTORY_Y*0.5;

//after having the next way point --> double check with OF
// bool change_heading=safety_check_optical_flow(GLOBAL_OF_VECTOR, x_rotated, y_rotated);

// if(change_heading){
//   float next_heading=safe_heading(GLOBAL_OF_VECTOR)+stateGetNedToBodyEulers_f()->psi;
//   FLOAT_ANGLE_NORMALIZE(next_heading);
//   float x_next = stateGetPositionEnu_i()->x + 1 * cos(next_heading); //confirm this angles tomorrow with values in excel!!
//   float y_next = stateGetPositionEnu_i()->y + 1 * sin(next_heading);
//   waypoint_set_xy_i(WP_STDBY,x_next,y_next);
// }
// else{
//   waypoint_set_xy_i(WP_STDBY,x_rotated,y_rotated);
// }

waypoint_set_xy_i(WP_GOAL,x_rotated,y_rotated);
waypoint_set_xy_i(WP_TRAJECTORY,x_rotated,y_rotated);
nav_set_heading_towards_waypoint(WP_GOAL);

  // Deallocate
// float *GLOBAL_OF_VECTOR = NULL; 
// float *GLOBAL_OBJECTS_VECTOR = NULL;
}


//***************************************** AVOIDANCE STRATEGIES *****************************************

void avoidance_straight_path(){

// h1 and h2 are the left and right headings in degrees of the obstable in a realtive 
// reference frame w/ origin in the center of the FOV 

float heading_increment = 0;


for(int i; i < AVOID_number_of_objects; i++){


  if(AVOID_objects[i][0] > 0 && AVOID_objects[i][0] < AVOID_safety_angle || AVOID_objects[i][0]*AVOID_objects[i][1] < 0){    
    if(i==0 || fabs(AVOID_objects[i][0]) > fabs(AVOID_objects[i-1][0]) || fabs(AVOID_objects[i][1]) > fabs(AVOID_objects[i-1][1])){
      heading_increment = AVOID_objects[i][0] - AVOID_safety_angle; // linear function to adapt heading change
  }
  }

  if(AVOID_objects[i][1] < 0 && AVOID_objects[i][1] > -1*AVOID_safety_angle || AVOID_objects[i][0]*AVOID_objects[i][1] < 0){    
    if(i==0 || fabs(AVOID_objects[i][0]) > fabs(AVOID_objects[i-1][0]) || fabs(AVOID_objects[i][1]) > fabs(AVOID_objects[i-1][1])){
      heading_increment = AVOID_objects[i][0] + AVOID_safety_angle; // linear function to adapt heading change
}
  }

  if(AVOID_objects[i][1] > -1*AVOID_safety_angle && AVOID_objects[i][1] < 0 || AVOID_objects[i][0]*AVOID_objects[i][1] < 0){    
    if(i==0 || fabs(AVOID_objects[i][0]) > fabs(AVOID_objects[i-1][0]) || fabs(AVOID_objects[i][1]) > fabs(AVOID_objects[i-1][1])){

      if(fabs(AVOID_objects[i][0]) > fabs(AVOID_objects[i][1])){
        heading_increment = AVOID_safety_angle;
      }
      else{
        heading_increment = -AVOID_safety_angle;
      }
    }
  }

  float new_heading = stateGetNedToBodyEulers_f()->psi + heading_increment;
  FLOAT_ANGLE_NORMALIZE(new_heading);
  nav_heading = ANGLE_BFP_OF_REAL(new_heading);

  }
}

bool safety_check_optical_flow(float *AVOID_safety_optical_flow, float x2, float y2){
  //returns true if it's not safe and there's the need to change heading
  //tunning parameters: AVOID_PERCENTAGE_THRESHOLD and AVOID_OF_angle

  float x1=stateGetPositionEnu_i()->x;
  float y1=stateGetPositionEnu_i()->y;
  //find next abssolute heading
  float next_absolute_heading=atan((y2-y1)/(x2-x1));
  if(x2<x1){
    next_absolute_heading+=M_PI;
  }
  FLOAT_ANGLE_NORMALIZE(next_absolute_heading);

  float relative_heading=next_absolute_heading-stateGetNedToBodyEulers_f()->psi;

  int i1=convert_heading_to_index(relative_heading-AVOID_OF_angle, OF_NUMBER_ELEMENTS);
  int i2=convert_heading_to_index(relative_heading+AVOID_OF_angle, OF_NUMBER_ELEMENTS);

  //array with the positions
  int indecis[OF_NUMBER_ELEMENTS];
  for(int i=0;i<OF_NUMBER_ELEMENTS;i++){
      indecis[i]=i;
  }
  quickSort(AVOID_safety_optical_flow,indecis,0,OF_NUMBER_ELEMENTS-1);

  bool change_heading=false;
  for(int i=0;i<OF_NUMBER_ELEMENTS;i++){  //runs through the entire sorted array
      if(indecis[i]<i && indecis[i]<i2){  //checks the values inside the original interval
        if(i>OF_NUMBER_ELEMENTS*(AVOID_PERCENTAGE_THRESHOLD/100)){
          change_heading=true;
      }
    }
  }
  /*
  if(change_heading){
    float new_heading=safe_heading(AVOID_safety_optical_flow)
  }*/

  return change_heading;
}

float convert_index_to_heading(int index, int N){
  float heading=2.0*index/(N-1)-1.0;  //normalize the index 
  heading=asin(heading); 
  return heading; //heading in radians
}

int convert_heading_to_index(float heading, int N){
  int index=(1+sin(heading))/(N-1)*2;
  return index; 
}

void quickSort(float array[],int indecis[], int low,int high){
  if (low < high)
  {
    /* pi is partitioning index, arr[pi] is now
       at right place */
      int pi = partition(array, indecis, low, high);

      quickSort(array, indecis, low, pi - 1);  // Before pi
      quickSort(array, indecis, pi + 1, high); // After pi
  }
  return;
}

int partition(float array[], int indecis[], int low, int high){
  int pivot = array[high];
  int i = (low - 1);

  for (int j = low; j < high; j++)
  {
    if (array[j] <= pivot)
    {
      i++;
      float aux=array[i];
      array[i]=array[j];
      array[j]=aux;
      int aux2=indecis[i];
      indecis[i]=indecis[j];
      indecis[j]=aux2;
    }
  }
  float aux = array[i+1];
  array[i+1]=array[high];
  array[high]=aux;
  int aux2 = indecis[i+1];
  indecis[i+1]=indecis[high];
  indecis[high]=aux2;
  
  return (i + 1);
}

float safe_heading(float array_of[]){
  //returns the safest heading according to OF values
  //saffest heading is the middle value of the i-th partition with span="angular_span"
  float angular_span=10* M_PI/180;
  int number_of_partitions=M_PI/angular_span;
  float partition_OF[number_of_partitions];

  float h1=-1*M_PI/2;     //h2 and h1 are the limits of the partition being processed
  float h2=h1+angular_span;  
  int i1,i2;      //i1 and i2 are the indices corresponding to h1 and h2 
  i2=convert_heading_to_index(h2, OF_NUMBER_ELEMENTS);

  for (int i = 0; i < number_of_partitions; i++){
    i1=convert_heading_to_index(h1, OF_NUMBER_ELEMENTS);

    //average of OF in the i-th span
    for (int j=i1; j <= i2; j++){   
      partition_OF[i]+=array_of[j];
    }
    partition_OF[i]=partition_OF[i]/(i2-i1+1);
    
    h2=h1;
    i2=i1;
    h1+=angular_span;
  }

  int indexis[number_of_partitions];
  for(int i=0;i<OF_NUMBER_ELEMENTS;i++){
    indexis[i]=i;
  }
  quickSort(partition_OF,indexis,0,number_of_partitions);
  float safest_heading=(indexis[0]+0.5)*angular_span; //partition with lowest OF average
  return safest_heading;
}


void unpack_object_list(){

     for (int i = 0; i < 100; i++) {

         AVOID_objects[i][0] = convert_index_to_heading(poles[i][0], 519);
         AVOID_objects[i][1] = convert_index_to_heading(poles[i][1], 519);
         AVOID_objects[i][2] = 0.0;
     }
}


void count_objects(){
  
    
    for (int i = 0; i < 10; i++) {
       if (poles[i][0] != 0 || poles[i][1] != 0){
            AVOID_number_of_objects = AVOID_number_of_objects+1;
        }
    }
}

//*****************************************  PATHING FUNCTIONS ********************************************
void circle(float current_time, float *TRAJECTORY_X, float *TRAJECTORY_Y, int r)
{
  double e = 1;
  //   if(AVOID_number_of_objects!=0)
  // {
  //   float r_reduced=0;
  //   float offset=asin(AVOID_d/(2*r))*180/M_PI; //offset in degrees
  //   if( AVOID_h1-offset<AVOID_safety_angle ||-1*AVOID_safety_angle<AVOID_h2-offset)
  //   {
  //     r_reduced=r*(AVOID_h2-offset)*M_PI/180;
  //   }
  //   r -= r_reduced;  
  // }  

  for(int i; i < AVOID_number_of_objects; i++){

    if(fabs(AVOID_objects[i][0]) < AVOID_safety_angle || fabs(AVOID_objects[i][1]) < AVOID_safety_angle || AVOID_objects[i][0]*AVOID_objects[i][1] < 0){    

      if(i==0 || fabs(AVOID_objects[i][0]) > fabs(AVOID_objects[i-1][0]) || fabs(AVOID_objects[i][1]) > fabs(AVOID_objects[i-1][1])){
          
          // float new_heading = stateGetNedToBodyEulers_f()->psi - 15*M_PI/180.0;
          // FLOAT_ANGLE_NORMALIZE(new_heading);
          // nav_heading = ANGLE_BFP_OF_REAL(new_heading);
          // moveWaypointForward(WP_TRAJECTORY, 5.0);
          
          printf("[%d]", r);
          r-=fabs(AVOID_objects[i][1])*400;
          printf("[%d] \n", r);
          dt = AVOID_slow_dt;
        }
    }
    else{
      dt = AVOID_normal_dt;
    }
  }

  *TRAJECTORY_X = r * cos(current_time);
  *TRAJECTORY_Y = e * r * sin(current_time);

return;
}

void square(float dt, float *TRAJECTORY_X, float *TRAJECTORY_Y, int r)
{
  int V = 700;

  for(int i; i < AVOID_number_of_objects; i++){

    if(fabs(AVOID_objects[i][0]) < AVOID_safety_angle || fabs(AVOID_objects[i][1]) < AVOID_safety_angle || AVOID_objects[i][0]*AVOID_objects[i][1] < 0){    

      if(i==0 || fabs(AVOID_objects[i][0]) > fabs(AVOID_objects[i-1][0]) || fabs(AVOID_objects[i][1]) > fabs(AVOID_objects[i-1][1])){
          printf("[%d]", r);
          //r-=fabs(AVOID_objects[i][1])*1000;
          r -= 450;
          printf("[%d %d %0.3f] \n", r, AVOID_number_of_objects, AVOID_objects[i][1]);
          dt = AVOID_slow_dt;
        }
    }
    else{
      dt = AVOID_normal_dt;
    }
  }

  if(square_mode==1){
    *TRAJECTORY_X = r;
    *TRAJECTORY_Y += dt*V;

    if (*TRAJECTORY_Y > r){
      square_mode=2;
    }
  }

  if(square_mode==2){
    *TRAJECTORY_X -= dt*V;
    *TRAJECTORY_Y=r;

    if (*TRAJECTORY_X<-r){
      square_mode=3;
    }
  }

  if(square_mode==3){
    *TRAJECTORY_X=-r;
    *TRAJECTORY_Y-=dt*V;

    if (*TRAJECTORY_Y<-r){
      square_mode=4;
    }
  }

  if(square_mode==4){
    *TRAJECTORY_X+=dt*V;
    *TRAJECTORY_Y=-r;

    if (*TRAJECTORY_X>r){
      square_mode=1;
    }
  }
    return;
}

void lace(float dt, float *TRAJECTORY_X, float *TRAJECTORY_Y, int r)
{
  int V = 700;

  for(int i; i < AVOID_number_of_objects; i++){

    if(fabs(AVOID_objects[i][0]) < AVOID_safety_angle || fabs(AVOID_objects[i][1]) < AVOID_safety_angle || AVOID_objects[i][0]*AVOID_objects[i][1] < 0){    

      if(i==0 || fabs(AVOID_objects[i][0]) > fabs(AVOID_objects[i-1][0]) || fabs(AVOID_objects[i][1]) > fabs(AVOID_objects[i-1][1])){
          
          if(lace_mode == 1 || lace_mode == 3){

            printf("[%d]", r);
            r-=fabs(AVOID_objects[i][1])*1000;
            printf("[%d %d %0.3f] \n", r, AVOID_number_of_objects, AVOID_objects[i][1]);
          }
          else{
            avoidance_straight_path();
          }
          dt = AVOID_slow_dt;
        }
    }
    // else{
    //   dt = AVOID_normal_dt;
    // }
  }


  if(lace_mode==1){
    *TRAJECTORY_X = r;
    *TRAJECTORY_Y -= dt*V;

    if (*TRAJECTORY_Y < -r){
      lace_mode=2;
    }
  }

  if(lace_mode==2){
    *TRAJECTORY_X -= dt*V*0.70710678;
    *TRAJECTORY_Y=-1 * *TRAJECTORY_X;

    if (*TRAJECTORY_X<-r){
      lace_mode=3;
    }
  }

  if(lace_mode==3){
    *TRAJECTORY_X=-r;
    *TRAJECTORY_Y-=dt*V;

    if (*TRAJECTORY_Y<-r){
      lace_mode=4;
    }
  }

  if(lace_mode==4){
    *TRAJECTORY_X+=dt*V*0.70710678;
    *TRAJECTORY_Y=*TRAJECTORY_X;

    if (*TRAJECTORY_X>r){
      lace_mode=1;
    }
  }
    return;
}

void lace_inverted(float dt, float *TRAJECTORY_X, float *TRAJECTORY_Y, int r)
{
  int V = 700;

  for(int i; i < AVOID_number_of_objects; i++){

    if(fabs(AVOID_objects[i][0]) < AVOID_safety_angle || fabs(AVOID_objects[i][1]) < AVOID_safety_angle || AVOID_objects[i][0]*AVOID_objects[i][1] < 0){    

      if(i==0 || fabs(AVOID_objects[i][0]) > fabs(AVOID_objects[i-1][0]) || fabs(AVOID_objects[i][1]) > fabs(AVOID_objects[i-1][1])){
          
          if(lace_mode == 1 || lace_mode == 3){

            printf("[%d]", r);
            r-=fabs(AVOID_objects[i][1])*1000;
            printf("[%d %d %0.3f] \n", r, AVOID_number_of_objects, AVOID_objects[i][1]);
          }
          else{
            avoidance_straight_path();
          }
          dt = AVOID_slow_dt;
        }
    }
    // else{
    //   dt = AVOID_normal_dt;
    // }
  }

  if(lace_mode==1){
    *TRAJECTORY_Y = r;
    *TRAJECTORY_X -= dt*V;

    if (*TRAJECTORY_X < -r){
      lace_mode=2;
    }
  }

  if(lace_mode==2){
    *TRAJECTORY_Y -= dt*V*0.70710678;
    *TRAJECTORY_X=-1 * *TRAJECTORY_Y;

    if (*TRAJECTORY_Y<-r){
      lace_mode=3;
    }
  }

  if(lace_mode==3){
    *TRAJECTORY_Y=-r;
    *TRAJECTORY_X-=dt*V;

    if (*TRAJECTORY_X<-r){
      lace_mode=4;
    }
  }

  if(lace_mode==4){
    *TRAJECTORY_Y+=dt*V*0.70710678;
    *TRAJECTORY_X=*TRAJECTORY_Y;

    if (*TRAJECTORY_Y>r){

      lace_mode=1;
    }
  }
    return;}





/*
 * Calculates coordinates of a distance of 'distanceMeters' forward w.r.t. current position and heading
 */
static uint8_t calculateForwards(struct EnuCoor_i *new_coor, float distanceMeters)
{
  float heading  = stateGetNedToBodyEulers_f()->psi;

  // Now determine where to place the waypoint you want to go to
  new_coor->x = stateGetPositionEnu_i()->x + POS_BFP_OF_REAL(sinf(heading) * (distanceMeters));
  new_coor->y = stateGetPositionEnu_i()->y + POS_BFP_OF_REAL(cosf(heading) * (distanceMeters));
  // VERBOSE_PRINT("Calculated %f m forward position. x: %f  y: %f based on pos(%f, %f) and heading(%f)\n", distanceMeters,	
                // POS_FLOAT_OF_BFP(new_coor->x), POS_FLOAT_OF_BFP(new_coor->y),
                // stateGetPositionEnu_f()->x, stateGetPositionEnu_f()->y, DegOfRad(heading));
  return false;
}

/*
 * Sets waypoint 'waypoint' to the coordinates of 'new_coor'
 */
uint8_t moveWaypoint(uint8_t waypoint, struct EnuCoor_i *new_coor)
{
  // VERBOSE_PRINT("Moving waypoint %d to x:%f y:%f\n", waypoint, POS_FLOAT_OF_BFP(new_coor->x),
                // POS_FLOAT_OF_BFP(new_coor->y));
  waypoint_set_xy_i(waypoint, new_coor->x, new_coor->y);
  return false;
}

/*
 * Calculates coordinates of distance forward and sets waypoint 'waypoint' to those coordinates
 */
uint8_t moveWaypointForward(uint8_t waypoint, float distanceMeters)
{
  struct EnuCoor_i new_coor;
  calculateForwards(&new_coor, distanceMeters);
  moveWaypoint(waypoint, &new_coor);
  return false;
}