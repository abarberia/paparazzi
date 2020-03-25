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
#define NUMBER_OF_PARTITIONS 10

enum trajectory_mode_t {
  CIRCLE,
  SQUARE,
  LACE,
  INVERTED_LACE
};

enum safety_level_t {
  SAFE,
  THREAT,
  ESCAPE_IN_PROGRESS
};

enum trajectory_mode_t trajectory_mode = CIRCLE;
enum safety_level_t safety_level = SAFE;
int TRAJECTORY_L = 1800; //for a dt f 0.0011, razor thin margins
// int TRAJECTORY_L = 1550; //for a dt f 0.0004

//initializing variables
int TRAJECTORY_D = 0;
float TRAJECTORY_X=0;
float TRAJECTORY_Y=0;
float current_time = 0;
int square_mode = 1;
int lace_mode = 1;
int mode=1;
int AVOID_number_of_objects = 0;
float AVOID_h1,AVOID_h2;
float AVOID_d;
float AVOID_objects[100][3];

//********************* TUNNING PARAMETERS *********************
float TRAJECTORY_SWITCHING_TIME=20;
float AVOID_safety_angle = 25 * M_PI/180;
int AVOID_PERCENTAGE_THRESHOLD=30;
float AVOID_slow_dt = 0.0001;
float AVOID_normal_dt = 0.0004;
int AVOID_keep_slow_count = 0;
int AVOID_biggest_threat;
//**** FOR OF TUNNING
float AVOID_OF_angle = 5 * M_PI/180;
float OF_NEXT_HEADING_INFLUENCE = 100;

float dt=0.0007; // 0.6 m/s speed
struct EnuCoor_i AVOID_start_avoid_coord; 


void trajectory_init(void){}

void trajectory_periodic(void)
{

AVOID_number_of_objects = 0;
unpack_object_list();
count_objects();

current_time += dt;
int r = TRAJECTORY_L/2 - TRAJECTORY_D;   

switch (trajectory_mode){
  case CIRCLE:

    circle(current_time, &TRAJECTORY_X, &TRAJECTORY_Y, r);
    switch_path(SQUARE);

    break;
  case SQUARE:

    square(dt, &TRAJECTORY_X, &TRAJECTORY_Y, r);
    switch_path(LACE);

    break;
  case LACE:

    lace(dt, &TRAJECTORY_X, &TRAJECTORY_Y, r);
    switch_path(INVERTED_LACE);

      break;
  case INVERTED_LACE:

    lace_inverted(dt, &TRAJECTORY_X, &TRAJECTORY_Y, r);
    switch_path(CIRCLE);

      break;
  default:
    break;
  }

float x_rotated=TRAJECTORY_X*0.5+TRAJECTORY_Y*0.866025;
float y_rotated=-TRAJECTORY_X*0.866025+TRAJECTORY_Y*0.5;

setCoord(&AVOID_start_avoid_coord, x_rotated, y_rotated); 

 //after having the next way point --> double check with OF
 bool change_heading=safety_check_optical_flow(GLOBAL_OF_VECTOR, x_rotated, y_rotated);
 printf("change_heading: %d \n", change_heading);

if(change_heading){
  float next_heading=safe_heading(GLOBAL_OF_VECTOR);
  printf("\nCurrent heading: %f", (stateGetNedToBodyEulers_f()->psi)*180/M_PI);
  printf("\nheading correction: %f", (next_heading*180/M_PI));
  next_heading+=stateGetNedToBodyEulers_f()->psi;
  FLOAT_ANGLE_NORMALIZE(next_heading);
  ANGLE_BFP_OF_REAL(next_heading);
  printf("\nheading next: %f", next_heading*180/M_PI);

  float dx=OF_NEXT_HEADING_INFLUENCE * cosf(next_heading-M_PI/2);
  float dy=OF_NEXT_HEADING_INFLUENCE * sinf(next_heading-M_PI/2);
  int x_next = stateGetPositionEnu_i()->x + dx;  
  int y_next = stateGetPositionEnu_i()->y + dy;
  printf("\nThis position: x:%d ",stateGetPositionEnu_i()->x);
  printf(" y: %d",stateGetPositionEnu_i()->y);  
  printf("\n dx: %f",dx);
  printf("dy: %f",dy);
  printf("\nNext x: %d ",x_next);
  printf(" Next y: %d\n \n",y_next);

  //waypoint_set_xy_i(WP_STDBY,x_next,y_next);
}
 else{
    waypoint_set_xy_i(WP_STDBY,x_rotated,y_rotated);
    nav_set_heading_towards_waypoint(WP_STDBY);
 }
 
  //nav_set_heading_towards_waypoint(WP_STDBY);

if(safety_level!=ESCAPE_IN_PROGRESS){
  waypoint_set_xy_i(WP_STDBY,x_rotated,y_rotated);
  nav_set_heading_towards_waypoint(WP_STDBY);
  // waypoint_set_xy_i(WP_GOAL,x_rotated,y_rotated);
  // waypoint_set_xy_i(WP_TRAJECTORY,x_rotated,y_rotated);
}
else{
  printf("HOLD for iteration %c \n", AVOID_keep_slow_count);
}

      //moveWaypointForwardWithDirection(WP_STDBY, 100.0, -45*M_PI/180.0);
      // printf("[%d] \n", AVOID_number_of_objects);
      // printf("[%d] \n", r);
  // Deallocate
// float *GLOBAL_OF_VECTOR = NULL; 
}


//***************************************** AVOIDANCE STRATEGIES *****************************************

void determine_if_safe(){

  if(AVOID_keep_slow_count!=0){
    safety_level = ESCAPE_IN_PROGRESS;
    AVOID_keep_slow_count += 1;
  
    //if(isCoordInRadius(&AVOID_start_avoid_coord, 5.0) == true){
    if(AVOID_keep_slow_count>10){
        AVOID_keep_slow_count = 0;
        }
    return;
  }

  safety_level = SAFE;
  for(int i; i < AVOID_number_of_objects; i++){
    if(fabs(AVOID_objects[i][0]) < AVOID_safety_angle || fabs(AVOID_objects[i][1]) < AVOID_safety_angle || AVOID_objects[i][0]*AVOID_objects[i][1] < 0){
          
      if(i==0 || fabs(AVOID_objects[i][0]) > fabs(AVOID_objects[i-1][0]) || fabs(AVOID_objects[i][1]) > fabs(AVOID_objects[i-1][1])){
          AVOID_biggest_threat = i;
      }
    safety_level = THREAT;
  }
}
return;
}



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
float TRAJECTORY_SWITCHING_TIME=20;
  float new_heading = stateGetNedToBodyEulers_f()->psi + heading_increment;
  FLOAT_ANGLE_NORMALIZE(new_heading);
  nav_heading = ANGLE_BFP_OF_REAL(new_heading);

  }
}

bool safety_check_optical_flow(float *AVOID_safety_optical_flow, float x2, float y2){
  //returns true if it's not safe and there's the need to change heading
  //tunning parameters: AVOID_PERCENTAGE_THRESHOLD and AVOID_OF_angle

/*   float x1=stateGetPositionEnu_i()->x;
  float y1=stateGetPositionEnu_i()->y;
  //find next abssolute heading
  float next_absolute_heading=atan((y2-y1)/(x2-x1));
  if(x2<x1){
    next_absolute_heading+=M_PI;
  }
  FLOAT_ANGLE_NORMALIZE(next_absolute_heading);

  float relative_heading=next_absolute_heading-stateGetNedToBodyEulers_f()->psi;
  FLOAT_ANGLE_NORMALIZE(relative_heading); */
  
/*   printf("next_absolute_heading: %f \n", next_absolute_heading*180/M_PI);
  printf("current_absolute_heading: %f \n", (stateGetNedToBodyEulers_f()->psi)*180/M_PI);
  printf("relative_heading: %f \n", relative_heading*180/M_PI);
 */

  //int i1=convert_heading_to_index(relative_heading-AVOID_OF_angle, OF_NUMBER_ELEMENTS);
  //int i2=convert_heading_to_index(relative_heading+AVOID_OF_angle, OF_NUMBER_ELEMENTS);
  int i1=convert_heading_to_index(-AVOID_OF_angle, OF_NUMBER_ELEMENTS);
  int i2=convert_heading_to_index(AVOID_OF_angle, OF_NUMBER_ELEMENTS);
/*   printf("h1: %f, ", (-AVOID_OF_angle)*180/M_PI); printf(" h2: %f \n", AVOID_OF_angle*180/M_PI);
  printf("i1: %d \n", i1); printf(" i2: %d \n", i2); */

/*   if(-M_PI/4 > relative_heading-AVOID_OF_angle || M_PI/4 < relative_heading-AVOID_OF_angle || -M_PI/4 > relative_heading+AVOID_OF_angle || M_PI/4 < relative_heading+AVOID_OF_angle)
    return false; */

  //array with the positions
  int indecis[OF_NUMBER_ELEMENTS];
  for(int i=0;i<OF_NUMBER_ELEMENTS;i++){
      indecis[i]=i;
  }

/*
  quickSort(AVOID_safety_optical_flow,indecis,0,OF_NUMBER_ELEMENTS-1);
  bool change_heading=false;
  for(int i=0;i<OF_NUMBER_ELEMENTS;i++){  //runs through the entire sorted array
      if(i1<indecis[i] && indecis[i]<i2){  //checks the values inside the original interval
        if(i>(OF_NUMBER_ELEMENTS*AVOID_PERCENTAGE_THRESHOLD/100)){
          change_heading=true;
      }
    }
  }
  */

/*   printf("Print the OF array:\n");
  for(int i=0;i<OF_NUMBER_ELEMENTS;i++){
    printf("%d: %f \n",indecis[i],AVOID_safety_optical_flow[i]);
  } */

  bool change_heading=false;
  for (int i = i1; i <= i2; i++){
    if(AVOID_safety_optical_flow[i]>0.2){
          change_heading=true;
      }
  }

  return change_heading;
}


float safe_heading(float array_of[]){
  //returns the safest heading according to OF values
  //saffest heading is the middle value of the i-th partition with span="angular_span"
  float field_of_view=M_PI/2;
  float angular_span=field_of_view/NUMBER_OF_PARTITIONS;
  
  float partition_OF[NUMBER_OF_PARTITIONS];
  int indexis[NUMBER_OF_PARTITIONS];
  
  for (int i=0; i<NUMBER_OF_PARTITIONS; i++){   
      partition_OF[i]=0;
      indexis[i]=i;
  }

  float h1=-1*field_of_view/2;     //h2 and h1 are the limits of the partition being processed
  float h2=h1+angular_span;  
  int i1,i2;      //i1 and i2 are the indices corresponding to h1 and h2 

  for (int i = 0; i < NUMBER_OF_PARTITIONS; i++){
    i1=convert_heading_to_index(h1, OF_NUMBER_ELEMENTS);
    i2=convert_heading_to_index(h2, OF_NUMBER_ELEMENTS);

    //average of OF in the i-th span
    for (int j=i1; j <= i2; j++){   
      partition_OF[i]+=array_of[j];
    }
    partition_OF[i]=partition_OF[i]/(i2-i1+1);
    //printf("h1: %f  i1: %d \n h2: %f i2: %d \n \n", h1*180/M_PI, i1, h2*180/M_PI, i2);

    h1+=angular_span;
    h2=h1+angular_span;
  }
  
/*   printf("\n \n Array before quick sorting: \n");
  for(int i=0;i<NUMBER_OF_PARTITIONS;i++){
    printf("i: %d , value: %f \n", indexis[i], partition_OF[i]);
  } */
  
  quickSort(partition_OF,indexis,0,NUMBER_OF_PARTITIONS-1);
  
/*   printf("\n \n Array after quick sorting: \n");
  for(int i=0;i<NUMBER_OF_PARTITIONS;i++){
    printf("i: %d , value: %f \n", indexis[i], partition_OF[i]);
  } */

  float safest_heading = -1*field_of_view/2 + indexis[0] * angular_span + angular_span/2; //partition with lowest OF average
  printf("safest heading: %f", safest_heading);
  return safest_heading;
}


float convert_index_to_heading(int index, int N){
  float heading=2.0*index/(N-1)-1.0;  //normalize the index 
  heading=atan(heading); 
  return heading; //heading in radians
}

int convert_heading_to_index(float heading, int N){
  int index=(1+tan(heading))*(N-1)/2;
  return index; 
}

void quickSort(float array[], int indecis[], int first,int last){
   int i, j, pivot, temp2;
   float temp;

   if(first<last){
      pivot=first;
      i=first;
      j=last;

      while(i<j){
         while(array[i]<=array[pivot]&&i<last)
            i++;
         while(array[j]>array[pivot])
            j--;
         if(i<j){
            temp=array[i];
            array[i]=array[j];
            array[j]=temp;
            temp2=indecis[i];
            indecis[i]=indecis[j];
            indecis[j]=temp2;            
         }
      }

      temp=array[pivot];
      array[pivot]=array[j];
      array[j]=temp;
      temp2=indecis[i];
      indecis[i]=indecis[j];
      indecis[j]=temp2;
      quickSort(array, indecis, first,j-1);
      quickSort(array, indecis ,j+1,last);
   }
}


void unpack_object_list(){

     for (int i = 0; i < 100; i++) {

         AVOID_objects[i][0] = convert_index_to_heading(final_objs[i][0], 519);
         AVOID_objects[i][1] = convert_index_to_heading(final_objs[i][1], 519);
         AVOID_objects[i][2] = 0.0;
     }
}


void count_objects(){
  
    
    for (int i = 0; i < 10; i++) {
       if (final_objs[i][0] != 0 || final_objs[i][1] != 0){
            AVOID_number_of_objects = AVOID_number_of_objects+1;
        }
    }
}

//*****************************************  PATHING FUNCTIONS ********************************************
void circle(float current_time, float *TRAJECTORY_X, float *TRAJECTORY_Y, int r)
{
  double e = 1;

  
  //   float offset=asin(AVOID_d/(2*r))*180/M_PI; //offset in degrees
  //     r_reduced=r*(AVOID_h2-offset)*M_PI/180;

  determine_if_safe();

  if(safety_level==THREAT){
    dt = AVOID_slow_dt;
    r-=fabs(AVOID_objects[AVOID_biggest_threat][1])*400;
    //moveWaypointForwaifrdWithDirection(WP_STDBY, 100.0, -45*M_PI/180.0);
    printf("[%d %d] \n", AVOID_biggest_threat, r);
    AVOID_keep_slow_count += 1;
  }
  else if(safety_level==SAFE){
    dt = AVOID_normal_dt;
  }
  else if(safety_level==ESCAPE_IN_PROGRESS){
    dt = AVOID_slow_dt;
  }

  *TRAJECTORY_X = r * cos(current_time);
  *TRAJECTORY_Y = e * r * sin(current_time);


return;
}

void square(float dt, float *TRAJECTORY_X, float *TRAJECTORY_Y, int r)
{
  int V = 700;

  if(safety_level==THREAT){
      dt = AVOID_slow_dt;
      r-=fabs(AVOID_objects[AVOID_biggest_threat][1])*400;
      //moveWaypointForwardWithDirection(WP_STDBY, 100.0, -45*M_PI/180.0);
      // printf("[%d] \n", AVOID_number_of_objects);
      // printf("[%d] \n", r);
      AVOID_keep_slow_count += 0;
      if(AVOID_keep_slow_count==2){
        AVOID_keep_slow_count=0;
        }
    }
    if(safety_level==SAFE){
    dt = AVOID_normal_dt;
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
            //r-=fabs(AVOID_objects[i][1])*1000;
            r -= 200;
            //printf("[%d %d %0.3f] \n", r, AVOID_number_of_objects, AVOID_objects[i][1]);
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
            //printf("[%d %d %0.3f] \n", r, AVOID_number_of_objects, AVOID_objects[i][1]);
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


void switch_path(enum trajectory_mode_t mode){

    if (current_time > TRAJECTORY_SWITCHING_TIME){ //move to another mode
        current_time=0;
        TRAJECTORY_Y=0;
        TRAJECTORY_X=0;  
        square_mode=1; 
        lace_mode=1;
        trajectory_mode = mode;
      }

}


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
 * Calculates coordinates of a distance of 'distanceMeters' forward w.r.t. current position and heading
 */
static uint8_t calculateForwardsWithDirection(struct EnuCoor_i *new_coor, float distanceMeters, float direction)
{
  float heading  = stateGetNedToBodyEulers_f()->psi + direction; 

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

/*
 * Calculates coordinates of distance forward and sets waypoint 'waypoint' to those coordinates
 */
uint8_t moveWaypointForwardWithDirection(uint8_t waypoint, float distanceMeters, float direction)
{
  struct EnuCoor_i new_coor;
  calculateForwardsWithDirection(&new_coor, distanceMeters, direction);
  moveWaypoint(waypoint, &new_coor);
  return false;
}

void setCoord(struct EnuCoor_i *coord, float x, float y){
  coord->x = stateGetPositionEnu_i()->x;
  coord->y = stateGetPositionEnu_i()->y;
}

bool isCoordInRadius(struct EnuCoor_i *coord, float radius){

  float dist = sqrt(pow(coord->x - stateGetPositionEnu_i()->x,2) + pow(coord->y - stateGetPositionEnu_i()->y,2));
  if(dist > radius){
    return true;
  }
  else{
  return false;
  }
}