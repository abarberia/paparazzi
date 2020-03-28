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
float TRAJECTORY_SWITCHING_TIME=19;
float AVOID_safety_angle = 15 * M_PI/180;
//int AVOID_PERCENTAGE_THRESHOLD=30;
float AVOID_slow_dt = 0.00008;
float AVOID_normal_dt = 0.0006;
int AVOID_keep_slow_count = 0;
int AVOID_biggest_threat;
float dt=0.0006; // 0.6 m/s speed
struct EnuCoor_i AVOID_start_avoid_coord; 
bool safe_mode_previous=false;
int last_iteration_safe_heading=0;

//********************* TUNNNG PARAMETERS *********************
//**** FOR Color filter TUNNING
float AVOID_dist_threat = 3.5; // typically between 2 and 3.5. 
int AVOID_keep_escape_count = 40;   // typically between 0 and 90. This is to avoid oscillations in the escape route. The higher, the fewer oscillations
//**** FOR Optical Flow TUNNING
float AVOID_OF_angle = 3.5 * M_PI/180;  // angle for which we look at the Optical flow
float OF_NEXT_HEADING_INFLUENCE = 0.25;  // Gain of escpae route from the optical flow-based avoidance
float OPTICAL_FLOW_THRESHOLD=0.6;  // Optical flow above which it's dangerous to move forward
//****** To Count the Distance
float distance_travelled;
float last_x;
float last_y;

void trajectory_init(void){
  distance_travelled=0;
  last_x=stateGetPositionEnu_f()->x;
  last_y=stateGetPositionEnu_f()->y;
}

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
    //printf("CIRCLE\n");
    switch_path(SQUARE);

    break;
  case SQUARE:

    square(dt, &TRAJECTORY_X, &TRAJECTORY_Y, r);
    //printf("SQUARE\n");
    switch_path(CIRCLE);

    break;
  case LACE:

    lace(dt, &TRAJECTORY_X, &TRAJECTORY_Y, r);
    switch_path(INVERTED_LACE);

      break;
  case INVERTED_LACE:

    lace_inverted(dt, &TRAJECTORY_X, &TRAJECTORY_Y, r);
    switch_path(LACE);

      break;
  default:
    break;
  }

float x_rotated=TRAJECTORY_X*0.5+TRAJECTORY_Y*0.866025;
float y_rotated=-TRAJECTORY_X*0.866025+TRAJECTORY_Y*0.5;


if(safety_level!=ESCAPE_IN_PROGRESS){
  waypoint_set_xy_i(WP_GOAL,x_rotated,y_rotated);
  //for the begining and when we change the mode
  if(current_time<19){
      bool change_heading = safety_check_optical_flow(GLOBAL_OF_VECTOR, x_rotated, y_rotated);
    if(change_heading){
      moveWaypointForwardWithDirection(WP_GOAL,OF_NEXT_HEADING_INFLUENCE,safe_heading(GLOBAL_OF_VECTOR));
      safe_mode_previous=true;
    }
    else{
      safe_mode_previous=false;
    }
  }
}
else if(trajectory_mode!=SQUARE){
  bool change_heading = safety_check_optical_flow(GLOBAL_OF_VECTOR, x_rotated, y_rotated);
  if(change_heading){
    moveWaypointForwardWithDirection(WP_GOAL,OF_NEXT_HEADING_INFLUENCE,safe_heading(GLOBAL_OF_VECTOR));
    safe_mode_previous=true;
  }
  else{
    safe_mode_previous=false;
  }
}
nav_set_heading_towards_waypoint(WP_GOAL);
distance_travelled+=distance_travelled_last_iteration();
printf("\n Distance tavelled= %f \n", distance_travelled);
// Deallocate
// float *GLOBAL_OF_VECTOR = NULL; 
}

//********************************************* COUNT DISTANCE *******************************************

//Returns the distance travelled in the last iteration
float distance_travelled_last_iteration(){
  float dx=stateGetPositionEnu_f()->x - last_x;
  float dy=stateGetPositionEnu_f()->y - last_y;
  //update last position
  last_x=stateGetPositionEnu_f()->x;
  last_y=stateGetPositionEnu_f()->y;
  //add new distance travelleds
  float r=sqrt(dx*dx+dy*dy);
  return r;
}

//***************************************** AVOIDANCE STRATEGIES *****************************************

//determines the safety level 
void determine_if_safe(){
  if(AVOID_keep_slow_count!=0){
    safety_level = ESCAPE_IN_PROGRESS;
    AVOID_keep_slow_count += 1;
  
   //if(isCoordInRadius(&AVOID_start_avoid_coord, 2.4) == true){
   if(AVOID_keep_slow_count > AVOID_keep_escape_count){
        AVOID_keep_slow_count = 0;
        }
    return;
  }

  safety_level = SAFE;
  for(int i; i < AVOID_number_of_objects; i++){
    if((fabs(AVOID_objects[i][0]) < AVOID_safety_angle || fabs(AVOID_objects[i][1]) < AVOID_safety_angle || AVOID_objects[i][0]*AVOID_objects[i][1] < 0) &&  final_objs[i][2]<AVOID_dist_threat){
      
      if(i==0 || fabs(AVOID_objects[i][0]) > fabs(AVOID_objects[i-1][0]) || fabs(AVOID_objects[i][1]) > fabs(AVOID_objects[i-1][1])){
          AVOID_biggest_threat = i;
          //printf("[%d] \n", AVOID_objects[AVOID_biggest_threat][2]);
      }
    safety_level = THREAT;
    setCoord(&AVOID_start_avoid_coord, stateGetPositionEnu_i()->x, stateGetPositionEnu_i()->y); 
  }
}
return;
}

//returns true if it's not safe and there's the need to change heading
//tunning parameters: AVOID_PERCENTAGE_THRESHOLD and AVOID_OF_angle
bool safety_check_optical_flow(float *AVOID_safety_optical_flow, float x2, float y2){
  int i1=convert_heading_to_index(-AVOID_OF_angle, OF_NUMBER_ELEMENTS);
  int i2=convert_heading_to_index(AVOID_OF_angle, OF_NUMBER_ELEMENTS);

  //printf("\n Optical flow array:\n");
  //array with the positions
  int indecis[OF_NUMBER_ELEMENTS];
  for(int i=0;i<OF_NUMBER_ELEMENTS;i++){
      indecis[i]=i;
  }

  bool change_heading=false;
  for (int i = i1; i <= i2; i++){
    //printf("%d: %f \n", i, AVOID_safety_optical_flow[i]);
    if(AVOID_safety_optical_flow[i]>OPTICAL_FLOW_THRESHOLD){
          change_heading=true;
      }
  }

  return change_heading;
}

//Function: Returns the safest heading according to OF values
// - divides the field of view in NUMBER_OF_PARTITIONS partitions
// - Finds the partition with smallest OF (partition i)
// - saffest heading is the middle value of the i-th partition with span="angular_span"
float safe_heading(float array_of[]){
  printf("\nOF activated\n");
  float field_of_view=M_PI/2;
  float angular_span=field_of_view/NUMBER_OF_PARTITIONS;
  
  float partition_OF[NUMBER_OF_PARTITIONS];
  int indexis[NUMBER_OF_PARTITIONS];
  
  for (int i=0; i<NUMBER_OF_PARTITIONS; i++){   
      partition_OF[i]=0;
      indexis[i]=i;
  }

  float h1=-1*field_of_view/2;     //h2 and h1 are the limits 
  float h2=h1+angular_span;        //of the partition being processed
  int i1,i2;      //i1 and i2 are the indices corresponding to h1 and h2 

  //calculating OF in each partition
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

  // float last_iteration_partition_OF=partition_OF[last_iteration_safe_heading];

     if(safe_mode_previous){
    if(partition_OF[last_iteration_safe_heading]<0.5){
      float safest_heading = -1*field_of_view/2 + last_iteration_safe_heading * angular_span + angular_span/2;
      printf("SAME HEADING: %f\n", safest_heading);
      return safest_heading;
    }
  }

  for (int i = 0; i < NUMBER_OF_PARTITIONS; i++){
    if(partition_OF[i]<0.5){
      float safest_heading = -1*field_of_view/2 + i * angular_span + angular_span/2;
      printf("Safest heading: %f \n", safest_heading);
      last_iteration_safe_heading=i;
      return safest_heading;
    }
  }
  
  //Finds the partition with smallest OF
  quickSort(partition_OF,indexis,0,NUMBER_OF_PARTITIONS-1);

/*   if(safe_mode_previous){
    if(last_iteration_partition_OF<0.2+partition_OF[0]){
      float safest_heading = -1*field_of_view/2 + last_iteration_safe_heading * angular_span + angular_span/2;
      printf("SAME HEADING: %f\n", safest_heading);
      return safest_heading;
    }
  } */

/*   printf("\n Partitions after sorting: \n");
  for (int i = 0; i < NUMBER_OF_PARTITIONS; i++){
    printf("%d: %f\n", indexis[i], partition_OF[i]);
  } */

  float safest_heading = -1*field_of_view/2 + indexis[0] * angular_span + angular_span/2; //partition with lowest OF average
  last_iteration_safe_heading=indexis[0];
  return safest_heading;
}

//Converts the index of OF array to the corresponding heading (due to distortion)
float convert_index_to_heading(int index, int N){
  float heading=2.0*index/(N-1)-1.0;  //normalize the index 
  heading=atan(heading); 
  return heading; //heading in radians
}

//Converts the heading to the index of OF array (due to distortion)
int convert_heading_to_index(float heading, int N){
  int index=(1+tan(heading))*(N-1)/2;
  return index; 
}

//Sorting function using quick sort algorithm
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
      temp2=indecis[pivot];
      indecis[pivot]=indecis[j];
      indecis[j]=temp2;
      quickSort(array, indecis, first,j-1);
      quickSort(array, indecis ,j+1,last);
   }
}


void unpack_object_list(){
     for (int i = 0; i < 100; i++) {
         AVOID_objects[i][0] = convert_index_to_heading(final_objs[i][0], 519);
         AVOID_objects[i][1] = convert_index_to_heading(final_objs[i][1], 519);
         AVOID_objects[i][2] = final_objs[i][2];
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

  determine_if_safe();

  if(safety_level==THREAT){
    dt = AVOID_slow_dt;
    r-=fabs(AVOID_objects[AVOID_biggest_threat][1])*450;
    //printf("[%d %d] \n", final_objs[AVOID_biggest_threat][2], r);
    AVOID_keep_slow_count += 1;
  }
  else if(safety_level==SAFE){
    dt = AVOID_normal_dt;
  }
  else if(safety_level==ESCAPE_IN_PROGRESS){
    dt = AVOID_normal_dt;
  }

  *TRAJECTORY_X = r * cos(current_time);
  *TRAJECTORY_Y = e * r * sin(current_time);


return;
}

void square(float dt, float *TRAJECTORY_X, float *TRAJECTORY_Y, int r)
{
  int V = 700;

  //determine_if_safe();

  if(safety_level==THREAT){
    dt = AVOID_slow_dt;
    r-=fabs(AVOID_objects[AVOID_biggest_threat][1])*600;
    //moveWaypointForwaifrdWithDirection(WP_STDBY, 100.0, -45*M_PI/180.0);
    //printf("[%d %d] \n", final_objs[AVOID_biggest_threat][2], r);
    AVOID_keep_slow_count += 1;
  }
  else if(safety_level==SAFE){
    dt = AVOID_normal_dt;
  }
  else if(safety_level==ESCAPE_IN_PROGRESS){
    dt = AVOID_slow_dt;
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
  //determine_if_safe();

  if(lace_mode==1){
    if(safety_level==THREAT){
      dt = AVOID_slow_dt;
      r-=fabs(AVOID_objects[AVOID_biggest_threat][1])*600;
      AVOID_keep_slow_count += 1;
    }
    else if(safety_level==SAFE){
      dt = AVOID_normal_dt;
    }
    else if(safety_level==ESCAPE_IN_PROGRESS){
      dt = AVOID_slow_dt;
    }
  *TRAJECTORY_X = r;
  *TRAJECTORY_Y -= dt*V;
  if (*TRAJECTORY_Y < -r){
    lace_mode=2;
    }
  }

  if(lace_mode==3){
    if(safety_level==THREAT){
      dt = AVOID_slow_dt;
      r-=fabs(AVOID_objects[AVOID_biggest_threat][1])*600;
      AVOID_keep_slow_count += 1;
    }
    else if(safety_level==SAFE){
      dt = AVOID_normal_dt;
    }
    else if(safety_level==ESCAPE_IN_PROGRESS){
      dt = AVOID_slow_dt;
    }
  *TRAJECTORY_X = -r;
  *TRAJECTORY_Y -= dt*V;
  if (*TRAJECTORY_Y < -r){
    lace_mode=4;
    }
  }

  if(lace_mode==2){
    if(safety_level==THREAT){
    dt = AVOID_slow_dt;
    calculatePointForwardsWithDirection(&TRAJECTORY_X, &TRAJECTORY_Y, 30.0, 45*M_PI/180.0);
    AVOID_keep_slow_count += 1;
  }
  else if(safety_level==SAFE){
    dt = AVOID_normal_dt;
    *TRAJECTORY_X+=dt*V*0.70710678;
    *TRAJECTORY_Y=*TRAJECTORY_X;
      if (*TRAJECTORY_X<-r){
      lace_mode=3;
    }
  }
  else if(safety_level==ESCAPE_IN_PROGRESS){
    dt = AVOID_slow_dt;
  }
  }

  if(lace_mode==4){
    if(safety_level==THREAT){
    dt = AVOID_slow_dt;
    calculatePointForwardsWithDirection(&TRAJECTORY_X, &TRAJECTORY_Y, 30.0, 45*M_PI/180.0);
    AVOID_keep_slow_count += 1;
  }
  else if(safety_level==SAFE){
    dt = AVOID_normal_dt;
     *TRAJECTORY_X+=dt*V*0.70710678;
    *TRAJECTORY_Y=*TRAJECTORY_X;
      if (*TRAJECTORY_X>r){
      lace_mode=1;
    }
  }
  else if(safety_level==ESCAPE_IN_PROGRESS){
    dt = AVOID_slow_dt;
  }
  }
  return;
}

void lace_inverted(float dt, float *TRAJECTORY_X, float *TRAJECTORY_Y, int r)
{
 int V = 700;
  determine_if_safe();

  if(lace_mode==1){
    if(safety_level==THREAT){
      dt = AVOID_slow_dt;
      r-=fabs(AVOID_objects[AVOID_biggest_threat][1])*600;
      AVOID_keep_slow_count += 1;
    }
    else if(safety_level==SAFE){
      dt = AVOID_normal_dt;
    }
    else if(safety_level==ESCAPE_IN_PROGRESS){
      dt = AVOID_slow_dt;
    }
  *TRAJECTORY_Y = r;
  *TRAJECTORY_X -= dt*V;
  if (*TRAJECTORY_X < -r){
    lace_mode=2;
    }
  }

  if(lace_mode==3){
    if(safety_level==THREAT){
      dt = AVOID_slow_dt;
      r-=fabs(AVOID_objects[AVOID_biggest_threat][1])*600;
      AVOID_keep_slow_count += 1;
    }
    else if(safety_level==SAFE){
      dt = AVOID_normal_dt;
    }
    else if(safety_level==ESCAPE_IN_PROGRESS){
      dt = AVOID_slow_dt;
    }
  *TRAJECTORY_Y=-r;
  *TRAJECTORY_X-=dt*V;
  if (*TRAJECTORY_X<-r){
    lace_mode=4;
    }
  }

  if(lace_mode==2){
    if(safety_level==THREAT){
    dt = AVOID_slow_dt;
    calculatePointForwardsWithDirection(&TRAJECTORY_X, &TRAJECTORY_Y, 30.0, 45*M_PI/180.0);
    AVOID_keep_slow_count += 1;
  }
  else if(safety_level==SAFE){
    dt = AVOID_normal_dt;
    *TRAJECTORY_Y -= dt*V*0.70710678;
    *TRAJECTORY_X=-1 * *TRAJECTORY_Y;
      if (*TRAJECTORY_Y<-r){
      lace_mode=3;
    }
  }
  else if(safety_level==ESCAPE_IN_PROGRESS){
    dt = AVOID_slow_dt;
  }
  }

  if(lace_mode==4){
    if(safety_level==THREAT){
    dt = AVOID_slow_dt;
    calculatePointForwardsWithDirection(&TRAJECTORY_X, &TRAJECTORY_Y, 30.0, 45*M_PI/180.0);
    AVOID_keep_slow_count += 1;
  }
  else if(safety_level==SAFE){
    dt = AVOID_normal_dt;
      *TRAJECTORY_Y+=dt*V*0.70710678;
    *TRAJECTORY_X=*TRAJECTORY_Y;
      if (*TRAJECTORY_Y>r){
      lace_mode=1;
    }
  }
  else if(safety_level==ESCAPE_IN_PROGRESS){
    dt = AVOID_slow_dt;
  }
  }
  return;
}

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

// ************************************* NAVIGATION USEFUL FUNCTIONS ********************************************

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
void calculatePointForwardsWithDirection(float *TRAJECTORY_X, float *TRAJECTORY_Y, float distanceMeters, float direction)
{
  float heading  = stateGetNedToBodyEulers_f()->psi + direction; 

  // Now determine where to place the waypoint you want to go to
  *TRAJECTORY_X = stateGetPositionEnu_i()->x + sinf(heading) * distanceMeters;
  *TRAJECTORY_Y = stateGetPositionEnu_i()->y + cosf(heading) * distanceMeters;
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
  printf("%f \n", dist);

  if(dist > radius){
    return true;
  }
  else{
  return false;
  }
}