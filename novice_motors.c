// Included libraries
#include <webots/robot.h> //obtain main library of webots
#include <webots/differential_wheels.h>   //obtain dif. wheels library
#include <stdlib.h> //for abs

// Global defines
#define LEFT 0        // Left side
#define RIGHT 1       // right side
#define INCR 10
#define TIME_STEP 128 // [ms] // time step of the simulation

int speed[2]={0,0};


int main() {
  wb_robot_init();

  wb_differential_wheels_enable_encoders(TIME_STEP);
  wb_differential_wheels_set_encoders(0,0);

  /* main loop */
  while(wb_robot_step(TIME_STEP) != -1) {
    // Either turn to the left or turn to the right
    if (abs(wb_differential_wheels_get_left_encoder()<1234)){
      speed[LEFT]+=INCR;
      speed[RIGHT]-=INCR;
    } else {
      speed[LEFT]=0;
      speed[RIGHT]=0;
      wb_differential_wheels_set_encoders(0,0);
    }

    wb_differential_wheels_set_speed(speed[LEFT],speed[RIGHT]);
  }

  wb_robot_cleanup();

  return 0;
}
