//neural_net emitts sensor values which are received by genetic_algorithm_supervisor
//neural_net_supervisor emitts fitness values which are received by neural_net

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <assert.h>

//Include the Webots libraries
#include <webots/robot.h>
#include <webots/differential_wheels.h>
#include <webots/distance_sensor.h>
#include <webots/emitter.h>
#include <webots/receiver.h>

static int time_step;

WbDeviceTag emitter;
WbDeviceTag receiver;              // for receiving genes from Supervisor

//Defining the Distance Sensors
#define NB_Dist_Sens  8
static WbDeviceTag ps[NB_Dist_Sens];
static double ps_values[NB_Dist_Sens];
static double ps_values_full[NB_Dist_Sens];
static const char *ps_labels[NB_Dist_Sens] = { "ps0", "ps1", "ps2", "ps3","ps4", "ps5", "ps6", "ps7"};

//Defining Floor sensors
#define NB_Floor_Sens 3
static WbDeviceTag gs[NB_Floor_Sens];
static double gs_values[NB_Floor_Sens] = { 0.0, 0.0, 0.0 };
static const char *gs_labels[NB_Floor_Sens] = {"gs0", "gs1", "gs2"};

#define NB_WHEELS 2
#define MAX_SPEED 1000.0
#define GENOTYPE_SIZE (NB_Dist_Sens * NB_WHEELS)
static double speed[NB_WHEELS];
static double matrix[NB_Dist_Sens][NB_WHEELS]; // sensor to wheels multiplication matrix each sensor has a weight for each wheel
static double offset[2] = {0.75*MAX_SPEED, 0.75*MAX_SPEED};

// check if a new set of genes was sent by the Supervisor
// in this case start using these new genes immediately
void check_for_new_genes() {
  
  if (wb_receiver_get_queue_length(receiver) > 0) {
  
    // check that the number of genes received match what is expected
    assert(wb_receiver_get_data_size(receiver) == GENOTYPE_SIZE * sizeof(double));
    
    // copy new genes into weight matrix
    memcpy(matrix, wb_receiver_get_data(receiver), GENOTYPE_SIZE * sizeof(double));
    
    // prepare for receiving next packet
    wb_receiver_next_packet(receiver);
  }
 }

//get simulation time step, returned in milliseconds
int get_time_step()
{
  time_step = -1;
  if (time_step == -1)
  {
    time_step = (int) wb_robot_get_basic_time_step();
  }
  return time_step;
}
static void initialise_time_step() 
{
  if (wb_robot_step(get_time_step()) == -1) {
    wb_robot_cleanup();
    exit(EXIT_SUCCESS);
  }
}
//Initialise the sensors at the beginning of the program
static void reset_sensors()
{
  //initalise emitter and receiver
    receiver = wb_robot_get_device("receiver");
    wb_receiver_enable(receiver, get_time_step());
    emitter = wb_robot_get_device("emitter");
 
  int it;
  //distance sensors initalise
   for(it=0;it<NB_Dist_Sens;it++)
    {
       ps[it]= wb_robot_get_device(ps_labels[it]);
       wb_distance_sensor_enable(ps[it], get_time_step());
    }
    
    //floor sensors
    for(it=0; it<NB_Floor_Sens; it++)
    {
      gs[it] = wb_robot_get_device(gs_labels[it]);
      wb_distance_sensor_enable(gs[it], get_time_step());
    }
    
     initialise_time_step();
}

//Get the input values from the sensors
static void run_sensors()
{
  int i;
  //distance sensors
  for (i=0; i<NB_Dist_Sens; i++) 
  {
  
    ps_values[i] = wb_distance_sensor_get_value(ps[i]);
    ps_values_full[i] =  ps_values[i];
    ps_values[i] /= 4096; //divison scales the sensor data to a number between 0 (nothing) and 1 (wall)
  }
  
  //floor sensors
  for (i=0; i<NB_Floor_Sens; i++)
  {
    gs_values[i] = wb_distance_sensor_get_value(gs[i]);
  }
  
}
//Set the speed of motors to stationary
static void reset_motor_speed()
{
    speed[0] = 0;
    speed[1] = 0;
}

static void set_motor_speed()
{
  wb_differential_wheels_set_speed(speed[0], speed[1]);
}

static void emit_data()
{
  int a, b;
  
  static double all_sensors[11];
  
  for(a = 0; a < 8; a++)
  {
    all_sensors[a] = ps_values[a];
  }
  
  for(b = 0; b < 3; b++)
  {
    all_sensors[b] = gs_values[b];  
  }
  //send sensor data to supervisor
   wb_emitter_send(emitter, all_sensors, 11*sizeof(double));
}


  // compute actuation using Braitenberg's algorithm:
  // The speed of each wheel is computed by summing the value
  // of each sensor multiplied by the corresponding weight of the matrix
static void neural_network()
{
  int a, b;
  //for left wheel and right wheel, set speed to 0
 // for(a=0; a<2; a++)
 // {
  // speed[a] = 0;
  //} 
  
  //for each sensor set speed to distance multiplied weight
  for(a=0; a<2; a++)
  {
      for(b=0; b<NB_Dist_Sens; b++)
      {
         speed[a] += matrix[b][a] * ps_values[b];
         speed[a] = offset[a] + speed[a]*MAX_SPEED;
      }
  
  //to check that speed doesn't exceed max speed
      if(speed[a] > MAX_SPEED)
      {
         speed[a] = MAX_SPEED;
      } 
      else if (speed[a] < -MAX_SPEED)
         speed[a] = -MAX_SPEED;
  }
}

static void run_neural_network()
{
  check_for_new_genes();
  reset_motor_speed();
  run_sensors();
  emit_data();
  neural_network();
  set_motor_speed();
}

int main(int argc, const char *argv[]) 
{
    wb_robot_init();
    reset_sensors();
    time_step = wb_robot_get_basic_time_step();
    // initialize matrix to zero, hence the robot wheels will initially be stopped
    memset(matrix, 0.0, sizeof(matrix));
    while (wb_robot_step(time_step) != -1)
    {
     run_neural_network();
    }
    
    wb_robot_cleanup();
    return 0;
}
