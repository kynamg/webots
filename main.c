#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include <advanced_genetic_algorithm_supervisor.h>
#include <genotype.h>
#include <population.h>
//Include the Webots libraries
#include <webots/robot.h>
#include <webots/differential_wheels.h>
#include <webots/distance_sensor.h>
#include <webots/supervisor.h>
#include <webots/emitter.h>
#include <webots/receiver.h>
#include <webots/display.h>

#define THRESHOLD_VALUE 100
#define SIMULATION 0
//Defining the IR Sensors
#define NB_Dist_Sens  8
#define PS_Right_10   0
#define PS_Right_45   1
#define PS_Right_90   2
#define PS_Right_Rear 3
#define PS_Left_Rear  4
#define PS_Left_90    5
#define PS_Left_45    6
#define PS_Left_10    7
WbDeviceTag ps[NB_Dist_Sens];
int ps_value[NB_Dist_Sens]={0,0,0,0,0,0,0,0};
int ps_offset_sim[NB_Dist_Sens] = {35,35,35,35,35,35,35,35};
int ps_offset_real[NB_Dist_Sens] = {375,158,423,682,447,594,142,360}; // to be modified according to your robot
char * ps_text[]={"one","two","three","eleven","ten","nine"};

//Defining the motors
#define LEFT 1
#define RIGHT 2
float LM=0;
float RM=0;

//Defining the fear factor in the neural network and its weights
float Fe = 0;
float weight_Fe_left = 0;
float weight_Fe_right = 0;

static int time_step;
static WbDeviceTag receiver;
static double floor_sensor_val[3] = {300, 300, 300}; //values found in lookup table of groundsensors.proto
static double total_dist, max_dist, dist_from_init;
static double previous_x, previous_y;
//fields for the fitness function
static int off_maze, check_val, danger_zone;

//given
static const int POPULATION_SIZE = 50;
static const int NUM_GENERATIONS = 25;
static const char *FILE_NAME = "most_fit.txt";

// must match the values in the advanced_genetic_algorithm.c code
static const int NUM_SENSORS = 8;
static const int NUM_WHEELS  = 2;
#define GENOTYPE_SIZE (NUM_SENSORS * NUM_WHEELS)

// index access
enum { X, Y, Z };

static WbDeviceTag emitter;   // to send genes to robot
static WbDeviceTag display;   // to display the fitness evolution
static int display_width, display_height;

// the GA population
static Population population;

// for reading or setting the robot's position and orientation
static WbFieldRef robot_translation;
static WbFieldRef robot_rotation;
static double robot_trans0[3];  // a translation needs 3 doubles
static double robot_rot0[4];    // a rotation needs 4 doubles

// for reading or setting the load's position
static WbFieldRef load_translation;
//static double load_trans0[3];

//Initialise the sensors at the beginning of the program
static void reset_sensors(void)
{
    int it;
    char textPS[]="ps0";
    for(it=0;it<NB_Dist_Sens;it++)
    {
        ps[it]= wb_robot_get_device(textPS);
        textPS[2]++;
    }
    for(it=0;it<NB_Dist_Sens;it++)
    {
        wb_distance_sensor_enable(ps[it],32);
    }
}

//Get the distances values from the IR sensors to detect obstacles
static void run_sensors(void)
{
   int i;
   int ps_offset[NB_Dist_Sens]={0,0,0,0,0,0,0,0};
   int mode = wb_robot_get_mode();
   if(mode==SIMULATION)
   {
      for(i=0;i<NB_Dist_Sens;i++)
      {
         ps_offset[i]=ps_offset_sim[i];
      }
   }
   else
   {
      for(i=0;i<NB_Dist_Sens;i++)
      {
         ps_offset[i]=ps_offset_real[i];
      }
   }
   for(i=0;i<NB_Dist_Sens;i++)
   {
      ps_value[i]= (int) wb_distance_sensor_get_value(ps[i]);
   }

   for(i=0;i<NB_Dist_Sens;i++)
   {
      if(ps_value[i]-ps_offset[i]>THRESHOLD_VALUE)
      {
         printf("An obstacle is detected at %s o'clock",ps_text[i]);
      }
   }
}

//Set the speed of motors
static void motor_set_speed(void)
{
    int speed[2]={0,0};
    speed[LEFT]=LM;
    speed[RIGHT]=RM;
    wb_differential_wheels_set_speed(speed[LEFT],speed[RIGHT]);
}

//Run the whole neural network
static void run_neural_network(void)
{
   run_sensors();
   Get_Fe();
   neural_network();
   motor_set_speed();
   Fe=0;
}

//Get the fear factor and which wheel it needs to apply to
void Get_Fe(void)
{
   double Fe_first = 0;
   if(LM>RM)
   {
      Fe_first=fitness();
      if(Fe_first>0)
      {
         Fe=0;
      }
      else
      {
         Fe=-Fe_first;
      }
      weight_Fe_right=1;
      weight_Fe_left=0;
   }
   else
   {
      Fe_first=fitness();
      if(Fe_first>0)
      {
         Fe=0;
      }
      else
      {
         Fe=-Fe_first;
      }
      weight_Fe_right=0;
      weight_Fe_left=0;
   }
}

//Run the neural network
void neural_network(void)
{
   float nodes[4]={0,0,0,0};
   float weights_left[4][3]={3,3,3,1,1,1,0.5,0.5,0.5,0.25,0.25,0.25};
   float weights_right[4][3]={0.25,0.25,0.25,0.5,0.5,0.5,1,1,1,3,3,3};
   float weights_result[4]={0.25,0.25,0.25,0.25};
   float nodes_final_left=0;
   float nodes_final_right=0;
   float weights_final_left[2]={0.7,0.3};
   float weights_final_right[2]={0.3,0.7};
   int n,w,s;
   //Right Side of Nodes
   for(n=0;n<4;n++)
   {
      w=0;
      for(s=0;s<3;s++)
      {
         nodes[n]=weights_right[n][w]*ps_value[s];
         w++;
      }
   }
   w=0;
   for(n=0;n<4;n++)
   {
      nodes_final_right=nodes_final_right+nodes[n]*weights_result[w];
      w++;
   }
   nodes_final_right=nodes_final_right+Fe*weight_Fe_right;
   //Left Side of Nodes
   for(n=0;n<4;n++)
   {
      w=0;
      for(s=5;s<8;s++)
      {
         nodes[n]=weights_left[n][w]*ps_value[s];
         w++;
      }
   }
   w=0;
   for(n=0;n<4;n++)
   {
      nodes_final_left=nodes_final_left+nodes[n]*weights_result[w];
      w++;
   }
   nodes_final_left=nodes_final_left+Fe*weight_Fe_left;
   //Getting values for the motors
   LM=nodes_final_left*weights_final_left[0]+nodes_final_right*weights_final_right[0]+10;
   RM=nodes_final_left*weights_final_left[1]+nodes_final_right*weights_final_right[1]+10;

}

//Main Program
int main()
{
    wb_robot_init();
    reset_sensors();
    wb_differential_wheels_enable_encoders(128);
    wb_differential_wheels_set_encoders(0,0);

    time_step = wb_robot_get_basic_time_step();
    emitter = wb_robot_get_device("emitter");
    receiver = wb_robot_get_device("receiver");
    wb_receiver_enable(receiver, get_time_step());

    display = wb_robot_get_device("display");
    display_width = wb_display_get_width(display);
    display_height = wb_display_get_height(display);

    while(1)
    {
        wb_display_draw_text(display, "fitness", 2, 2);

        population = population_create(POPULATION_SIZE, GENOTYPE_SIZE);
        previous_x = robot_rot0[X];
        previous_y = robot_rot0[Y];

        WbNodeRef robot = wb_supervisor_node_get_from_def("ROBOT");
        robot_translation = wb_supervisor_node_get_field(robot, "translation");
        robot_rotation = wb_supervisor_node_get_field(robot, "rotation");

        memcpy(robot_trans0, wb_supervisor_field_get_sf_vec3f(robot_translation), sizeof(robot_trans0));
        memcpy(robot_rot0, wb_supervisor_field_get_sf_rotation(robot_rotation), sizeof(robot_rot0));

        run_optimization();
        run_neural_network();
    }
    wb_robot_cleanup();
    return 0;
}
