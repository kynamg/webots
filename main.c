#include <stdio.h>
#include <stdlib.h>

#include <advanced_genetic_algorithm_supervisor.c>
//Include the Webots libraries
#include <webots/robots.h>
#include <webots/differential_wheels.h>
#include <webots/distance_sensor.h>

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
int ps_offset_sim[NB_DIST_SENS] = {35,35,35,35,35,35,35,35};
int ps_offset_real[NB_DIST_SENS] = {375,158,423,682,447,594,142,360}; // to be modified according to your robot
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
   for(i=0;i<NB_Sens_Dist;i++)
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
static void Get_Fe(void)
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
static void neural_network(void)
{
   float nodes[4]={0,0,0,0};
   float weights_left[4][3]={3,3,3;1,1,1;0.5,0.5,0.5;0.25,0.25,0.25};
   float weights_right[4][3]={0.25,0.25,0.25;0.5,0.5,0.5;1,1,1;3,3,3};
   float weights_result[4]={0.25,0.25,0.25,0.25};
   float nodes_final_left=0;
   float nodes_final_right=0;
   float weights_final_left[2]={0.7,0.3};
   float weights_final_right[2]={0.3,0.7};
   int n,w,s;
   //Right Side of Nodes
   for(n=0;n<nodes.length;n++)
   {
      w=0;
      for(s=0;s<3;s++)
      {      
         nodes[n]=weights_right[n][w]*ps_value[s];
         w++;
      }
   }
   w=0;
   for(n=0;n<nodes.length;n++)
   {
      nodes_final_right=nodes_final_right+nodes[n]*weights_result[w];
      w++;
   }
   nodes_final_right=nodes_final_right+Fe*weight_Fe_right;
   //Left Side of Nodes      
   for(n=0;n<nodes.length;n++)
   {
      w=0;
      for(s=5;s<8;s++)
      {
         nodes[n]=weights_left[n][w]*ps_value[s];
         w++;
      }
   }
   w=0;
   for(n=0;n<nodes.length;n++)
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
    while(1)
    {
      run_neural_network();
    }
    return 0;
}
