//neural_net emitts sensor values which are received by genetic_algorithm_supervisor
//genetic_algorithm_supervisor emitts fitness values which are received by neural_net

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

//Include the Webots libraries
#include <webots/robot.h>
#include <webots/differential_wheels.h>
#include <webots/distance_sensor.h>
#include <webots/emitter.h>
#include <webots/receiver.h>

#define THRESHOLD_VALUE 100
#define SIMULATION 0
//Defining the IR Distance Sensors
#define NB_Dist_Sens  8
#define PS_Right_10   0
#define PS_Right_45   1
#define PS_Right_90   2
#define PS_Right_Rear 3
#define PS_Left_Rear  4
#define PS_Left_90    5
#define PS_Left_45    6
#define PS_Left_10    7

static int time_step;
WbDeviceTag ps[NB_Dist_Sens];
int ps_value[NB_Dist_Sens]={0,0,0,0,0,0,0,0};
int ps_ofgset_sim[NB_Dist_Sens] = {35,35,35,35,35,35,35,35};
int ps_ofgset_real[NB_Dist_Sens] = {375,158,423,682,447,594,142,360}; // to be modified according to your robot
char *ps_text[]={"one","two","three","five","seven","nine","ten","eleven"};

//defining IR floor sensors
#define NB_Floor_Sens 3
#define PS_Floor_1 0
#define PS_Floor_2 1
#define PS_Floor_3 2
WbDeviceTag gs[NB_Floor_Sens];
int gs_value[NB_Floor_Sens]={0,0,0};
static const char *floor_sensor_label[NB_Floor_Sens] = {"gs0","gs1","gs2"};

//Defining the motors
#define LEFT 1
#define RIGHT 2
float LM=0;
float RM=0;

//Defining the fear factor in the neural network and its weights
float Fe = 0;
float weight_Fe_left = 0;
float weight_Fe_right = 0;

static WbDeviceTag emitter;   // to send genes to robot
static WbDeviceTag receiver; //to receive fitness from supervisor

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
    
    for(it=0; it<NB_Floor_Sens; it++)
    {
      gs[it] = wb_robot_get_device(floor_sensor_label[it]);
      wb_distance_sensor_enable(gs[it],32);
    }
}

//Get the distances values from the IR sensors to detect obstacles
static void run_sensors(void)
{
   int i;
   int ps_ofgset[NB_Dist_Sens]={0,0,0,0,0,0,0,0};
   int mode = wb_robot_get_mode();
   if(mode==SIMULATION)
   {
      for(i=0;i<NB_Dist_Sens;i++)
      {
         ps_ofgset[i]=ps_ofgset_sim[i];
      }
   }
   else
   {
      for(i=0;i<NB_Dist_Sens;i++)
      {
         ps_ofgset[i]=ps_ofgset_real[i];
      }
   }
   for(i=0;i<NB_Dist_Sens;i++)
   {
      ps_value[i]= (int) wb_distance_sensor_get_value(ps[i]);
   }

   for(i=0;i<NB_Dist_Sens;i++)
   {
      if(ps_value[i]-ps_ofgset[i]>THRESHOLD_VALUE)
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



//Get the fear factor and which wheel it needs to apply to
void Get_Fe(void)
{
   double Fe_first = 0;
   
   while(wb_receiver_get_queue_length(receiver)>0)
   {
       const double *fitness_receive = wb_receiver_get_data(emitter);
       wb_receiver_next_packet(receiver); 
       if(LM>RM)
       {
          if(fitness_receive[0] > 0)
          {
             Fe=0;
          }
          else
          {
             Fe=-fitness_receive[0];
          }
          weight_Fe_right=1;
          weight_Fe_left=0;
       }
       else
       {
          if(fitness_receive[0]>0)
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
}

//Run the neural network
void neural_network(void)
{
   float nodes[4]={0,0,0,0};
   float weights_left[4][3]={{3,3,3},{1,1,1},{0.5,0.5,0.5},{0.25,0.25,0.25}};
   float weights_right[4][3]={{0.25,0.25,0.25},{0.5,0.5,0.5},{1,1,1},{3,3,3}};
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

static void emit_data()
{
  int a, b;
  
  static double all_sensors[11];
  
  for(a = 0; a < 8; a++)
  {
    all_sensors[a] = ps_value[a];
  }
  
  for(b = 0; b < 3; b++)
  {
    all_sensors[b] = gs_value[b];  
  }
  
   wb_emitter_send(emitter, all_sensors, sizeof(all_sensors));
}

//Run the whole neural network
static void run_neural_network(void)
{
   run_sensors();
   emit_data();
   Get_Fe();
   neural_network();
   motor_set_speed();
   Fe=0;
}

//Main Program
int main()
{
    wb_robot_init();
    emitter = wb_robot_get_device("emitter");
    receiver = wb_robot_get_device("receiver");
    wb_receiver_enable(receiver, get_time_step());
    reset_sensors();
    wb_differential_wheels_enable_encoders(128);
    wb_differential_wheels_set_encoders(0,0);

    time_step = wb_robot_get_basic_time_step();

    while(wb_robot_step(time_step) != -1)
    {
      run_neural_network();
    }
    wb_robot_cleanup();
    return 0;
}
