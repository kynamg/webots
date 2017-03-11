//   Description:   Supervisor code for genetic algorithm

#include "genotype.h"
#include "population.h"
#include <webots/supervisor.h>
#include <webots/robot.h>
#include <webots/emitter.h>
#include <webots/receiver.h> //locate this h file
#include <webots/display.h>
#include <string.h>
#include <math.h>
#include <stdio.h>

static int time_step;
static WbDeviceTag receiver;
static double floor_sensor_val[3] = {250, 250, 250}; //check in debugging - 300 - values for this need to be investigated
//fields for distance calculations
static double total_dist, max_dist, dist_from_start;
static double previous_x, previous_y;
//fields for the fitness function
static int punishment, reward, off_maze;

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

static bool demo = false; //demo used in run_seconds

void draw_scaled_line(int generation, double y1, double y2) {
  const double XSCALE = (double)display_width / NUM_GENERATIONS;
  const double YSCALE = 10.0;
  wb_display_draw_line(display, (generation - 0.5) * XSCALE, display_height - y1 * YSCALE,
    (generation + 0.5) * XSCALE, display_height - y2 * YSCALE);
}

// plot best and average fitness
void plot_fitness(int generation, double best_fitness, double average_fitness) {
  static double prev_best_fitness = 0.0;
  static double prev_average_fitness = 0.0;
  if (generation > 0) {  
    wb_display_set_color(display, 0xff0000); // red
    draw_scaled_line(generation, prev_best_fitness, best_fitness);

    wb_display_set_color(display, 0x00ff00); // green
    draw_scaled_line(generation, prev_average_fitness, average_fitness);
  }

  prev_best_fitness = best_fitness;
  prev_average_fitness = average_fitness;
}
 
// run the robot simulation for the specified number of seconds
void run_seconds(double seconds) {
  int i, n = 1000.0 * seconds / time_step;
  for (i = 0; i < n; i++) {
    if (demo && wb_robot_keyboard_get_key() == 'O') {
      demo = false;
      return; // interrupt demo and start GA optimization
    }

    wb_robot_step(time_step);
  }
}

//written by Kyna Mowat-Gosnell
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

//check if robot is inside the black EPM arms
int detect_maze_edge()
{
  if(floor_sensor_val[0] > 500) //edge left
  {
    return 0;
  }
  else if(floor_sensor_val[1] > 500) //edge forward
  {
    return 1;
  }
   else if(floor_sensor_val[2] > 500) //edge right
  {
    return 2;
  }
  else return 4; //no edge detected - check in debugging
}

void check()
{
  int f, r, maze;
  //check if receiver is getting info
  if (wb_receiver_get_queue_length(receiver) > 0) 
  {
     const double *array = wb_receiver_get_data(receiver);
    reward = 0; //reset inside loop
    reward = array[0]; //set variable to first element in array
    
    //floor sensor data
    for(f = 0; f < 3; f++)
    {
      floor_sensor_val[f] = array[f + 8];
    }    
  }
}
  
double measure_fitness() {
  //const double *load_trans = wb_supervisor_field_get_sf_vec3f(load_translation);
  //double dx = load_trans[X] - load_trans0[X];
  //double dz = load_trans[Z] - load_trans0[Z];
  //return sqrt(dx * dx + dz * dz);
}

// evaluate one genotype at a time
void evaluate_genotype(Genotype genotype) {
  
  // send genotype to robot for evaluation
  wb_emitter_send(emitter, genotype_get_genes(genotype), GENOTYPE_SIZE * sizeof(double));
  
  // reset robot vector position and rotation
  wb_supervisor_field_set_sf_vec3f(robot_translation, robot_trans0);
  wb_supervisor_field_set_sf_rotation(robot_rotation, robot_rot0);

  // evaluation genotype during one minute
  run_seconds(60.0);
  
  // measure fitness
  double fitness = measure_fitness();
  genotype_set_fitness(genotype, fitness);

  printf("fitness: %g\n", fitness);
}

void run_optimization() {
  wb_robot_keyboard_disable();

  printf("---\n");
  printf("starting GA optimization ...\n");
  printf("population size is %d, genome size is %d\n", POPULATION_SIZE, GENOTYPE_SIZE);

  int i, j;
  for  (i = 0; i < NUM_GENERATIONS; i++) {    
    for (j = 0; j < POPULATION_SIZE; j++) {
      printf("generation: %d, genotype: %d\n", i, j);

      // evaluate genotype
      Genotype genotype = population_get_genotype(population, j);
      evaluate_genotype(genotype);
    }
  
    double best_fitness = genotype_get_fitness(population_get_fittest(population));
    double average_fitness = population_compute_average_fitness(population);
    
    // display results
    plot_fitness(i, best_fitness, average_fitness);
    printf("best fitness: %g\n", best_fitness);
    printf("average fitness: %g\n", average_fitness);
    
    // reproduce (but not after the last generation)
    if (i < NUM_GENERATIONS - 1)
      population_reproduce(population);
  }
  
  printf("GA optimization terminated.\n");

  // save fittest individual
  Genotype fittest = population_get_fittest(population);
  FILE *outfile = fopen(FILE_NAME, "w");
  if (outfile) {
    genotype_fwrite(fittest, outfile);
    fclose(outfile);
    printf("wrote best genotype into %s\n", FILE_NAME);
  }
  else
    printf("unable to write %s\n", FILE_NAME);
  
  population_destroy(population);
}
  
// show demo of the fittest individual
void run_demo() {
  wb_robot_keyboard_enable(time_step);
  
  printf("---\n");
  printf("running demo of best individual ...\n");
  printf("select the 3D window and push the 'O' key\n");
  printf("to start genetic algorithm optimization\n");

  FILE *infile = fopen(FILE_NAME, "r");
  if (! infile) {
    printf("unable to read %s\n", FILE_NAME);
    return;
  }
  
  Genotype genotype = genotype_create();
  genotype_fread(genotype, infile);
  fclose(infile);
  
  while (demo)
    evaluate_genotype(genotype);
}

int main(int argc, const char *argv[]) {
  
  // initialize Webots
  wb_robot_init();
  
  // get simulation step in milliseconds
  time_step = wb_robot_get_basic_time_step();

  // the emitter to send genotype to robot
  emitter = wb_robot_get_device("emitter");
  
  // to display the fitness evolution
  display = wb_robot_get_device("display");
  display_width = wb_display_get_width(display);
  display_height = wb_display_get_height(display);
  wb_display_draw_text(display, "fitness", 2, 2);

  // initial population
  population = population_create(POPULATION_SIZE, GENOTYPE_SIZE);
  
  // find robot node and store initial position and orientation
  WbNodeRef robot = wb_supervisor_node_get_from_def("ROBOT");
  robot_translation = wb_supervisor_node_get_field(robot, "translation");
  robot_rotation = wb_supervisor_node_get_field(robot, "rotation");
  memcpy(robot_trans0, wb_supervisor_field_get_sf_vec3f(robot_translation), sizeof(robot_trans0));
  memcpy(robot_rot0, wb_supervisor_field_get_sf_rotation(robot_rotation), sizeof(robot_rot0));
  
  if (demo)
    run_demo();

  // run GA optimization
  run_optimization();
  
  // cleanup Webots
  wb_robot_cleanup();
  return 0;  // ignored
}
