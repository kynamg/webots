#ifndef ADVANCED_GENETIC_ALGORITHM_SUPERVISOR_H
#define ADVANCED_GENETIC_ALGORITHM_SUPERVISOR_H

#include "genotype.h"
#include "population.h"
include "random.h"

// abstract type definition
typedef struct _ADVANCED_GENETIC_ALGORITHM_SUPERVISOR_ *Advanced_genetic_algorithm_supervisor;

//methods
void draw_scaled_line(int generation, double y1, double y2)

void plot_fitness(int generation, double best_fitness, double average_fitness)

int get_time_step()

int detect_maze_edge()

void robot_check()

void run_seconds(double seconds)

double fitness()

void evaluate_genotype(Genotype genotype)

void run_optimization()

#endif