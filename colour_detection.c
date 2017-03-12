//Description: e-puck to detect colour changes in floor

#include <webots/robot.h>
#include <webots/emitter.h>
#include <webots/receiver.h>
#include <webots/distance_sensor.h>
#include <webots/differential_wheels.h>
#include <webots/nodes.h>
#include <webots/led.h>
#include <webots/device.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>

static WbDeviceTag receiver;
static WbDeviceTag emmitter;
static int time_step

//differential wheel def
#define NUM_WHEELS 2
#define MAX_SPEED 500.0
static double wheel_speed[NUM_WHEELS]

//genotype def
#define GENOTYPE_SIZE (NUM_SENSORS * NUM_WHEELS)

//floor sensor device def
#define GROUND_SENSORS_NUMBER 3
static WbDeviceTag ground_sensors[GROUND_SENSORS_NUMBER];
static double ground_sensors_values[GROUND_SENSORS_NUMBER] = { 0.0, 0.0, 0.0 };
static const char *ground_sensors_labels[GROUND_SENSORS_NUMBER] = {
  "gs0", "gs1", "gs2"
};

//distance sensor device def
#define NUM_SENSORS 8
static WbDeviceTag distance_sensors[NUM_SENSORS];
static double distance_sensors_values_total[NUM_SENSORS];
static double distance_sensors_values[NUM_SENSORS];
static const char *distance_sensors_labels[NUM_SENSORS] = {
  "ps0", "ps1", "ps2", "ps3",
  "ps4", "ps5", "ps6", "ps7"
};


