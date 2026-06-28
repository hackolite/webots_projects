/*
 * iRobot Create controller + front_camera
 */

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/touch_sensor.h>
#include <webots/distance_sensor.h>
#include <webots/position_sensor.h>
#include <webots/receiver.h>
#include <webots/led.h>
#include <webots/camera.h>

/* ================= CONFIG ================= */

#define BUMPERS_NUMBER 2
#define BUMPER_LEFT 0
#define BUMPER_RIGHT 1

static WbDeviceTag bumpers[BUMPERS_NUMBER];
static const char *bumpers_name[BUMPERS_NUMBER] = {
  "bumper_left", "bumper_right"
};

#define CLIFF_SENSORS_NUMBER 4
static WbDeviceTag cliff_sensors[CLIFF_SENSORS_NUMBER];
static const char *cliff_sensors_name[CLIFF_SENSORS_NUMBER] = {
  "cliff_left", "cliff_front_left", "cliff_front_right", "cliff_right"
};

#define LEDS_NUMBER 3
static WbDeviceTag leds[LEDS_NUMBER];
static const char *leds_name[LEDS_NUMBER] = {
  "led_on", "led_play", "led_step"
};

static WbDeviceTag receiver;
static const char *receiver_name = "receiver";

static WbDeviceTag left_motor, right_motor;
static WbDeviceTag left_position_sensor, right_position_sensor;

/* CAMERA */
static WbDeviceTag camera;

/* ================= CONSTANTS ================= */

#define MAX_SPEED 16.0
#define HALF_SPEED 8.0

#define WHEEL_RADIUS 0.031
#define AXLE_LENGTH 0.271756

static int time_step;

/* ================= UTILS ================= */

static void step() {
  if (wb_robot_step(time_step) == -1) {
    wb_robot_cleanup();
    exit(0);
  }
}

/* ================= INIT ================= */

static void init_devices() {
  int i;

  receiver = wb_robot_get_device(receiver_name);
  wb_receiver_enable(receiver, time_step);

  for (i = 0; i < LEDS_NUMBER; i++)
    leds[i] = wb_robot_get_device(leds_name[i]);

  for (i = 0; i < BUMPERS_NUMBER; i++) {
    bumpers[i] = wb_robot_get_device(bumpers_name[i]);
    wb_touch_sensor_enable(bumpers[i], time_step);
  }

  for (i = 0; i < CLIFF_SENSORS_NUMBER; i++) {
    cliff_sensors[i] = wb_robot_get_device(cliff_sensors_name[i]);
    wb_distance_sensor_enable(cliff_sensors[i], time_step);
  }

  left_motor = wb_robot_get_device("left wheel motor");
  right_motor = wb_robot_get_device("right wheel motor");

  wb_motor_set_position(left_motor, INFINITY);
  wb_motor_set_position(right_motor, INFINITY);

  left_position_sensor = wb_robot_get_device("left wheel sensor");
  right_position_sensor = wb_robot_get_device("right wheel sensor");

  wb_position_sensor_enable(left_position_sensor, time_step);
  wb_position_sensor_enable(right_position_sensor, time_step);

  /* ================= CAMERA ================= */
  camera = wb_robot_get_device("front_camera");

  if (camera == 0) {
    printf("ERROR: front_camera not found\n");
  } else {
    wb_camera_enable(camera, time_step);
    printf("front_camera enabled: %dx%d\n",
           wb_camera_get_width(camera),
           wb_camera_get_height(camera));
  }
}

/* ================= SENSORS ================= */

static bool bump_left() {
  return wb_touch_sensor_get_value(bumpers[BUMPER_LEFT]) != 0.0;
}

static bool bump_right() {
  return wb_touch_sensor_get_value(bumpers[BUMPER_RIGHT]) != 0.0;
}

static bool cliff_left() {
  return wb_distance_sensor_get_value(cliff_sensors[0]) < 100.0 ||
         wb_distance_sensor_get_value(cliff_sensors[1]) < 100.0;
}

static bool cliff_right() {
  return wb_distance_sensor_get_value(cliff_sensors[2]) < 100.0 ||
         wb_distance_sensor_get_value(cliff_sensors[3]) < 100.0;
}

static bool cliff_front() {
  return wb_distance_sensor_get_value(cliff_sensors[1]) < 100.0 ||
         wb_distance_sensor_get_value(cliff_sensors[2]) < 100.0;
}

static bool virtual_wall() {
  return wb_receiver_get_queue_length(receiver) > 0;
}

/* ================= MOTION ================= */

static void forward() {
  wb_motor_set_velocity(left_motor, MAX_SPEED);
  wb_motor_set_velocity(right_motor, MAX_SPEED);
}

static void backward() {
  wb_motor_set_velocity(left_motor, -HALF_SPEED);
  wb_motor_set_velocity(right_motor, -HALF_SPEED);
}

static void stop() {
  wb_motor_set_velocity(left_motor, 0);
  wb_motor_set_velocity(right_motor, 0);
}

static double rand01() {
  return rand() / (double)RAND_MAX;
}

static void turn(double angle) {
  stop();

  double l0 = wb_position_sensor_get_value(left_position_sensor);
  double r0 = wb_position_sensor_get_value(right_position_sensor);

  step();

  double dir = (angle > 0) ? 1.0 : -1.0;

  wb_motor_set_velocity(left_motor, -dir * HALF_SPEED);
  wb_motor_set_velocity(right_motor, dir * HALF_SPEED);

  double orientation = 0.0;

  do {
    double l = wb_position_sensor_get_value(left_position_sensor) - l0;
    double r = wb_position_sensor_get_value(right_position_sensor) - r0;

    double dl = l * WHEEL_RADIUS;
    double dr = r * WHEEL_RADIUS;

    orientation = (dl - dr) / AXLE_LENGTH;

    step();
  } while (fabs(orientation) < fabs(angle));

  stop();
}

/* ================= MAIN ================= */

int main(int argc, char **argv) {
  wb_robot_init();

  time_step = wb_robot_get_basic_time_step();

  printf("iRobot Create started\n");

  init_devices();
  srand(time(NULL));

  while (1) {

    /* CAMERA READ */
    const unsigned char *img = wb_camera_get_image(camera);
    if (img)
      printf("camera OK\n");

    if (virtual_wall()) {
      turn(M_PI);

    } else if (bump_left() || cliff_left()) {
      backward();
      step();
      turn(M_PI * rand01());

    } else if (bump_right() || cliff_right() || cliff_front()) {
      backward();
      step();
      turn(-M_PI * rand01());

    } else {
      forward();
    }

    step();
  }

  return 0;
}