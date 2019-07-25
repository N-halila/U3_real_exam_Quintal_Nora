/*
 * File:          U3_real_exam_Quintal_Nora.c
 * Date:
 * Description:
 * Author:
 * Modifications:
 */
#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/position_sensor.h>
#include <webots/distance_sensor.h>
#include <stdio.h>
#include <math.h>

#define TIME_STEP 64
#define PI 3.14159
#define OBSTACLE_DISTANCE 0.2
#define RANGE 0.4
#define MAXRE 255
#define OBSTACLE_DISTANCE_2 0.72
#define RANGE2 2
#define MAXRE2 1023

double encoder;
double dis;
double dis2;

enum {
  GO,
  TURN,
  FREEWAY,
  OBSTACLE,
  AIMING,
  ENEMY,
  EMPTY,
  SHOOTING
};

double initial_angle_wheel1;

int checkForObstacles(WbDeviceTag sen_1) {
  double distance = wb_distance_sensor_get_value(sen_1);

  dis = (distance*RANGE)/MAXRE;

    if (dis > OBSTACLE_DISTANCE)
      return FREEWAY;
    else
      return OBSTACLE;
}

int checkForEnemy(WbDeviceTag distance_sensor) {
  double distance2 = wb_distance_sensor_get_value(distance_sensor);

  dis2 = (distance2*RANGE2)/MAXRE2;

  if(dis2 <= 0.8) {
    printf("THE ENEMY IS COMMING: THA!\n");
  }

  if (dis2 > OBSTACLE_DISTANCE_2)
    return EMPTY;
  else
    return ENEMY;
}

void goRobot(WbDeviceTag *wheels, double vel) {
  wb_motor_set_position(wheels[0], INFINITY);
  wb_motor_set_velocity(wheels[0], 0);
  wb_motor_set_position(wheels[1], INFINITY);
  wb_motor_set_velocity(wheels[1], vel);
  wb_motor_set_position(wheels[2], INFINITY);
  wb_motor_set_velocity(wheels[2], -vel);
}

void stopRobot(WbDeviceTag *wheels) {
  wb_motor_set_velocity(wheels[0], 0);
  wb_motor_set_velocity(wheels[1], 0);
  wb_motor_set_velocity(wheels[2], 0);
}

void turnRight(WbDeviceTag *wheels) {

  wb_motor_set_velocity(wheels[0], 6);
  wb_motor_set_velocity(wheels[1], 6);
  wb_motor_set_velocity(wheels[2], 6);
}

void sensorRotate(WbDeviceTag *radar) {
  wb_motor_set_position(radar[1], INFINITY);
  wb_motor_set_velocity(radar[1], -3);
}

void stopSensor(WbDeviceTag *radar) {
  wb_motor_set_position(radar[1], INFINITY);
  wb_motor_set_velocity(radar[1], 0);
}

void rotateGun(WbDeviceTag *gun, double pos) {
  wb_motor_set_position(gun[0], pos);

}

double getAngleRobot(WbDeviceTag pos_sensor) {

  double angle, angle_wheel1;
  angle_wheel1 = wb_position_sensor_get_value(pos_sensor);
  angle = fabs(angle_wheel1 - initial_angle_wheel1);
  return angle;
}

double getAngle(WbDeviceTag radar_sensor) {
  double  angle_2, radar_angle;
  radar_angle = wb_position_sensor_get_value(radar_sensor);
  angle_2 = radar_angle;
  return angle_2;
}


int main(int argc, char **argv)
{
  
  wb_robot_init();

  WbDeviceTag wheels[3];
    wheels[0] = wb_robot_get_device("wheel1");
    wheels[1] = wb_robot_get_device("wheel2");
    wheels[2] = wb_robot_get_device("wheel3");

  WbDeviceTag radar[1] = wb_robot_get_device("RADAR");
  WbDeviceTag Gun[1] = wb_robot_get_device("RT_GUN");

  // Encoder devices
  WbDeviceTag encoder = wb_robot_get_device("encoder1");
  wb_position_sensor_enable(encoder, TIME_STEP);

  WbDeviceTag radar_sen_pos = wb_robot_get_device("POS_RAD");
  wb_position_sensor_enable(radar_sen_pos, TIME_STEP);

  // Distance sensor devices
  WbDeviceTag dis_sen = wb_robot_get_device("DISTANCE_BODY");
  wb_distance_sensor_enable(dis_sen, TIME_STEP);

  WbDeviceTag rad = wb_robot_get_device("DS_RADAR");
  wb_distance_sensor_enable(rad, TIME_STEP);

  short int ds_state,robot_state = GO;
  float vel = 4;
  float angle;
  float ang2;
  float pos;
  int turns = 20;
  while (wb_robot_step(TIME_STEP) != -1) {

    sensorRotate(radar);

    if (robot_state == GO) {
      ds_state = checkForObstacles(dis_sen);
      

      if (ds_state == FREEWAY) {
        goRobot(wheels, vel);
        angle = wb_position_sensor_get_value(encoder);
        printf("We are free of enemys\n");

      } else if (ds_state == OBSTACLE) {
        robot_state = TURN;
        stopRobot(wheels);
        initial_angle_wheel1 = wb_position_sensor_get_value(encoder);
      }
    } else if (robot_state == TURN) {
        turnRight(wheels);
        angle = getAngleRobot(encoder);

          if (angle >= PI ) {
            robot_state = AIMING;
            stopRobot(wheels);
            printf("We are free of enemys\n");
          }
    }
    if (robot_state == AIMING) {
      ds_state = checkForEnemy(rad);

        if (ds_state == EMPTY) {
          goRobot(wheels, vel);
          printf("We are free of enemys\n");
        }
        else if (ds_state == ENEMY) {
          ds_state = SHOOTING;
          stopRobot(wheels);
          stopSensor(radar);
          ang2 = getAngle(radar_sen_pos);
          pos = (turns + ang2)*-1;
        }
        if (ds_state== SHOOTING) {

          rotateGun(gun, pos);
            if (dis2 >= 0.6 && dis2 <= 0.4 ) {
              printf("The Enemy IS COMMING: THA THA!\n");
            }
            else if (dis2 <= 0.4 && dis2 >= 0.2) {
              printf("Enemy: THA THA THA THA!!\n");
            }
            else if (dis2 <= 0.2 && dis2 >= 0.1) {
              printf("Enemy: THA THA THA THA THA THA!!!\n");
            }
        }
    }

  };
    wb_robot_cleanup();

  return 0;
}
