/*
 * File:          U3_real_exam_Quintal_Nora.c
 * Date:
 * Description:
 * Author:
 * Modifications:
 */

/*
 * You may need to add include files like <webots/distance_sensor.h> or
 * <webots/differential_wheels.h>, etc.
 */
#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/distance_sensor.h>
#include <webots/position_sensor.h>


#include <stdio.h>
#include <math.h>

/*
 * macros
 */
#define TIME_STEP 64
#define PI 3.141592

#define OBSTACLE_DISTANCE 0.2
#define RANGE 0.4 
#define MAXRE 255  


#define OBSTACLE_DISTANCE_2 0.72 
#define RANGE2 2 
#define MAXRE2 1023

double encoder;
double dis_1;
double dis_2;  

enum {
  GO,
  TURN,
  FreeWay,
  OBSTACLE,
  LOOKING,
  Enemy, 
  ANYENEMY,
  STOP
};

double initial_angle_wheel1;

int checkForObstacles(WbDeviceTag dis_S1) {
  double distance = wb_distance_sensor_get_value(dis_S1);

  if (distance > OBSTACLE_DISTANCE)
    return FreeWay;
  else
    return OBSTACLE;
}


int checkForTheEnemy(WbDeviceTag distance_sensor) {
  double distance2 = wb_distance_sensor_get_value(distance_sensor);
  
  dis2 = (distance2*RANGE2)/MAXRE2;
  
  if(dis2 <= 0.7) {
      printf("The Enemy!: THA!\n");
  }
  
  if (dis2 > OBSTACLE_DISTANCE_2)
    return ANYENEMY;
  else
    return Enemy;
}


//////Functions for the robot
/////avanzar
void goRobot(WbDeviceTag *wheels, double velocity) {
  wb_motor_set_position(wheels[0], INFINITY);
  wb_motor_set_velocity(wheels[0], 0);
  wb_motor_set_position(wheels[1], INFINITY);
  wb_motor_set_velocity(wheels[1], velocity);
  wb_motor_set_position(wheels[2], INFINITY);
  wb_motor_set_velocity(wheels[2], -velocity);
}
///detener
void stopRobot(WbDeviceTag *wheels) {
  wb_motor_set_velocity(wheels[0], 0);
  wb_motor_set_velocity(wheels[1], 0);
  wb_motor_set_velocity(wheels[2], 0);
}

///girar derecha
void turnRight(WbDeviceTag *wheels) {
  wb_motor_set_velocity(wheels[0],-6);
  wb_motor_set_velocity(wheels[1],-6);
  wb_motor_set_velocity(wheels[2],-6);
}

void sensorRotate(WbDeviceTag *radar) {
  wb_motor_set_position(radar[0], INFINITY);
  wb_motor_set_velocity(radar[0], -3);
}

void stopSensor(WbDeviceTag *radar_gun) {
  wb_motor_set_position(radar[0], INFINITY);
  wb_motor_set_velocity(radar[0], 0);
}

void gunRotation(WbDeviceTag *gun, double pos_rad) {
  wb_motor_set_position(gun[0], pos_rad);
  
}


///angulo
double getAngleRobot(WbDeviceTag p_sen) {

  double angle_wheel1 = wb_position_sensor_get_value(p_sen);
  double angle;

  angle = fabs(angle_wheel1 - initial_angle_wheel1);

  return angle;
}


double radarGetAngle(WbDeviceTag pos_rad){
  double angle_radar = wb_position_sensor_get_value(radar_sensor);
  double second_angle;
  
  second_angle= angle_radar;
  
  return second_angle;

}
/*
 * main
 */
int main(int argc, char **argv)
{
  /* necessary to initialize webots stuff */
  wb_robot_init();

   //inicializamos y guardamos en un array
   WbDeviceTag wheels[3];
     wheels[0] = wb_robot_get_device("wheel1");
     wheels[1] = wb_robot_get_device("wheel2");
     wheels[2] = wb_robot_get_device("wheel3");
     ///las hacemos rotar infinitamente
     //guardamos las variables del radar
   WbDeviceTag radar[1]= wb_robot_get_device("RADAR");
   WbDeviceTag Gun [1] = wb_robot_get_device("RT_GUN");


  ////obtenemos la info del sensor
   WbDeviceTag DSensor[2];
     DSensor[0] = wb_robot_get_device("DISTANCE_BODY");
     DSensor[1] = wb_robot_get_device("DS_RAD");
     ////activamos el sensor
   wb_distance_sensor_enable(DSensor[0], TIME_STEP);
   wb_distance_sensor_enable(DSensor[1], TIME_STEP);
   ////obtenemos la info y activamos el sensor
   WbDeviceTag encoder = wb_robot_get_device("encoder1");
   wb_position_sensor_enable(encoder, TIME_STEP);
   
   WbDeviceTag ps_radar = wb_robot_get_device("POS_RAD");
   wb_position_sensor_enable(ps_radar, TIME_STEP);
   
   WbDeviceTag ps_gun = wb_robot_get_device("POS_GUN");
   wb_position_sensor_enable(ps_gun, TIME_STEP);

  /*
   * main loop
   */
  while (wb_robot_step(TIME_STEP) != -1) {
       ///Para el autonomo debe hacerlo todo por su cuenta

      if (robot_state == Go) {
        ds_state = checkForObstacles(DSensor[0]);
        ds_state1 = checkForObstacles(DSensor[1]);

        dis1 = wb_distance_sensor_get_value(DSensor[0]);
        dis2 = wb_distance_sensor_get_value(DSensor[1]);
        printf ("dis1 : %lf\n",dis1);
        printf ("dis2 : %lf\n",dis2);

        if (ds_state == FreeWay && ds_state1 == FreeWay) {
          velocity = 8;///nueva velocidad para moverse automatico
          goRobot(wheels, velocity);
        } else if (ds_state == Obstacle && ds_state1 == FreeWay) {
            robot_state = TurnL;
            stopRobot(wheels);
        } else if (ds_state == FreeWay && ds_state1 == Obstacle) {
            robot_state = TurnR;
            stopRobot(wheels);
        } else if (ds_state == Obstacle && ds_state1 == Obstacle) {
            robot_state = TurnL;
            stopRobot(wheels);
        }
      } else if (robot_state == TurnL) {
          turnLeft(wheels);
          ds_state = checkForObstacles(DSensor[0]);
          ds_state1 = checkForObstacles(DSensor[1]);
          if (ds_state == FreeWay && ds_state1 == FreeWay) {
            robot_state = Go;
            stopRobot(wheels);
          }
      } else if (robot_state == TurnR) {
          turnRight(wheels);
          ds_state = checkForObstacles(DSensor[0]);
          ds_state1 = checkForObstacles(DSensor[1]);
          if (ds_state1 == FreeWay && ds_state == FreeWay) {
            robot_state = Go;
            stopRobot(wheels);
          }
      }
     }

        ) } else {
             stopRobot(wheels);
         }
          dis1 = wb_distance_sensor_get_value(DSensor[0]);
          dis2 = wb_distance_sensor_get_value(DSensor[1]);
          printf ("dis1 : %lf\n",dis1);///impresiones distance sensors
          printf ("dis2 : %lf\n",dis2);
       }


  /* Enter your cleanup code here */

  /* This is necessary to cleanup webots resources */
  wb_robot_cleanup();

  return 0;
}