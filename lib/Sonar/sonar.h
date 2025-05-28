#ifndef __SONAR_H__
#define __SONAR_H__

/**************************************************************************************************
*
*   INCLUDES
*
**************************************************************************************************/

#include <Arduino.h>
#include <MovingAverage.h>

/**************************************************************************************************
*
*   DEFINES
*
**************************************************************************************************/

#define CAR1_PIXY 0
#define CAR2_ARM 1

#if CAR1_PIXY == 1
#define DISTANCE_SENSOR_TRIG_PIN A2
#define DISTANCE_SENSOR_ECHO_PIN A3
#endif


#if CAR2_ARM == 1
#define DISTANCE_SENSOR_TRIG_PIN 12
#define DISTANCE_SENSOR_ECHO_PIN 13
#endif

#define ULTRASONIC_TIME_MU 10

/**************************************************************************************************
*
*   VARIABLE DECLARATIONS
*
**************************************************************************************************/

static float duration;
static float lastSonarValue;
static MovingAverage movingAverage(MOVING_AVG_SIZE);
static float startMillis;

/**************************************************************************************************
*
*   FUNCTION DECLARATIONS
*
**************************************************************************************************/

void setupSonar();
extern float getFrontObstacleDistance_cm();

#endif
