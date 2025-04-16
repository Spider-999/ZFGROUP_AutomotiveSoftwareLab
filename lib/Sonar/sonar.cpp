#include "direction.h"
#include "sonar.h"


/**************************************************************************************************
                              FUNCTION INFO
NAME:
    setupSonar

DESCRIPTION:
    Setup pins for sonar.

**************************************************************************************************/
void setupSonar()
{
  #if CAR1_PIXY == 1
    pinMode(DISTANCE_SENSOR_TRIG_PIN, OUTPUT); // ultrasonic trigger
    pinMode(DISTANCE_SENSOR_ECHO_PIN, INPUT); // ultrasonic echo
  #endif

  #if CAR2_ARM == 1
    pinMode(DISTANCE_SENSOR_TRIG_PIN, OUTPUT); // ultrasonic trigger
    pinMode(DISTANCE_SENSOR_ECHO_PIN, INPUT); // ultrasonic echo
  #endif
}

/**************************************************************************************************
                              FUNCTION INFO
NAME:
    getFrontObstacleDistance_cm

DESCRIPTION:
    Returns the distance to the obstacle.

**************************************************************************************************/
static float getFrontObstacleDistance_cm()
{

}