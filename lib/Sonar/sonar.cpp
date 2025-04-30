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
extern float getFrontObstacleDistance_cm()
{
  // Generate an ultrasonic pulse that lasts for ULTRASONIC_TIME_MS microseconds
  digitalWrite(DISTANCE_SENSOR_TRIG_PIN, HIGH);
  delayMicroseconds(ULTRASONIC_TIME_MS);
  digitalWrite(DISTANCE_SENSOR_TRIG_PIN, LOW);

  // Measure the length in microseconds
  float distanceMS = pulseIn(DISTANCE_SENSOR_ECHO_PIN, HIGH);

  // Convert the length in microseconds to cm
  float distanceCM = distanceMS * 0.017;
  
  Serial.println(distanceMS);
  Serial.println(distanceCM);
  return distanceCM;
}