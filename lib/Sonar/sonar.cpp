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
  float currentMillis = millis();  

  // Read the value once every 100 milliseconds
  if (currentMillis - startMillis >= 100)  
  {
    // calculations were made in centimeters
    static uint32_t pulseInTimeout_us = (uint32_t)((200.0f / 34300.0f) * 1000000.0f);
    float measured_distance = 0.0f;
    float estimated_distance = 0.0f;

    digitalWrite(DISTANCE_SENSOR_TRIG_PIN, LOW);
    delayMicroseconds(2);
    // Sets the DISTANCE_SENSOR_TRIG_PIN on HIGH state for 10 micro seconds
    digitalWrite(DISTANCE_SENSOR_TRIG_PIN, HIGH);
    delayMicroseconds(10); //This pin should be set to HIGH for 10 μs, at which point the HC­SR04 will send out an eight cycle sonic burst at 40 kHZ
    digitalWrite(DISTANCE_SENSOR_TRIG_PIN, LOW);

    // Reads the DISTANCE_SENSOR_ECHO_PIN, returns the sound wave travel time in microseconds
    duration = (float)(pulseIn(DISTANCE_SENSOR_ECHO_PIN, HIGH, pulseInTimeout_us));

    // Calculating the distance
    measured_distance = duration * 0.034321f / 2.0f;
    if (measured_distance <= 0.0f) 
    {
      measured_distance = 400.0f;
    }

    measured_distance = MIN(measured_distance, 400.0f);
    estimated_distance = movingAverage.next(measured_distance);

    lastSonarValue = estimated_distance;
    startMillis = currentMillis;
  }
  Serial.println(lastSonarValue);
  return lastSonarValue;
}