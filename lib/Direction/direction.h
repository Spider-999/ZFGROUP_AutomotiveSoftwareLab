#ifndef __DIRECTION_H__
#define __DIRECTION_H__

/**************************************************************************************************
*
*   INCLUDES
*
**************************************************************************************************/

#include <Arduino.h>

/**************************************************************************************************
*
*   DEFINES
*
**************************************************************************************************/

#define FORWARD 1
#define BACKWARD 2
#define LEFT 3
#define RIGHT 4
#define AEB_THRESHOLD 15
#define BRAKE_SPEED 0
#define LOWER_PWM_LIMIT 1
#define HIGHER_PWM_LIMIT 255

/**************************************************************************************************
*
*   STRUCTURES
*
**************************************************************************************************/

typedef struct PID
{
  float Kp;
  float Ki;
  float Kd;
  float currentError;
  float lastError;
  float derivativeError;
  float integralError = 0.0f;
  float speedControlSignal;
  unsigned int lastTime;
  unsigned int samplingTime;
} PID;

/**************************************************************************************************
*
*   VARIABLE DECLARATIONS
*
**************************************************************************************************/

static int DC_PWM_Value;
static float distance;
static long int previousTime = 0;
static PID pid;


/**************************************************************************************************
*
*   FUNCTION DECLARATIONS
*
**************************************************************************************************/

void    setupMotors();
void    Move_Forward(int speed);
void    Move_Backward(int speed);
void    Stop();
void    Rotate_Left(int speed);
void    Rotate_Right(int speed);
void    autonomousEmergencyBrake();
void    setPwm(uint8_t, uint8_t);
int     getPwm();
void    adaptive_cruise_control(uint8_t, uint8_t, uint8_t, uint16_t);
int     checkPWM(int, int, int);
void    changeSpeed(uint8_t, uint8_t);
void    initializePID();

#endif