#ifndef __DIROECTION_H__
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
*   VARIABLE DECLARATIONS
*
**************************************************************************************************/

static int DC_PWM_Value;
static float distance;

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

#endif