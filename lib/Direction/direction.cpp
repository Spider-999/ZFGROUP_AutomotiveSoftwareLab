#include <Servo.h>
#include <Arduino.h>
#include "direction.h"
#include "sonar.h"

/**************************************************************************************************
                              FUNCTION INFO
NAME:
    setupMotors

DESCRIPTION:
    Setup the motors and an default PWM.

**************************************************************************************************/
void setupMotors()
{
  pinMode(2, OUTPUT); // left front/rear motor (direction)(port A)
  pinMode(5, OUTPUT); // PWM for port A
  pinMode(4, OUTPUT); // right front/rear motor (direction)(port B)
  pinMode(6, OUTPUT); // PWM for port B

  //viteza default
  DC_PWM_Value = 0;
}

/**************************************************************************************************
                              FUNCTION INFO
NAME:
    Move_Forward

DESCRIPTION:
    Move the car forward with a desired speed.

**************************************************************************************************/
void Move_Forward(int speed) 
{
    // Set motor spin direction to forward
    digitalWrite(2, HIGH);
    digitalWrite(4, LOW);

    // Set motor speed
    analogWrite(5, speed);
    analogWrite(6, speed);
}

/**************************************************************************************************
                              FUNCTION INFO
NAME:
    Move_Backward

DESCRIPTION:
    Move the car backward with a desired speed.

**************************************************************************************************/
void Move_Backward(int speed) 
{
    // Set motor spin direction to backward
    digitalWrite(2, LOW);
    digitalWrite(4, HIGH);

    // Set motor speed
    analogWrite(5, speed);
    analogWrite(6, speed);
}

/**************************************************************************************************
                              FUNCTION INFO
NAME:
    Stop

DESCRIPTION:
    Stop the motors from running.

**************************************************************************************************/
void Stop() 
{
    // Set no spin direction
    digitalWrite(2, LOW);
    digitalWrite(4, LOW);

    // Stop motors
    analogWrite(5, BRAKE_SPEED);
    analogWrite(6, BRAKE_SPEED);
}

/**************************************************************************************************
                              FUNCTION INFO
NAME:
    Rotate_Left

DESCRIPTION:
    Rotate the car to the left with a desired speed.

**************************************************************************************************/
void Rotate_Left(int speed) 
{
    // Set motor spin direction to forward
    digitalWrite(2, HIGH);
    digitalWrite(4, LOW);

    // Set left turn rotation speed of the motor
    analogWrite(5, speed);
    analogWrite(6, BRAKE_SPEED);
}

/**************************************************************************************************
                              FUNCTION INFO
NAME:
    Rotate_Right

DESCRIPTION:
    Rotate the car to the right with a desired speed.

**************************************************************************************************/
void Rotate_Right(int speed) 
{
    // Set motor spin direction to forward
    digitalWrite(2, HIGH);
    digitalWrite(4, LOW);

    // Set right turn rotation speed of the motor
    analogWrite(5, BRAKE_SPEED);
    analogWrite(6, speed);
}


/**************************************************************************************************
                              FUNCTION INFO
NAME:
    autonomousEmergencyBrake

DESCRIPTION:
    Handle the AEB functionality(stop at obstacle).

**************************************************************************************************/
void autonomousEmergencyBrake()
{

}

/**************************************************************************************************
                              FUNCTION INFO
NAME:
    setPwm

DESCRIPTION:
    Set the PWM(speed) and the direction of the car(FORWARD/BACKWARD/LEFT/RIGHT)

**************************************************************************************************/
void setPwm(uint8_t dcspeed, uint8_t mode)
{

}

/**************************************************************************************************
                              FUNCTION INFO
NAME:
    getPwm

DESCRIPTION:
    Get the current PWM.

**************************************************************************************************/
int getPwm()
{

}

/**************************************************************************************************
                              FUNCTION INFO
NAME:
    changeSpeed

DESCRIPTION:
    Change the speed.
    Parameters:
    dcspeed - desired speed 
    mode    - direction of the car (FORWARD/BACKWARD/LEFT/RIGHT)

**************************************************************************************************/
void changeSpeed(uint8_t targetSpeed, uint8_t mode)
{

}

/**************************************************************************************************
                              FUNCTION INFO
NAME:
    adaptive_cruise_control

DESCRIPTION:
    Controls the car's speed to maintain a safe distance from the obstacle in front (ACC - Adaptive Cruise Control).
    - If the obstacle is too close (obstacle <'min_distance'), AEB is activated.
    - If the obstacle is very far away (obstacle >'max_distance'), the maximum speed is used.
    - If the obstacle is somewhere between 'min_distance' and 'max_distance', the speed is calculated proportionally.

**************************************************************************************************/
void adaptive_cruise_control(uint8_t min_speed, uint8_t max_speed, uint8_t min_distance, uint16_t max_distance) 
{ 

}

/**************************************************************************************************
                              FUNCTION INFO
NAME:
    checkPWM

DESCRIPTION:
    Verify if the PWM it's within threshold.

**************************************************************************************************/
int checkPWM(uint8_t number, int lower_limit, int upper_limit)
{

}
