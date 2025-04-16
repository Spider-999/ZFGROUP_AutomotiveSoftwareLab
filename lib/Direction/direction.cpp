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
    digitalWrite(2, LOW);
    digitalWrite(4, LOW);

    // Set left turn rotation speed of the motors
    analogWrite(5, speed);
    analogWrite(6, speed);
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
    digitalWrite(4, HIGH);

    // Set right turn rotation speed of the motors
    analogWrite(5, speed);
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
    // Check the input dc speed
    dcspeed = checkPWM(dcspeed, LOWER_PWM_LIMIT, HIGHER_PWM_LIMIT);

    // Update the DC_PWM_value global variable
    DC_PWM_Value = dcspeed;

    // Choose the mode based on the user's option
    switch(mode)
    {
        case FORWARD:
            Move_Forward(dcspeed);
            break;
        case BACKWARD:
            Move_Backward(dcspeed);
            break;
        case LEFT:
            Rotate_Left(dcspeed);
            break;
        case RIGHT:
            Rotate_Right(dcspeed);
            break;
        default:
            Stop();
            break;
    }
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
    // Just a safety check. The DC_PWM_Value shouln't ever be
    // outside the pwm interval.
    DC_PWM_Value = checkPWM(DC_PWM_Value, LOWER_PWM_LIMIT, HIGHER_PWM_LIMIT);
    return DC_PWM_Value;
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
    // Reset the PWM value
    DC_PWM_Value = 0;

    // Increase the speed gradually until it reaches the target speed.
    do
    {
        DC_PWM_Value = getPwm() + 1;
        delay(20);
        setPwm(DC_PWM_Value, mode);
    } while (checkPWM(DC_PWM_Value, LOWER_PWM_LIMIT, targetSpeed) < targetSpeed);
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
int checkPWM(int number, int lower_limit, int upper_limit)
{
    // If the number is lower than the lower limit
    // return the lower limit number.
    if(number < lower_limit)
    {
        return lower_limit;
    }
    
    // If the number is higher than the upper limit
    // return the upper limit number.
    if(number > upper_limit)
    {
        return upper_limit;
    }

    // If the number is within the interval then return it.
    return number;
}
