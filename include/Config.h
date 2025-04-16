#ifndef __CONFIG_H__
#define __CONFIG_H__

/**************************************************************************************************
*
*   INCLUDES
*
**************************************************************************************************/

#include <Servo.h>
#include "direction.h"
#include "sonar.h"
#include <Arduino.h>

/**************************************************************************************************
*
*   VARIABLE DECLARATIONS
*
**************************************************************************************************/

extern bool showMenu = 0;
static int option;
extern bool bluetooth_flag = 0;
String BLE_val;

/**************************************************************************************************
                              FUNCTION INFO
NAME:
    Print_Menu

DESCRIPTION:
    Print the menu in terminal for being able to select the functionality for the car.

**************************************************************************************************/
void Print_Menu()
{
  Serial.println("Enter command:");
  Serial.println("0 - Show Menu");
  Serial.println("1 - Move Forward");
  Serial.println("2 - Move Backwards");
  Serial.println("3 - Stop");
  Serial.println("4 - Turn Left");
  Serial.println("5 - Turn Right");
  showMenu = 0;
}

/**************************************************************************************************
                              FUNCTION INFO
NAME:
    Print_Menu

DESCRIPTION:
    Select the menu in terminal for being able to select the functionality for the car.

**************************************************************************************************/
void Select_Menu()
{ 
  if(Serial.available() > 0)
  {
    char input = Serial.read();  // Read one character
    switch (input) {
      case '0':
        option = 0;
        showMenu = 1;
        break;
      case '1':
        option = 1;
        break;
      case '2':
        option = 2;
        break;
      case '3':
        option = 3;
        break;
      case '4':
        option = 4;
        break;
      case '5':
        option = 5;
        break;
      default:
        Serial.println("Unknown command!");
      break;
    }
  }

  // Goes to the selected option of the operator
  switch(option)
  {
    case 0:
      if(showMenu == 1)
      {
        Stop();
        Print_Menu();
      }
      break;
    case 1:
      Move_Forward(128);
      break;
    case 2:
      Move_Backward(128);
      break;
    case 3:
      Stop();
      break;
    case 4:
      Rotate_Left(128);
      break;
    case 5:
      Rotate_Right(128);
      break;
    default:
      Stop();
      Serial.println("Unknown command!");
      break;
  }
}
#endif