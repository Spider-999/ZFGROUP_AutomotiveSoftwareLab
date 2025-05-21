#include <Config.h>

void setup()
{
  Serial.begin(9600);
  BLE_val = "";

  setupSonar();
  setupMotors();
  setupLineTracking();
  initializePID();
  
  Print_Menu();
}

void loop()
{
  Select_Menu();
}