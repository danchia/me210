#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif
#include "defines.h"

void readFrontSensors(int values[]) {
       values[0] = analogRead(TAPE_SENSOR_FRONT_LEFT);  
       values[1] = analogRead(TAPE_SENSOR_FRONT_CENTER); 
       values[2] = analogRead(TAPE_SENSOR_FRONT_RIGHT);  
}

void readBackSensors(int values[]) {
       values[0] = analogRead(TAPE_SENSOR_REAR_LEFT);  
       values[1] = analogRead(TAPE_SENSOR_REAR_CENTER);  
       values[2] = analogRead(TAPE_SENSOR_REAR_RIGHT); 
}

char readSideSensor() {
       return digitalRead(TAPE_SENSOR_SIDE) != 0;
}

char readHomeBeacon() {
       return digitalRead(HOME_BEACON_SENSOR) == 0;
}

char readFrontSeesaw() {
       return digitalRead(FRONT_SEESAW_SENSOR) == 0;
      
}

char readSideSeesaw() {
       return digitalRead(SIDE_SEESAW_SENSOR) == 0;
}




