
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

int readSideSensor() {
       return digitalRead(TAPE_SENSOR_SIDE);  
      
}

int readHomeBeacon() {
       return digitalRead(HOME_BEACON_SENSOR);  
      
}

int readFrontSeesaw() {
       return digitalRead(FRONT_SEESAW_SENSOR);  
      
}

int readSideSeesaw() {
       return digitalRead(SIDE_SEESAW_SENSOR);  
      
}




