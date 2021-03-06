#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif
#include "defines.h"

void sensorSetup() {
  digitalWrite(WALL_SENSOR,HIGH); //Pull up the wall sensor to be on by default...
  digitalWrite(TOKENS_LOADED,HIGH); //Pull up token toggle
}

char testWallSensor(void) {
  return digitalRead(WALL_SENSOR) == LOW;
}

char testTokensLoadedSensor(void) {
  return digitalRead(TOKENS_LOADED) == LOW;
}

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
       return digitalRead(TAPE_SENSOR_SIDE) == LOW;
}

char readHomeBeacon() {
       return digitalRead(HOME_BEACON_SENSOR) == LOW;
}

char readFrontSeesaw() {
       return digitalRead(FRONT_SEESAW_SENSOR) == LOW;
      
}

char readSideSeesaw() {
       return digitalRead(SIDE_SEESAW_SENSOR) == LOW;
}

int removeMin(int val[]) {
	int min = val[0];
	
	if (val[1] < min)
		min = val[1];
	if (val[2] < min)
		min = val[2];

	val[0] -= min;
	val[1] -= min;
	val[2] -= min;

	return min;
}

int getLinePos(int val[]) {
	int num = -val[0] + val[2];
	long denom = val[0] + val[1] + val[2];

	return (int)(((long)num * LINE_POS_SCALE) / denom);
}

char hasLine(int rawVal[]) {
	if (rawVal[0] > LINE_SENSOR_MIN_THRES
			|| rawVal[1] > LINE_SENSOR_MIN_THRES
			|| rawVal[2] > LINE_SENSOR_MIN_THRES)
		return 1;
	
	return 0;
}
