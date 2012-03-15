// functions to help extract, and process sensor inputs
// if return values are digital, high means active (sensed)

#pragma once

// one time setup call, sets pin directions, etc
void sensorSetup();

// check front bumper
char testWallSensor(void);

// check switch on robot used to signal done loading
char testTokensLoadedSensor(void);

// reads the set of three sensors in the front of the robot
void readFrontSensors(int values[]);

// read the set of three sensors at the back of the robot
void readBackSensors(int values[]);

// reads the side tape sensor, used to detect turning point on line
char readSideSensor();

// reads the home beacon sensor
char readHomeBeacon();

// reads the front seesaw beacon sensor
char readFrontSeesaw();

// reads the side seesaw beacon sensor
char readSideSeesaw();

//
// collection of functions to help process line sensor values (set of three)
//

// finds the minimum value of the three, and removes it from all readings
int removeMin(int val[]);
// does a weighted average to find the line position (0 is center)
int getLinePos(int val[]);
// checks whether a set of values has a line, by simple thresholding
char hasLine(int rawVal[]);
