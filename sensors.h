#pragma once

void readFrontSensors(int values[]);
void readBackSensors(int values[]);
char readSideSensor();
char readHomeBeacon();
char readFrontSeesaw();
char readSideSeesaw();

int removeMean(int val[]);
int getLinePos(int val[]);
