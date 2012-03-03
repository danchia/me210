// Implement the game timeout,
// after which the robot should cease automatically.
#include "defines.h"
#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

unsigned long startTime;

void globalTimeoutSetup() {
	startTime = millis();
}

char globalTimeout() {
	return millis() - startTime >= GAME_TIMEOUT_MSEC;
}
