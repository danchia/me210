// Implement the game timeout,
// after which the robot should cease automatically.
#include "defines.h"

unsigned long startTime;

void globalTimeoutSetup() {
	startTime = millis();
}

char gameTimeout() {
	return millis() - startTime >= GAME_TIMEOUT_MSEC;
}
