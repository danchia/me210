// Main program file
// Controls initialization, as well as the high-level FSM controlling the robot.
#include "defines.h"
#include "globalTimeout.h"
#include "sensors.h"
#include "motor.h"
#include "lineMotions.h"

#include <Servo.h>
#include <Timers.h>

void setup() {
	// initialize modules
	globalTimeoutSetup();
	motorSetup();
	pinMode(13, OUTPUT);

	// initialize serial
	Serial.begin(57600);
	Serial.println("Initialized");

	// test line following
	startLineFollowing(190);
}
void loop() {
//	digitalWrite(13, (readSideSensor() ? HIGH: LOW));
	followLine(190);
}
