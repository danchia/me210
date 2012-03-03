// Main program file
// Controls initialization, as well as the high-level FSM controlling the robot.
#include "globalTimeout.h"
#include <Servo.h>
#include <Timers.h>

void setup() {
	// initialize modules
	globalTimeoutSetup();

	// initialize serial
	Serial.init(57600);
	Serial.println("Initialized");
}

void loop() {
}
