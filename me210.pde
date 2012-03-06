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
	//startLineFollowing(190);
}
void loop() {
	setMotion(0, 210);
	while(!readFrontSeesaw());
	unsigned long startTime = millis();
	//Serial.println("Seesaw");
	while(!readHomeBeacon());
	unsigned long endTime = millis() - startTime;
	//Serial.println("Home");
	//Serial.println(endTime);
	if (endTime > 1100)
		digitalWrite(13, HIGH);

	setMotion(0,0);
	while(1);
}
