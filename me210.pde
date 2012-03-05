// Main program file
// Controls initialization, as well as the high-level FSM controlling the robot.
#include "defines.h"
#include "globalTimeout.h"
#include "sensors.h"
#include "motor.h"

#include <Servo.h>
#include <Timers.h>

void setup() {
	// initialize modules
	globalTimeoutSetup();
	motorSetup();

	// initialize serial
	Serial.begin(57600);
	Serial.println("Initialized");
}
void loop() {
	static int oldError = 0;
	int val[3];
	readBackSensors(val);
	removeMin(val);

	int error = getLinePos(val);

	int correction = (getLinePos(val) >> 1) + ((error - oldError) << 3);

//	Serial.print(val[0]);
//	Serial.print(" ");
//	Serial.print(val[1]);
//	Serial.print(" ");
//	Serial.print(val[2]);
//	Serial.print(" ");
//	Serial.print(correction);
//	Serial.println("");


	//setMotion(190, -correction);
	//setMotion(-190, -correction);

	oldError = error;

	delay(2);
}
