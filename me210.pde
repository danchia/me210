// Main program file
// Controls initialization, as well as the high-level FSM controlling the robot.
#include "defines.h"
#include "globalTimeout.h"
#include "sensors.h"
#include "motor.h"
#include "lineMotions.h"
#include "tokenManagement.h"

#include <Servo.h>
#include <Timers.h>

#define FWD_SPEED 210
#define SLOW_SPEED 180

#define TURN_SPEED 170

void setup() {
	// initialize modules
	globalTimeoutSeup();
	motorSetup();
	initializeServo();
	pinMode(LED_PIN, OUTPUT);

	// initialize serial
	Serial.begin(57600);
	Serial.println("Initialized");

}

// function for determining side of board
// use blocking code for now
// returns 1 if right side
// need to recalibrate
char findSide() {
	setMotion(0, SEESAW_HOME_TURN_SPD);

	while(!readFrontSeesaw());
	unsigned long startTime = millis();
	//Serial.println("Seesaw");
	while(!readHomeBeacon());
	unsigned long endTime = millis() - startTime;
	//Serial.println("Home");
	//Serial.println(endTime);
	setMotion(0,0);

	if (endTime > SEESAW_HOME_TIME_THRES) {
		digitalWrite(13, HIGH);
		return 1;
	}
	else
		return 0;
}

void loop() {
	for (int i = 0; i < 50; i++) {
		updateServo();
		delay(15);
	}

	// test line following
	startLineFollowing(FWD_SPEED);

	while(followLine(FWD_SPEED) == LINE_FOLLOW_OK) updateMotor();	// follow line
	setMotion(SLOW_SPEED,0);
	while(!readSideSensor()) updateMotor();	// wait for turn sensor
	stopMotion();
	while(!motorDoneStop()) updateMotor();

	//turn!
	setMotion(0, -TURN_SPEED);
	int val[3];
	do {
		readFrontSensors(val);
		updateMotor();
	}while(val[0] < LINE_SENSOR_MIN_THRES);

	stopMotion();
	while(!motorDoneStop()) updateMotor();

	startLineFollowing(FWD_SPEED);

	char token = 0;

	while(followLine(FWD_SPEED) == LINE_FOLLOW_OK) {
		if (readSideSeesaw() && !token) {
			stopMotion();
			while(!motorDoneStop()) updateMotor();

			depositTokens();
			for (int i = 0; i < 200; i++) {
				updateMotor();
				updateServo();
				delay(15);
			}
			
			token = 1;
			startLineFollowing(FWD_SPEED);
		}

		updateServo();
	}
	stopMotion();
	while(!motorDoneStop()) updateMotor();

	while(1) updateMotor();
}
