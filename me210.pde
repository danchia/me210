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

#define FWD_SPEED 180
#define SLOW_SPEED 110

#define TURN_SPEED 160

void setup() {
	// initialize modules
	globalTimeoutSetup();
	motorSetup();
	initializeServo();
	pinMode(13, OUTPUT);

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
	for (int i = 0; i < 200; i++) {
		updateServo();
		delay(15);
	}

	// test line following
	startLineFollowing(FWD_SPEED);

	while(followLine(FWD_SPEED) == LINE_FOLLOW_OK);	// follow line
	setMotion(SLOW_SPEED,0);
	while(!readSideSensor());	// wait for turn sensor
	setMotion(-255,0);
	delay(60);
	setMotion(0,0);
	delay(60);
	//jerk stop
	//turn!
	setMotion(0, -TURN_SPEED);
	int val[3];
	do {
		readFrontSensors(val);
	}while(val[0] < LINE_SENSOR_MIN_THRES);
	setMotion(0, 255);
	delay(70);
	setMotion(0,0);
	delay(70);
	startLineFollowing(FWD_SPEED);

	char token = 0;

	while(followLine(FWD_SPEED) == LINE_FOLLOW_OK) {
		if (readSideSeesaw() && !token) {
			setMotion(-255,0);
			delay(40);
			setMotion(0,0);
			delay(50);
			depositTokens();
			for (int i = 0; i < 200; i++) {
				updateServo();
				delay(15);
			}
			
			token = 1;
			startLineFollowing(FWD_SPEED);
		}

		updateServo();
	}
	setMotion(0, 0);
	while(1);
}
