// Main program file
// Controls initialization, as well as the high-level FSM controlling the robot.
#include "defines.h"
#include "globalTimeout.h"
#include "sensors.h"
#include "motor.h"
#include "lineMotions.h"
#include "tokenManagement.h"
#include "states.h"

#include <Servo.h>
#include <Timers.h>

#define FWD_SPEED 210
#define SLOW_SPEED 115

#define PIVOT_SPEED 185

static MainState state, nextState;	// next state used for helper states combinations (factored FSMs)

void setup() {
	// init state machine
	state = STATE_START;

	// initialize modules
	globalTimeoutSetup();
	motorSetup();
	initializeServo();
	pinMode(LED_PIN, OUTPUT);

	// initialize serial
	Serial.begin(57600);
	Serial.println("Initialized");
}

// stops the robot, and when done transition to state next
void stopRobot(MainState next) {
	state = STATE_STOPPING;
	stopMotion();
	nextState = next;
}

void loop() {
	static unsigned long time1, time2, time3;
	int val[3];

	// do FSM update
	switch(state) {
		case STATE_START:
			setMotion(0, SEESAW_HOME_TURN_SPD);
			state = STATE_FIND_SIDE_HOME1;
			break;

		case STATE_FIND_SIDE_HOME1:
			if (readHomeBeacon()) {
				time1 = millis();
				state = STATE_FIND_SIDE_SEESAW;
			}
			break;
			
		case STATE_FIND_SIDE_SEESAW:
			if (readFrontSeesaw()) {
				time2 = millis();
				state = STATE_FIND_SIDE_HOME2;
			}
			break;

		case STATE_FIND_SIDE_HOME2:
			if (readHomeBeacon()) {
				time3 = millis();

				// check to see which side we're on
				if (time2 - time1 < time3 - time2) { 	// right side
					state = STATE_STARTED_RIGHT1;
					digitalWrite(LED_PIN, HIGH);

					//setMotion(0,PIVOT_SPEED);
					setMotion(0,0);
					TMRArd_InitTimer(MAIN_TIMER, 100);
				}
				else {	// left side
					state = STATE_STARTED_LEFT1;

					setMotion(0,-PIVOT_SPEED);
					TMRArd_InitTimer(MAIN_TIMER, 100);
				}
			}
			break;

		case STATE_STARTED_LEFT1:
			if (TMRArd_IsTimerExpired(MAIN_TIMER) == TMRArd_EXPIRED) {
				stopRobot(STATE_STARTED_LEFT2);
			}
			break;

		case STATE_STARTED_LEFT2:
			setMotion(FWD_SPEED,0);
			state = STATE_STARTED_LEFT3;
			break;

		case STATE_STARTED_LEFT3:
			readFrontSensors(val);
			if (hasLine(val))
				TMRArd_InitTimer(MAIN_TIMER, 100);
				state = STATE_STARTED_LEFT4;
			break;

		case STATE_STARTED_LEFT4:
			if (TMRArd_IsTimerExpired(MAIN_TIMER) == TMRArd_EXPIRED) {
				adjustMotion(50, -140);
				state = STATE_STARTED_LEFT5;
			}
			break;

		case STATE_STARTED_LEFT5:
			readFrontSensors(val);
			if (hasLine(val)) {
				// start line following
			}
			break;

		case STATE_STOPPING:
			if (motorDoneStop())
				state = nextState;
			break;
	}

	// call periodic functions
	updateServo();
	updateMotor();
}
