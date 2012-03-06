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

					adjustMotion(0,PIVOT_SPEED);
					TMRArd_InitTimer(MAIN_TIMER, 450);
				}
				else {	// left side
					state = STATE_STARTED_LEFT1;

					setMotion(0,-PIVOT_SPEED);
					TMRArd_InitTimer(MAIN_TIMER, 400);
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
			if (hasLine(val)) {
				//TMRArd_InitTimer(MAIN_TIMER, 100);
				adjustMotion(100,0);
				state = STATE_STARTED_LEFT4;
			}
			break;

		case STATE_STARTED_LEFT4:
			//if (TMRArd_IsTimerExpired(MAIN_TIMER) == TMRArd_EXPIRED) {
			if (readSideSensor()) {
				adjustMotion(0, -PIVOT_SPEED);
				state = STATE_STARTED_LEFT5;
			}
			break;

		case STATE_STARTED_LEFT5:
			readFrontSensors(val);
			if (hasLine(val)) {
				startLineFollowing(FWD_SPEED);
				state = STATE_FOLLOW_HLINE1;
			}
			break;

		case STATE_STARTED_RIGHT1:
			if (TMRArd_IsTimerExpired(MAIN_TIMER) == TMRArd_EXPIRED) {
				stopRobot(STATE_STARTED_RIGHT2);
			}
			break;

		case STATE_STARTED_RIGHT2:
			setMotion(FWD_SPEED,0);
			state = STATE_STARTED_RIGHT3;
			break;

		case STATE_STARTED_RIGHT3:
			readFrontSensors(val);
			if (hasLine(val)) {
				//TMRArd_InitTimer(MAIN_TIMER, 100);
				adjustMotion(100,0);
				state = STATE_STARTED_RIGHT4;
			}
			break;

		case STATE_STARTED_RIGHT4:
			//if (TMRArd_IsTimerExpired(MAIN_TIMER) == TMRArd_EXPIRED) {
			if (readSideSensor()) {
				adjustMotion(0, PIVOT_SPEED);
				state = STATE_STARTED_RIGHT5;
			}
			break;

		case STATE_STARTED_RIGHT5:
			readFrontSensors(val);
			if (hasLine(val)) {
				startLineFollowing(FWD_SPEED);
				state = STATE_FOLLOW_HLINE1;
			}
			break;

		case STATE_FOLLOW_HLINE1:
			if (followLine(FWD_SPEED) != LINE_FOLLOW_OK) {
				adjustMotion(SLOW_SPEED, 0);	// assume reached T
				state = STATE_FOLLOW_HLINE2;
			}
			break;

		case STATE_FOLLOW_HLINE2:
			if (readSideSensor()) {
				stopRobot(STATE_FOLLOW_HLINE3);
			}
			break;

		case STATE_FOLLOW_HLINE3:
			setMotion(0, -PIVOT_SPEED);
			state = STATE_FOLLOW_HLINE4;
			break;

		case STATE_FOLLOW_HLINE4:
			readFrontSensors(val);
			if (hasLine(val)) {
				startLineFollowing(FWD_SPEED);
				state = STATE_FOLLOW_SLLINE1;
			}
			break;

		case STATE_FOLLOW_SLLINE1:
			if (followLine(FWD_SPEED) != LINE_FOLLOW_OK) {
				// U-turn, end of line
				state = STATE_IDLE;
			}

			if (readSideSeesaw()) {
				stopRobot(STATE_DISPENSE_TOKEN1);
			}
			break;

		case STATE_DISPENSE_TOKEN1:
			depositTokens();			
			TMRArd_InitTimer(MAIN_TIMER, 3000);
			state = STATE_DISPENSE_TOKEN2;
			break;

		case STATE_DISPENSE_TOKEN2:
			if (TMRArd_IsTimerExpired(MAIN_TIMER) == TMRArd_EXPIRED) {
				startLineFollowing(FWD_SPEED);
				state = STATE_FOLLOW_SLLINE1;
			}
			break;

		case STATE_STOPPING:
			if (motorDoneStop())
				state = nextState;
			break;

		case STATE_IDLE:
			setMotion(0,0);
			break;
	}

	// call periodic functions
	updateServo();
	updateMotor();
}
