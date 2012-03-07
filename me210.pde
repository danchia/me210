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

#define FWD_SPEED 150
#define SLOW_SPEED 90

#define PIVOT_SPEED 140

#define END_OF_LINE_TIME 300
#define TOKEN_DROP_TIME 2500

static MainState state, nextState;	// next state used for helper states combinations (factored FSMs)
static char sDir;
static char outOfTokens;

void setup() {
	// init state machine
	state = STATE_START;
	outOfTokens = 0;

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
	state = STATE_STOPPING1;
	stopMotion();
	nextState = next;
}

#define S_LEFT 0
#define S_RIGHT 1

void loop() {
	static unsigned long time1, time2, time3;
	int val[3];
	char followLineRetVal;

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
					TMRArd_InitTimer(MAIN_TIMER, 40);
				}
				else {	// left side
					state = STATE_STARTED_LEFT1;

					setMotion(0,-PIVOT_SPEED);
					TMRArd_InitTimer(MAIN_TIMER, 520);
				}
			}
			break;

		case STATE_STARTED_LEFT1:
			if (TMRArd_IsTimerExpired(MAIN_TIMER) == TMRArd_EXPIRED) {
				stopRobot(STATE_STARTED_LEFT2);
			}
			break;

		case STATE_STARTED_LEFT2:
			setMotion(FWD_SPEED, -4);
			state = STATE_STARTED_LEFT3;
			break;

		case STATE_STARTED_LEFT3:
			readFrontSensors(val);
			if (hasLine(val)) {
				//TMRArd_InitTimer(MAIN_TIMER, 100);
				adjustMotion(30, 0);
				state = STATE_STARTED_LEFT4;
			}
			break;

		case STATE_STARTED_LEFT4:
			//if (TMRArd_IsTimerExpired(MAIN_TIMER) == TMRArd_EXPIRED) {
			if (readSideSensor()) {
				stopRobot(STATE_STARTED_LEFT4A);
			}
			break;

		case STATE_STARTED_LEFT4A:
			setMotion(0, -PIVOT_SPEED);
			state = STATE_STARTED_LEFT5;
			break;

		case STATE_STARTED_LEFT5:
			readFrontSensors(val);
			if (hasLine(val)) {
				stopRobot(STATE_STARTED_LEFT6);
			}
			break;

		case STATE_STARTED_LEFT6:
			startLineFollowing(FWD_SPEED);
			state = STATE_FOLLOW_HLINE1;
			break;

		case STATE_STARTED_RIGHT1:
			if (TMRArd_IsTimerExpired(MAIN_TIMER) == TMRArd_EXPIRED) {
				stopRobot(STATE_STARTED_RIGHT2);
			}
			break;

		case STATE_STARTED_RIGHT2:
			setMotion(FWD_SPEED, -3);
			state = STATE_STARTED_RIGHT3;
			break;

		case STATE_STARTED_RIGHT3:
			readFrontSensors(val);
			if (hasLine(val)) {
				//TMRArd_InitTimer(MAIN_TIMER, 100);
				adjustMotion(30, 0);
				state = STATE_STARTED_RIGHT4;
			}
			break;

		case STATE_STARTED_RIGHT4:
			//if (TMRArd_IsTimerExpired(MAIN_TIMER) == TMRArd_EXPIRED) {
			if (readSideSensor()) {
				stopRobot(STATE_STARTED_RIGHT4A);
			}
			break;

		case STATE_STARTED_RIGHT4A:
			setMotion(0, PIVOT_SPEED);
			state = STATE_STARTED_RIGHT5;
			break;

		case STATE_STARTED_RIGHT5:
			readFrontSensors(val);
			if (hasLine(val)) {
				stopRobot(STATE_STARTED_RIGHT6);
			}
			break;

		case STATE_STARTED_RIGHT6:
			startLineFollowing(FWD_SPEED);
			state = STATE_FOLLOW_HLINE1;
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
				sDir = S_LEFT;	
			}
			break;

		case STATE_FOLLOW_SLLINE0:
			startLineFollowing(FWD_SPEED);
			state = STATE_FOLLOW_SLLINE1;
			break;

		case STATE_FOLLOW_SLLINE1:
			if (readSideSeesaw() && !outOfTokens) {
				stopRobot(STATE_DISPENSE_TOKEN1);
			}

			followLineRetVal = followLine(FWD_SPEED);	

			if (followLineRetVal == LINE_FOLLOW_NO_LINE) {
				TMRArd_InitTimer(MAIN_TIMER, END_OF_LINE_TIME); // continue for a while before going around 
				state = STATE_FOLLOW_SLLINE2;
			}
			else if (outOfTokens && followLineRetVal == LINE_FOLLOW_ALL_LINE) {
				// go home. yipee!
				adjustMotion(SLOW_SPEED, 0);
				state = STATE_HEAD_HOME1;
			}
			break;

		case STATE_FOLLOW_SLLINE2:
			if (readSideSeesaw()) {
				stopRobot(STATE_DISPENSE_TOKEN1);
			}

			if (TMRArd_IsTimerExpired(MAIN_TIMER) == TMRArd_EXPIRED) {
				stopRobot(STATE_FOLLOW_SRLINE0);
				sDir = S_RIGHT;
			}
			break;

		case STATE_FOLLOW_SRLINE0:
			startLineFollowing(-FWD_SPEED);
			state = STATE_FOLLOW_SRLINE1;
			break;

		case STATE_FOLLOW_SRLINE1:
			if (readSideSeesaw() && outOfTokens) {
				stopRobot(STATE_DISPENSE_TOKEN1);
			}

			followLineRetVal = followLine(-FWD_SPEED);

			if (followLineRetVal == LINE_FOLLOW_NO_LINE) {
				TMRArd_InitTimer(MAIN_TIMER, END_OF_LINE_TIME); // continue for a while before going around 
				state = STATE_FOLLOW_SRLINE2;
			}
			else if (outOfTokens && followLineRetVal == LINE_FOLLOW_ALL_LINE) {
				// go home. yipee!
				adjustMotion(-SLOW_SPEED, 0);
				state = STATE_HEAD_HOME1;
			}
			break;

		case STATE_FOLLOW_SRLINE2:
			if (readSideSeesaw()) {
				stopRobot(STATE_DISPENSE_TOKEN1);
			}

			if (TMRArd_IsTimerExpired(MAIN_TIMER) == TMRArd_EXPIRED) {
				stopRobot(STATE_FOLLOW_SLLINE0);
				sDir = S_LEFT;
			}

			break;

		case STATE_DISPENSE_TOKEN1:
			depositTokens();			
			TMRArd_InitTimer(MAIN_TIMER, TOKEN_DROP_TIME);
			state = STATE_DISPENSE_TOKEN2;

			if (bucketsLeft() == 0)
				outOfTokens = 1;
			break;

		case STATE_DISPENSE_TOKEN2:
			if (TMRArd_IsTimerExpired(MAIN_TIMER) == TMRArd_EXPIRED) {
				if (sDir == S_LEFT)
					state = STATE_FOLLOW_SLLINE0;
				else
					state = STATE_FOLLOW_SRLINE0;
			}
			break;

		case STATE_HEAD_HOME1:
			if (readSideSensor()) {
				stopRobot(STATE_HEAD_HOME2);
			}
			break;

		case STATE_HEAD_HOME2:
			setMotion(0, -PIVOT_SPEED);
			TMRArd_InitTimer(MAIN_TIMER, 200);
			state = STATE_HEAD_HOME3;
			break;

		case STATE_HEAD_HOME3:
			if (TMRArd_IsTimerExpired(MAIN_TIMER) == TMRArd_EXPIRED)
				state = STATE_HEAD_HOME4;
			break;

		case STATE_HEAD_HOME4:
			readFrontSensors(val);
			if (hasLine(val))
				stopRobot(STATE_HEAD_HOME5);
			break;
		
		case STATE_HEAD_HOME5:
			setMotion(FWD_SPEED, 0);
			goHome();
			state = STATE_HEAD_HOME6;
			break;

		case STATE_HEAD_HOME6:
			if (testWallSensor()) {
				stopRobot(STATE_WAIT_TOKEN_LOAD);
			}

			if (testTokensLoadedSensor()) {
				stopRobot(STATE_TOKEN_LOADED);
			}
			break;

		case STATE_WAIT_TOKEN_LOAD:
			if (testTokensLoadedSensor())
				state = STATE_TOKEN_LOADED;
			break;

		case STATE_TOKEN_LOADED:
			// TODO...
			break;

		case STATE_STOPPING1:
			if (motorDoneStop()) {
				TMRArd_InitTimer(MAIN_TIMER, STOP_PAUSE_TIME);
				state = STATE_STOPPING2;
			}
			break;

		case STATE_STOPPING2:
			if (TMRArd_IsTimerExpired(MAIN_TIMER) == TMRArd_EXPIRED)
				state = nextState;
			break;

		case STATE_IDLE:
			adjustMotion(0,0);
			break;
	}

	// call periodic functions
	updateServo();
	updateMotor();
}
