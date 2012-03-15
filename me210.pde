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

//
//	Constants
//

#define FWD_SPEED 150		// speed when headed to line
#define FWD_SPEED2 157	// speed while following line
#define SLOW_SPEED 100	// speed while looking for junctions

#define PIVOT_SPEED 155	// pivot turn speed

#define END_OF_LINE_TIME 300	// how long to go on forwards after detect end of line for seesaw line
#define END_OF_LINE_TIME_RIGHT 420	// how long to go on backwards after detecting end of seesaw line
#define TOKEN_DROP_TIME 1200	// how long to delay after token dispensor is done moving

#define PIVOT_LINE_CLEAR_TIME 500	// delay to wait for front sensors to clear the line during initial part of pivot turn

// direction while on seesaw line
#define S_LEFT 0	
#define S_RIGHT 1


static MainState state, nextState;	// next state used for helper states combinations (factored FSMs)
static char sDir, crossedMiddle;
static char outOfTokens;

void setup() {
	// init state machine
	state = STATE_START;
	outOfTokens = 0;

	// initialize modules
	globalTimeoutSetup();
	motorSetup();
	initializeServo();
	sensorSetup();
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

// main FSM
void loop() {
	static unsigned long time1, time2, time3;
	int val[3];
	char followLineRetVal;

	if (globalTimeout()) {
		state = STATE_IDLE;
	}

	// do FSM update
	switch(state) {
		case STATE_START:
			setMotion(0, SEESAW_HOME_TURN_SPD);
			state = STATE_FIND_SIDE_HOME1;
			break;

		// spin around to decide which side of play area we are on

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
					TMRArd_InitTimer(MAIN_TIMER, 20);
				}
				else {	// left side
					state = STATE_STARTED_LEFT1;

					setMotion(0,-PIVOT_SPEED);
					TMRArd_InitTimer(MAIN_TIMER, 530);
				}
			}
			break;

		// started on the left side, head to line and turn left

		case STATE_STARTED_LEFT1:
			if (TMRArd_IsTimerExpired(MAIN_TIMER) == TMRArd_EXPIRED) {
				stopRobot(STATE_STARTED_LEFT2);
			}
			break;

		case STATE_STARTED_LEFT2:
			setMotion(FWD_SPEED, -5);
			state = STATE_STARTED_LEFT3;
			break;

		case STATE_STARTED_LEFT3:
			readFrontSensors(val);
			if (hasLine(val)) {
				adjustMotion(-30, 0);
				state = STATE_STARTED_LEFT4;
			}
			break;

		case STATE_STARTED_LEFT4:
			if (readSideSensor()) {
				adjustMotion(10,0);	// hack to get kick in right direction
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
			startLineFollowing(FWD_SPEED2);
			state = STATE_FOLLOW_HLINE1;
			break;

		// started on the right side, head to line and turn right

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
				adjustMotion(-60, 0);
				state = STATE_STARTED_RIGHT4;
			}
			break;

		case STATE_STARTED_RIGHT4:
			if (readSideSensor()) {
				adjustMotion(10,0);	// hack to get kick in right direction
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
			startLineFollowing(FWD_SPEED2);
			state = STATE_FOLLOW_HLINE1;
			break;

		// follow the line from home to the T-junction

		case STATE_FOLLOW_HLINE1:
			if (followLine(FWD_SPEED2) != LINE_FOLLOW_OK) {
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
				stopRobot(STATE_FOLLOW_HLINE5);
			}
			break;

		case STATE_FOLLOW_HLINE5:
			startLineFollowing(FWD_SPEED2);
			state = STATE_FOLLOW_SLLINE1;
			sDir = S_LEFT;	
			crossedMiddle = 1;
			break;

		// follow the seesaw line, heading forwards (left)
		// deposit tokens if see beacon

		case STATE_FOLLOW_SLLINE0:
			startLineFollowing(FWD_SPEED2);
			state = STATE_FOLLOW_SLLINE0A;
			crossedMiddle = 0;
			break;

		case STATE_FOLLOW_SLLINE0A:
			readFrontSensors(val);
			if (hasLine(val))
				state = STATE_FOLLOW_SLLINE1;
			break;

		case STATE_FOLLOW_SLLINE1:
			if (readSideSeesaw() && !outOfTokens) {
				stopRobot(STATE_DISPENSE_TOKEN1);
				break;
			}

			followLineRetVal = followLine(FWD_SPEED2);	

			if (followLineRetVal == LINE_FOLLOW_NO_LINE) {
				TMRArd_InitTimer(MAIN_TIMER, END_OF_LINE_TIME); // continue for a while before going around 
				state = STATE_FOLLOW_SLLINE2;
				sDir = S_RIGHT;
				crossedMiddle = 0;
			}

			if (readSideSensor()) {
				crossedMiddle = 1;

				if (outOfTokens) {
					// go home. yipee!
					stopRobot(STATE_HEAD_HOME2);
				}
			}
			break;

		case STATE_FOLLOW_SLLINE2:
			if (readSideSeesaw() && !outOfTokens) {
				stopRobot(STATE_DISPENSE_TOKEN1);
				break;
			}

			if (TMRArd_IsTimerExpired(MAIN_TIMER) == TMRArd_EXPIRED) {
				stopRobot(STATE_FOLLOW_SRLINE0);
			}
			break;

		// follow the seesaw line, heading backwards (right)
		// deposit tokens if see beacon

		case STATE_FOLLOW_SRLINE0:
			startLineFollowing(-FWD_SPEED2);
			state = STATE_FOLLOW_SRLINE0A;
			crossedMiddle = 0;
			break;

		case STATE_FOLLOW_SRLINE0A:		// cover the case where we accidentally shoot off the line
			readBackSensors(val);
			if (hasLine(val))
				state = STATE_FOLLOW_SRLINE1;
			break;

		case STATE_FOLLOW_SRLINE1:
			if (readSideSeesaw() && !outOfTokens) {
				stopRobot(STATE_DISPENSE_TOKEN1);
				break;
			}

			followLineRetVal = followLine(-FWD_SPEED2);

			if (followLineRetVal == LINE_FOLLOW_NO_LINE) {
				TMRArd_InitTimer(MAIN_TIMER, END_OF_LINE_TIME_RIGHT); // continue for a while before going around 
				state = STATE_FOLLOW_SRLINE2;
				sDir = S_LEFT;
				crossedMiddle = 0;
			}
			
			if (readSideSensor()) {
				crossedMiddle = 1;

				if (outOfTokens) {
					// go home. yipee!
					stopRobot(STATE_HEAD_HOME2);
				}
			}
			break;

		case STATE_FOLLOW_SRLINE2:
			if (readSideSeesaw() && !outOfTokens) {
				stopRobot(STATE_DISPENSE_TOKEN1);
				break;
			}

			if (TMRArd_IsTimerExpired(MAIN_TIMER) == TMRArd_EXPIRED) {
				stopRobot(STATE_FOLLOW_SLLINE0);
			}
			break;


		// set of helper states to disponse tokens

		case STATE_DISPENSE_TOKEN1:
			depositTokens();			
			state = STATE_DISPENSE_TOKEN1A;

			if (bucketsLeft() == 0)
				outOfTokens = 1;
			break;

		case STATE_DISPENSE_TOKEN1A:
			if (!isServoDepositingTokens()) {
				TMRArd_InitTimer(MAIN_TIMER, TOKEN_DROP_TIME);
				state = STATE_DISPENSE_TOKEN2;
			}
			break;

		case STATE_DISPENSE_TOKEN2:
			if (TMRArd_IsTimerExpired(MAIN_TIMER) == TMRArd_EXPIRED) {
				if (readSideSeesaw() && !outOfTokens) {	// if seesaw not down yet dispense again
					state = STATE_DISPENSE_TOKEN1;
				}
				else if (sDir == S_LEFT) {		// was going left
					if (outOfTokens && crossedMiddle)
						state = STATE_FOLLOW_SRLINE0;
					else
						state = STATE_FOLLOW_SLLINE0;
				}
				else {	// was going right
					if (outOfTokens && crossedMiddle)
						state = STATE_FOLLOW_SLLINE0;
					else
						state = STATE_FOLLOW_SRLINE0;
				}
			}
			break;

		// start following to line to super-PAC

		case STATE_HEAD_HOME1:
			if (readSideSensor()) {
				stopRobot(STATE_HEAD_HOME2);
			}
			break;

		case STATE_HEAD_HOME2:
			setMotion(0, -PIVOT_SPEED);
			TMRArd_InitTimer(MAIN_TIMER, PIVOT_LINE_CLEAR_TIME);
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
			startLineFollowing(FWD_SPEED);
			goHome();
			state = STATE_HEAD_HOME6;
			break;

		case STATE_HEAD_HOME6:
			followLine(FWD_SPEED);

			if (testWallSensor()) {
				setMotion(255,0);
				state = STATE_WAIT_TOKEN_LOAD;
			}

			if (testTokensLoadedSensor()) {
				stopRobot(STATE_TOKEN_LOADED);
			}
			break;

		case STATE_WAIT_TOKEN_LOAD:
			adjustMotion(0,0);

			if (testTokensLoadedSensor())
				state = STATE_TOKEN_LOADED;
			break;

		case STATE_TOKEN_LOADED:
			startLineFollowing(-FWD_SPEED2);
			state = STATE_FOLLOW_B_HLINE1;
			outOfTokens = 0;
			break;

		// tokens loaded, head back from home line backwards

		case STATE_FOLLOW_B_HLINE1:
			if (followLine(-FWD_SPEED2) != LINE_FOLLOW_OK) {
				adjustMotion(-SLOW_SPEED, 0); // assume reached T
				state = STATE_FOLLOW_B_HLINE2;
			}
			break;

		case STATE_FOLLOW_B_HLINE2:
			if (readSideSensor()) {
				stopRobot(STATE_FOLLOW_B_HLINE3);
			}
			break;

		case STATE_FOLLOW_B_HLINE3:
			setMotion(0, PIVOT_SPEED);
			TMRArd_InitTimer(MAIN_TIMER, PIVOT_LINE_CLEAR_TIME);
			state = STATE_FOLLOW_B_HLINE4;
			break;

		case STATE_FOLLOW_B_HLINE4:
			if (TMRArd_IsTimerExpired(MAIN_TIMER) == TMRArd_EXPIRED)
				state = STATE_FOLLOW_HLINE4;
			break;

		// waiting for robot to stop moving (through stopMotion)

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

		// chillax, set motor speed to 0

		case STATE_IDLE:
			adjustMotion(0,0);
			break;
	}

	// call periodic functions
	updateServo();
	updateMotor();
}
