#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include "defines.h"
#include "motor.h"
#include "Timers.h"

static int leftSpeed, rightSpeed;

void motorSetup() {
	pinMode(MOTOR_LEFT_DIR, OUTPUT);
	pinMode(MOTOR_LEFT_PWM, OUTPUT);
	pinMode(MOTOR_RIGHT_DIR, OUTPUT);
	pinMode(MOTOR_RIGHT_PWM, OUTPUT);

	// set the motors as idle first
	leftSpeed = 0;
	rightSpeed = 0;

	analogWrite(MOTOR_LEFT_PWM, 0);
	analogWrite(MOTOR_RIGHT_PWM, 0);
}

void setMotion(int fwdSpeed, int rotSpeed) {
	// calculate individual motor speed
	leftSpeed = fwdSpeed + rotSpeed;
	rightSpeed = fwdSpeed - rotSpeed;

	// set the direction
	digitalWrite(MOTOR_LEFT_DIR, (leftSpeed > 0) ? LOW : HIGH);
	digitalWrite(MOTOR_RIGHT_DIR, (rightSpeed < 0) ? LOW : HIGH);

	// cap the speed
	if (leftSpeed < -255)
		leftSpeed = -255;
	else if (leftSpeed > 255)
		leftSpeed = 255;
	if (rightSpeed < -255)
		rightSpeed = -255;
	else if (rightSpeed > 255)
		rightSpeed = 255;


	// set it to get full power for a fixed time
	analogWrite(MOTOR_LEFT_PWM, 255);
	analogWrite(MOTOR_RIGHT_PWM, 255);

	// set a timer so we know when to revert to actual level
	TMRArd_InitTimer(MOTOR_TIMER, MOTOR_FULL_POWER_TIME);
}

void adjustMotion(int fwdSpeed, int rotSpeed) {
	// calculate individual motor speed
	leftSpeed = fwdSpeed + rotSpeed;
	rightSpeed = fwdSpeed - rotSpeed;

	// set the direction
	digitalWrite(MOTOR_LEFT_DIR, (leftSpeed > 0) ? LOW : HIGH);
	digitalWrite(MOTOR_RIGHT_DIR, (rightSpeed < 0) ? LOW : HIGH);

	// cap the speed
	if (leftSpeed < -255)
		leftSpeed = -255;
	else if (leftSpeed > 255)
		leftSpeed = 255;
	if (rightSpeed < -255)
		rightSpeed = -255;
	else if (rightSpeed > 255)
		rightSpeed = 255;
}

// called periodically by main code to update motor state
void updateMotor() {
	if (TMRArd_IsTimerExpired(MOTOR_TIMER) == TMRArd_EXPIRED) {
		// set the motor values to the actual power levels
		analogWrite(MOTOR_LEFT_PWM, abs(leftSpeed));
		analogWrite(MOTOR_RIGHT_PWM, abs(rightSpeed));
	}
}

char motorDoneStop() {
	return TMRArd_IsTimerExpired(MOTOR_TIMER) == TMRArd_EXPIRED;
}

void stopMotion() {
	// reverse direction for short time
	digitalWrite(MOTOR_LEFT_DIR, (leftSpeed < 0) ? LOW : HIGH);
	digitalWrite(MOTOR_RIGHT_DIR, (rightSpeed > 0) ? LOW : HIGH);

	analogWrite(MOTOR_LEFT_PWM, 255);
	analogWrite(MOTOR_RIGHT_PWM, 255);

	leftSpeed = 0;
	rightSpeed = 0;

	// set stop timer
	TMRArd_InitTimer(MOTOR_TIMER, MOTOR_STOP_TIME);
}
