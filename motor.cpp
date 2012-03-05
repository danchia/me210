#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include "defines.h"
#include "motor.h"

void motorSetup() {
	pinMode(MOTOR_LEFT_DIR, OUTPUT);
	pinMode(MOTOR_LEFT_PWM, OUTPUT);
	pinMode(MOTOR_RIGHT_DIR, OUTPUT);
	pinMode(MOTOR_RIGHT_PWM, OUTPUT);

	setMotion(0,0);
}

void setMotion(int fwdSpeed, int rotSpeed) {
	int leftSpeed = fwdSpeed + rotSpeed;
	int rightSpeed = fwdSpeed - rotSpeed;

	digitalWrite(MOTOR_LEFT_DIR, (leftSpeed > 0) ? LOW : HIGH);
	digitalWrite(MOTOR_RIGHT_DIR, (rightSpeed < 0) ? LOW : HIGH);

	leftSpeed = abs(leftSpeed);
	rightSpeed = abs(rightSpeed);

	analogWrite(MOTOR_LEFT_PWM, (leftSpeed > 255) ? 255 : leftSpeed);
	analogWrite(MOTOR_RIGHT_PWM, (rightSpeed > 255) ? 255 : rightSpeed);
}
