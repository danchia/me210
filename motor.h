// set of high-level motor control functions
// implement a mini-state machine to give the motors a starting full power kick
// setMotion starts a motion (gives kick, then reduces power)
// adjustMotion changes the desired motor power
// updateMotor should be called often to update the motor power (because of the state machine)
//
// stopMotion implements backward kick to make sure the robot stops
#pragma once

void motorSetup();

// rot is positive clockwise
void setMotion(int fwdSpeed, int rotSpeed);
void adjustMotion(int fwdSpeed, int rotSpeed);
void updateMotor();
char motorDoneStop();
void stopMotion();
