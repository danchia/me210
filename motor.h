#pragma once

void motorSetup();

// rot is positive clockwise
void setMotion(int fwdSpeed, int rotSpeed);
void adjustMotion(int fwdSpeed, int rotSpeed);
void updateMotor();
char motorDoneStop();
void stopMotion();
