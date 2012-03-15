// implements line following, true the use of a PD controller

#pragma once

#define LINE_FOLLOW_OK 0
#define LINE_FOLLOW_ALL_LINE 1
#define LINE_FOLLOW_NO_LINE 2

// call the first time starting line following
void startLineFollowing(int spd);

// after starting line following, this should be called often (500Hz or more)
// calculates a correction factor from sensor reading
// if spd is negative, switches to rear set of sensors
//
// returns a code indicating status of line following
char followLine(int spd);
