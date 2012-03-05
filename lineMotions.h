#pragma once

#define LINE_FOLLOW_OK 0
#define LINE_FOLLOW_ALL_LINE 1
#define LINE_FOLLOW_NO_LINE 2

void startLineFollowing(int spd);
char followLine(int spd);
