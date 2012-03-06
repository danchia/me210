#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include "lineMotions.h"
#include "sensors.h"
#include "motor.h"
#include "Timers.h"
#include "defines.h"

static int oldError;

void startLineFollowing(int spd) {
	oldError = 0;
	setMotion(spd, 0);
	followLine(spd);
	TMRArd_InitTimer(LINE_FOLLOW_TIMER, LINE_FOLLOW_UPDATE_PERIOD);
}

static int getMax(int val[]) {
	int max = val[0];
	if (max < val[1])
		max = val[1];
	if (max < val[2])
		max = val[2];

	return max;
}

char followLine(int spd) {
	if (TMRArd_IsTimerExpired(LINE_FOLLOW_TIMER) != TMRArd_EXPIRED)
		return LINE_FOLLOW_OK;

	// reinit the timer
	TMRArd_InitTimer(LINE_FOLLOW_TIMER, LINE_FOLLOW_UPDATE_PERIOD);

	int val[3];
	char retVal;
	
	if (spd > 0)
		readFrontSensors(val);
	else
		readBackSensors(val);

	int min = removeMin(val);
	int max = getMax(val);
	int correction = 0;
	int error = getLinePos(val);

	// check min and max to determine
	// where all sensors covered, or all sensors on white
	
	if (min > LINE_SENSOR_MIN_THRES) {	// all black
		retVal = LINE_FOLLOW_ALL_LINE;
	}
	else if (max < LINE_SENSOR_MAX_THRES) {	// no line!
		retVal = LINE_FOLLOW_NO_LINE;
	}
	else {
		correction = (error * LINE_KP) >> 3;
		correction += (error - oldError) * LINE_KD;
		retVal = LINE_FOLLOW_OK;
	}

	if (spd > 0)
		adjustMotion(spd, -correction);
	else
		adjustMotion(-spd, -correction);
		
	oldError = error;

	return retVal;
}
