// Main program file
// Controls initialization, as well as the high-level FSM controlling the robot.
#include "defines.h"
#include "globalTimeout.h"
#include "sensors.h"
#include "motor.h"

#include <Servo.h>
#include <Timers.h>

void setup() {
	// initialize modules
	globalTimeoutSetup();
	motorSetup();
        sensorSetup();
	// initialize serial
	Serial.begin(57600);
	Serial.println("Initialized");
}
void loop() {
	static int oldError = 0;
	int val[3];
//	readFrontSensor(val);
        //Check side Sensor
        if(!readSideSensor()) Serial.println("OFF");
        //
        
        //Test Wall Sensor
        /*
        if(testWallSensor()) Serial.println("Elena rocks!!");
        if(testTokensLoadedSensor()) Serial.println("Woohooooo!");
        
        */
        
        /* Uncomment this part if sensorSetup doesn't work for some reason..
        digitalWrite(WALL_SENSOR,HIGH); //Pull up the wall sensor to be on by default...
        digitalWrite(TOKENS_LOADED,HIGH); //Pull up token toggle        
        */
        
	removeMin(val);

	int error = getLinePos(val);

	int correction = (getLinePos(val) >> 1) + ((error - oldError) << 3);

/* Print readouts from front or rear sensors
	Serial.print(val[0]);
	Serial.print(" ");
	Serial.print(val[1]);
	Serial.print(" ");
	Serial.print(val[2]);
	Serial.print(" ");
	Serial.print(correction);
	Serial.println("");
*/



	//setMotion(190, -correction);
	//setMotion(-190, -correction);

	oldError = error;

	delay(2);
}
