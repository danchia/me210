// Main program file
// Controls initialization, as well as the high-level FSM controlling the robot.
#include "defines.h"
#include "globalTimeout.h"
#include "sensors.h"
#include "motor.h"
#include "lineMotions.h"
#include "tokenManagement.h"

#include <Servo.h>
#include <Timers.h>

#define FWD_SPEED 210
#define SLOW_SPEED 115

#define TURN_SPEED 185

void setup() {
	// initialize modules
	globalTimeoutSetup();
	motorSetup();
	initializeServo();
	pinMode(LED_PIN, OUTPUT);

	// initialize serial
	Serial.begin(57600);
	Serial.println("Initialized");


      //Find out which side we're on
      char initialSide = findSide();
      updateMotor();
      //Head forward just for a little bit
//      setMotion(255,0);
  //    delay(50); 
  
      
      int transSpeed = 200; 
      int rotSpeed = 5;

int val[3];
      
      if (initialSide == 0) {
        
              //Turn just a little bit
              setMotion(0,-175);
              for (int i = 0; i < 100; i++) {
                updateMotor();
                delay(1);
              }
                    
              stopMotion();
              for (int i = 0; i < 50; i++) {
                updateMotor();
                delay(1);
              }
              updateMotor();
        
        //Idea #1: Do a gentle curve to get onto the "home" tape          
         setMotion(transSpeed,0);
         
              
      //Keep going until we hit tape, then do line following routine
      while(1) {
        updateMotor();
        readFrontSensors(val);
        if(hasLine(val)) break;
      }

      do {
        readFrontSensors(val);
      }while(hasLine(val));

      delay(30);

      adjustMotion(0, -160);
      do {
        updateMotor();
        readFrontSensors(val);
      } while(!hasLine(val));
         
      } 
      else {
         setMotion(transSpeed,3);
         
              
      //Keep going until we hit tape, then do line following routine
      while(1) {
        updateMotor();
        readFrontSensors(val);
        if(hasLine(val)) break;
      }

      do {
        readFrontSensors(val);
      }while(hasLine(val));

      delay(30);

      adjustMotion(0, 160);
      do {
        updateMotor();
        readFrontSensors(val);
      } while(!hasLine(val));
         
      }
     
}

// function for determining side of board
// use blocking code for now
// returns 1 if right side
// need to recalibrate
char findSide() {
	setMotion(0, SEESAW_HOME_TURN_SPD);

	
	//Serial.println("Seesaw");
	while(!readHomeBeacon()) {
            updateMotor();
        }
        unsigned long startTime = millis();
	
        
        while(!readFrontSeesaw()) {
          updateMotor();
        };
        
        unsigned long endTime1 = millis();

        while(!readHomeBeacon()) {
            updateMotor();
        }

        unsigned long endTime2 = millis();
	//Serial.println("Home");
	//Serial.println(endTime);
        stopMotion();

	if (endTime1 - startTime < endTime2 - endTime1) {
		digitalWrite(13, HIGH);
		return 1;
	}
	else
		return 0;
}

void loop() {
  /*
	for (int i = 0; i < 50; i++) {
		updateServo();
		delay(15);
	}
*/

	// test line following
	startLineFollowing(FWD_SPEED);

	while(followLine(FWD_SPEED) == LINE_FOLLOW_OK) updateMotor();	// follow line
	adjustMotion(SLOW_SPEED,0);
	while(!readSideSensor()) updateMotor();	// wait for turn sensor
	stopMotion();
	while(!motorDoneStop()) updateMotor();

	//turn!
	setMotion(0, -TURN_SPEED);
	int val[3];
	do {
		readFrontSensors(val);
		updateMotor();
	}while(val[1] < LINE_SENSOR_MIN_THRES);

	stopMotion();
	while(!motorDoneStop()) updateMotor();


	startLineFollowing(FWD_SPEED);

	char token = 1;

	while(followLine(FWD_SPEED) == LINE_FOLLOW_OK) {
		if (readSideSeesaw() && !token) {
			stopMotion();
			while(!motorDoneStop()) updateMotor();

			depositTokens();
			for (int i = 0; i < 200; i++) {
				updateMotor();
				updateServo();
				delay(15);
			}
			
			token = 1;
			startLineFollowing(FWD_SPEED);
		}

		updateServo();
                updateMotor();
	}
	stopMotion();

	while(!motorDoneStop()) updateMotor();

	while(1) updateMotor();
}
