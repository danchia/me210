/******************************************************************************
Module
  tokenManagement.cpp 
Description
  This library contains code to manage the servo for token deposition..

Arduino IDE version:  0022
  
History
When      Who  Description
--------  ---  -------------------------------------
03/02/12  ELC  created tokenManagement.h for the Arduino UNO R2

******************************************************************************/

/*----------------------------- Include Files -------------------------------*/
// include definitions folder: SERVO_PIN, SERVO_TIMER
#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include "tokenManagement.h"
#include "defines.h"
#include <Timers.h>
#include <Servo.h>

/*----------------------------- Module Defines ------------------------------*/
#define SERVO_TIMER_LENGTH 15
#define GOING_FORWARD 1
#define GOING_HOME 2
#define AT_REST 3
#define DEGREE_FRACTION 1
#define DELTA 2

#define MAX_BUCKETS 1
#define FORTYFIVE 45
#define NINETY 100
#define ONETHIRTYFIVE 150
#define MAX_ANGLE 150
#define HOME 0


/*----------------------------- Module Variables ---------------------------*/
static int servoAngle;
static int bucketNo;
static int servoState;
static int nextBucketAngle;
static Servo servo;

/*----------------------------- Module Code --------------------------------*/

/******************************************************************************
  Function:    depositTokens
  Contents:    Servo deposits tokens from next bucket. If all buckets have been
                used, servo will instead go home.
  Parameters:  None
  Returns:     Nothing
  Notes:
******************************************************************************/
void depositTokens(void) {
  bucketNo = bucketNo + 1;
  nextBucketAngle = evaluateBucketAngle(bucketNo);
  if (nextBucketAngle == HOME) {
    servoState = GOING_HOME; // sets servo to return to home position
    ////Serial.println("Servo going home (deposit)");
  } else {
    servoState = GOING_FORWARD;
    ////Serial.println("Servo depositing");
  }
} 


/******************************************************************************
  Function:    goHome
  Contents:    Servo returns to home position (angle = 0)
  Parameters:  None
  Returns:     Nothing
  Notes:
******************************************************************************/
void goHome(void) {
  servoState = GOING_HOME;
  bucketNo = HOME;
} 
  

/******************************************************************************
  Function:    bucketAngle = evaluateBucketAngle(bucketNo)
  Contents:    Returns the servo angle to reach the bucket.
  Parameters:  None
  Returns:     Bucket angle
  Notes:
******************************************************************************/

int evaluateBucketAngle(int bucketNo) {
  int bucketAngle;
  switch (bucketNo) {
    case 1:
      bucketAngle = FORTYFIVE;
      break;
    case 2:
      bucketAngle = NINETY;
      break;
    case 3:
      bucketAngle = ONETHIRTYFIVE;
      break;
    case HOME:
      bucketAngle = HOME;
      break;
    default:
      bucketAngle = HOME;
      break;
  }
  return bucketAngle<<DEGREE_FRACTION;
}
      
/******************************************************************************
  Function:    updateServo(servoAngle)
  Contents:    Updates the angle of the servo.
  Parameters:  None
  Returns:     Nothing
  Notes:
******************************************************************************/
void updateServo(void) {
  if (TMRArd_IsTimerExpired(SERVO_TIMER)) {
    switch (servoState) {
      case GOING_FORWARD:
        servoAngle = servoAngle + DELTA;
        ////Serial.println(servoAngle);
        if (servoAngle > nextBucketAngle) {
          servoAngle = nextBucketAngle;
          servoState = AT_REST;
          //Serial.println("reached rest");
        }
        break;
      case GOING_HOME:
        servoAngle = servoAngle - DELTA;
        //Serial.println(servoAngle);
        if (servoAngle < 0) {
          servoAngle = HOME;
          servoState = AT_REST;
          //Serial.println("reached rest");
        }
        break;
      case AT_REST: 
      default:
        // do nothing
        break;
    }
    ////Serial.println(servoAngle>>DEGREE_FRACTION);
    servo.write(servoAngle>>DEGREE_FRACTION);
    TMRArd_InitTimer(SERVO_TIMER, SERVO_TIMER_LENGTH);
  }
}


/******************************************************************************
  Function:    initializeServo
  Contents:    Initializes the servo at the beginning of the program.
  Parameters:  None
  Returns:     Nothing
  Notes:
******************************************************************************/
void initializeServo(void) {
  servo.attach(SERVO_PIN);
  bucketNo = 0;
  servoAngle = 0;
  servoState = AT_REST;
  TMRArd_InitTimer(SERVO_TIMER, SERVO_TIMER_LENGTH);
}

/******************************************************************************
  Function:    bucketsLeft
  Contents:    Returns the number of buckets left presumabely still full
  Parameters:  None
  Returns:     number of buckets left
  Notes:
******************************************************************************/
int bucketsLeft(void) {
  int numBucketsLeft = MAX_BUCKETS - bucketNo;
  return numBucketsLeft;
}
