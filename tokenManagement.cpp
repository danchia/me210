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
#include <Servo.h>
#include "tokenManagement.h"
#include "defines.h"
#include <Timers.h>

/*----------------------------- Module Defines ------------------------------*/
#define SERVO_TIMER_LENGTH 15
#define GOING_FORWARD 1
#define GOING_HOME 2
#define AT_REST 3
#define DEGREE_FRACTION 1
#define DELTA 1

#define MAX_BUCKETS 3
#define FORTYFIVE 45
#define NINETY 90
#define ONETHIRTYFIVE 135
#define MAX_ANGLE 135
#define HOME 0


/*----------------------------- Module Variables ---------------------------*/
static int servoAngle;
static int bucketNo;
static int servoState;
static int nextBucketAngle;

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
  nextBucketAngle = evaluateBucketAngle(bucketNo + 1);
  if (nextBucketAngle == HOME) {
    servoState = GOING_HOME; // sets servo to return to home position
  } else {
    servoState = GOING_FORWARD;
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
        if (servoAngle > nextBucketAngle) {
          bucketNo = bucketNo + 1;
          servoAngle = nextBucketAngle;
          servoState = AT_REST;
        }
        break;
      case GOING_HOME:
        servoAngle = servoAngle - DELTA;
        if (servoAngle < 0) {
          bucketNo = HOME;
          servoAngle = HOME;
          servoState = AT_REST;
        }
        break;
      case AT_REST: 
      case default:
        // do nothing
        break;
    }
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
int numBucketsLeft = bucketsLeft(void) {
  numBucketsLeft = MAX_BUCKETS - bucketNo;
  return numBucketsLeft;
}
