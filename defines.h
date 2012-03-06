// Global defines for program
// basically constants such as pinouts, and various params
// for tweaking

#pragma once

//
// General Constants and params
//

#define SEESAW_HOME_TURN_SPD 190

#define GAME_TIMEOUT_MSEC 120000		// two minutes

#define LINE_KP 4		// * 1/8
#define LINE_KD 15	
#define LINE_POS_SCALE 100
#define LINE_SENSOR_MIN_THRES 800
#define LINE_SENSOR_MAX_THRES 100
#define LINE_FOLLOW_UPDATE_PERIOD 2	// 2ms period, 500Hz control rate

#define MOTOR_FULL_POWER_TIME 100	// time to give motors full power initially (msecs)
#define MOTOR_STOP_TIME 70	// time to give motors full power reverse for stop (msecs)

//
// Timers
//

#define MOTOR_TIMER 1
#define LINE_FOLLOW_TIMER 2
#define SERVO_TIMER 3
#define MAIN_TIMER 4




//
// Pins
//

//TOGGLE SWITCHES
#define WALL_SENSOR 8
#define TOKENS_LOADED 9


// MOTOR
#define MOTOR_LEFT_DIR 2
#define MOTOR_LEFT_PWM 3
#define MOTOR_RIGHT_DIR 12
#define MOTOR_RIGHT_PWM 11

// TAPE
#define TAPE_SENSOR_FRONT_LEFT 0
#define TAPE_SENSOR_FRONT_CENTER 1
#define TAPE_SENSOR_FRONT_RIGHT 2
#define TAPE_SENSOR_REAR_LEFT 3
#define TAPE_SENSOR_REAR_CENTER 4
#define TAPE_SENSOR_REAR_RIGHT 5
#define TAPE_SENSOR_SIDE 7

// BEACON
#define HOME_BEACON_SENSOR 4
#define FRONT_SEESAW_SENSOR 5
#define SIDE_SEESAW_SENSOR 6

// SERVO
#define SERVO_PIN 10

// LED
#define LED_PIN 13
