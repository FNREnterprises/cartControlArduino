// MotorizedBase.h

#ifndef _MOTORIZEDBASE_h
#define _MOTORIZEDBASE_h

//#include "arduino.h"
#include "bno055.h"

// print macros using F() PROGMEM for text strings
#define prt(x) Serial.print(F(x))
#define prtl(x) Serial.println(F(x))
#define pr(x) Serial.print(x)
#define prl(x) Serial.println(x)

#define PIN_TABLE_HEIGHT A13
#define PIN_TABLE_PWM1 2
#define PIN_TABLE_PWM2 3
#define PIN_TABLE_DIR1 26
#define PIN_TABLE_DIR2 27

#define PIN_TABLE_POWER 30
#define PIN_DRIVE_POWER 31
#define PIN_KINECT_POWER 32
#define PIN_RELAIS_TEST 33

// docking switch and charging relais assignment (4 Relais Board 2, front left)
#define DOCKING_SWITCH_PIN 45
#define CHARGE_LAPTOP_BATTERY_PIN 44
#define CHARGE_12V_BATTERY_PIN 46
#define CHARGE_6V_BATTERY_PIN 48

// the relais boards switch off with HIGH signal
#define SWITCH_ON LOW
#define SWITCH_OFF HIGH

// the direction of the table motors
#define TABLE_UP HIGH
#define TABLE_DOWN LOW

// encoder assignments
#define WHEEL_ENCODER_PIN_A 19
//#define WHEEL_ENCODER_PIN_B 19
#define PULS_PER_MM_STRAIGHT 3.32	//3.13, lower value shorter distance
#define PULS_PER_MM_SIDEWAYS 3.35
#define PULS_PER_MM_DIAGONAL 3.35
#define NUM_MEASURE_STEPS 11			// measure steps in swipe

// i2c connected subprocessor for ultrasonic sensors
#define ULTRASONIC_DISTANCE_DEVICE_I2C_ADDRESS 10

// enum and typedef
enum TABLE_STATUS { UP, DOWN, TABLE_STOP };

enum MOVEMENT {
	STOP, 
	FORWARD, 
	FOR_DIAG_RIGHT, 
	FOR_DIAG_LEFT,
	LEFT, 
	RIGHT,
	BACKWARD, 
	BACK_DIAG_RIGHT, 
	BACK_DIAG_LEFT,
	ROTATE_COUNTERCLOCK, 
	ROTATE_CLOCKWISE,
	MOVEMENT_COUNT
};
extern char const * MOVEMENT_NAMES[];

enum BLOCKING_STATUS { NO_DATA, FREE, OBSTACLE, ABYSS };
extern char const * BLOCKING_STATUS_NAMES[];

enum MOVEMENT_STATUS { MOVING, STOPPED };
extern char const * MOVEMENT_STATUS_NAMES[];



// DECLARATION of global variables in motorizedBase.ino
extern int NUM_REPEATED_MEASURES;
extern int DELAY_BETWEEN_ANALOG_READS; 
extern int MIN_MEASURE_CYCLE_DURATION;
extern int finalDockingMoveDistance;
extern int sensorInTest;
extern bool analogMeasureCycle;

extern bool verbose;

extern Imu platformImu;
extern Imu headImu;

extern int movementDuration;
extern int maxDuration;
extern int delayMillis;
extern boolean dockingSwitchState;
extern float distance;
extern unsigned long counts;

extern void loadFloorReferenceDistances();		//distance.cpp
extern volatile unsigned long wheelPulseCounter;

#endif


