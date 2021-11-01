// Fahren.h

#ifndef _FAHREN_h
#define _FAHREN_h

//#include "arduino.h"
#include "distance.h"

//given by cartControl with message 0
extern int FLOOR_MAX_OBSTACLE;
extern int FLOOR_MAX_ABYSS;

extern unsigned long last12VCheckMillis;

extern unsigned long moveRequestReceivedMillis;

extern int MIN_MEASURE_CYCLE_DURATION;

extern boolean servoInvolved[];
extern int involvedIrSensors[];
extern int numInvolvedIrSensors;

enum MOTORS { MOTOR_FRONT_RIGHT, MOTOR_FRONT_LEFT, MOTOR_BACK_RIGHT, MOTOR_BACK_LEFT, MOTORS_COUNT };
enum SPEED_PHASE {ACCELERATE, CRUISE, DECELERATE};
enum MOVE_TYPE {STRAIGHT, ROTATE, SENSORTEST, NONE};
extern int _speedUnifyer[MOTORS_COUNT];
extern MOVE_TYPE moveType;

// initialise move
extern void setupFahren();

// driving direction
extern void setPlannedCartMove(MOVEMENT, int, int, int, int, bool);
extern MOVEMENT plannedCartMovement;
extern MOVEMENT activeCartMovement;

// stop cart
extern void stopCart(bool, String);

// drive
extern void handleCartMovement();
extern int checkUltrasonicDistances();

#endif

