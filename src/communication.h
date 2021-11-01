// communication.h

#ifndef _COMMUNICATION_h
#define _COMMUNICATION_h

//#include "arduino.h"

#include "cartControlArduino.h"
#include "bno055.h"
#include "table.h"
#include "distance.h"

#define PIN_KINECT_POWER  32

extern bool cartControlActive;
//extern bool newData;
//extern char receivedChars[64];
//extern int numChars;
extern unsigned long lastMsg;
extern bool configurationComplete;

//given by cartControl with message 'a'
extern int FLOOR_MAX_OBSTACLE;
extern int FLOOR_ABYSS_MAX;
extern int FLOOR_FAR_MIN;
extern int FLOOR_FAR_MAX;

extern float speedFactor;
extern float speedOffset;
extern float speedFactorSideway;
extern float speedFactorDiagonal;

extern int _requestedTableHeight;

void recvWithEndMarker();

void checkCommand();

void sendImuValues(Imu imu);

#endif