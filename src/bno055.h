// bno055.h

#ifndef _bno055_h
#define _bno055_h

//#include "arduino.h"
#include <Adafruit_BNO055.h>

class Imu {

public:
	bool setAddressAndName(byte thisAddress, String thisName, String thisId);
	bool changedBnoSensorData();
	float getYaw();
	float getRoll();
	float getPitch();
	String getId();
	String getName();
	unsigned long getMillisLastPublished();
	void setMillisLastPublished();
	int absAngleDiff(int a, int b);
	
private:
	byte address;
	String name;
	String id;
	Adafruit_BNO055 device;
	float yaw;
	float roll;
	float pitch;
	float prevYaw;
	float prevRoll;
	float prevPitch;
	unsigned long millisLastPublished;
};

#endif