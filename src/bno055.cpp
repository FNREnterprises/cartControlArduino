#include <Adafruit_BNO055.h>
#include "bno055.h"

extern bool verbose;

bool Imu::setAddressAndName(byte thisAddress, String thisName, String thisId) {

	address = thisAddress;
	name = thisName;
	id = thisId;

	Serial.print(F("check i2c address 0x"));
	Serial.print(address, HEX);
	Serial.println();
	device = Adafruit_BNO055(55, address);

	/* Initialise the sensor */
	Serial.print(F("Check for BNO055 "));
	Serial.print(name);
	Serial.println();

	if (!device.begin())
	{
		/* There was a problem detecting the BNO055 ... check your connections */
		Serial.println(F("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!"));
		return false;
	}
	Serial.println(F("BNO055 connected"));
	
	device.setExtCrystalUse(true);

	sensors_event_t bnoData;
	device.getEvent(&bnoData);
	prevYaw = bnoData.orientation.x;
	prevPitch = bnoData.orientation.y;
	prevRoll = bnoData.orientation.z;	
	return true;
}

float Imu::getYaw() { return yaw;  }
float Imu::getRoll() { return roll; }
float Imu::getPitch() { return pitch; }
String Imu::getId() { return id; }
String Imu::getName() { return name; }
unsigned long Imu::getMillisLastPublished() { return millisLastPublished; }
void Imu::setMillisLastPublished() { millisLastPublished = millis(); }

int Imu::absAngleDiff(int a, int b) {
	int diff = (a % 360) - (b % 360) + 360;		// arduino can not modulo with neg numbers
	return abs((diff + 180) % 360 - 180) % 360;	
}

bool Imu::changedBnoSensorData() {

	sensors_event_t bnoData;
	bool dataChanged = false;

	device.getEvent(&bnoData);

	// normalize, depends on sensor mounting
	yaw = bnoData.orientation.x;
	pitch = bnoData.orientation.y;
	roll = bnoData.orientation.z;	// Correction?

	// avoid high number of updates when cart in move
	if (millis() - getMillisLastPublished() < 100) return false;

	// check for changed values
	int yawDiff = absAngleDiff(int(yaw), int(prevYaw));
	if (yawDiff > 1) {
		dataChanged = true;
		prevYaw = yaw;
	}

	if (abs(prevRoll - roll) > 1) {
		dataChanged = true;
		prevRoll = roll;
	}

	if (abs(prevPitch - pitch) > 1) {
		dataChanged = true;
		prevPitch = pitch;
	}
	return dataChanged;
}



