

char version[10] = "v2.5";

// 
// 
#include <Arduino.h>
#include <Adafruit_BNO055.h>
#include <math.h>
#include <Wire.h>
#include <EEPROM.h>

#include "cartControlArduino.h"
#include "drive.h"
#include "distance.h"
#include "table.h"
#include "bno055.h"
#include "communication.h"


bool verbose = true;	// initial setting

///////////////////////////////////////////////////////////////////////////////
// these values may get overwritten by cartControl with message 'a'
int FLOOR_MAX_OBSTACLE = 15;
int FLOOR_MAX_ABYSS = 20;
int NUM_REPEATED_MEASURES = 7;
int DELAY_BETWEEN_ANALOG_READS = 20;	// may get overwritten by cart control
int MIN_MEASURE_CYCLE_DURATION = 80;  // may get overwritten by cart control
int finalDockingMoveDistance = 12;		//cm!
////////////////////////////////////////////////////////////////////////////////

// when reading ir sensor values wait at least this time to allow servo to reach its destination
int IR_MIN_WAIT_SWIPE_SERVO = 70;  // wait time after moving swipe servos to next step
int IR_MIN_READS_TO_ADJUST = 5;

char const *BLOCKING_STATUS_NAMES[] = {"NO_DATA", "FREE", "OBSTACLE", "ABYSS"};
char const *MOVEMENT_STATUS_NAMES[] = {"MOVING", "STOPPED"};
char const *MOVEMENT_NAMES[] = { "STOP", "FORWARD", "FOR_DIAG_RIGHT", "FOR_DIAG_LEFT",
  "LEFT", "RIGHT", "BACKWARD", "BACK_DIAG_RIGHT", "BACK_DIAG_LEFT",
  "ROTATE_COUNTERCLOCK", "ROTATE_CLOCKWISE" };

////////////////////////////////////////////
// global values
////////////////////////////////////////////
int loopCount;
unsigned long loopStartMs;
unsigned long workMs;
int avgWait = 0;

// global variable for table
int _currentTableHeight;
int _requestedTableHeight;
TABLE_STATUS _tableStatus;

float speedFactor = 1.527;
float speedOffset = 52.9;
float speedFactorSideway = 0.6;
float speedFactorDiagonal = 0.7;

byte platformBnoAddress = 0x28;
byte headBnoAddress = 0x29;

Imu headImu;
Imu platformImu;

int cartCamPosition = 0;		// pointing down
int cartCamServoPin = 13;

// serial commands
boolean cartControlActive = false;
bool configurationComplete = false;

int testDirection;

// last check on 12V supply
unsigned long last12VCheckMillis;

// docking state
boolean isDocked = false;

// wheel pulses
volatile unsigned long wheelPulseCounter = 0;

int _speedUnifyer[MOTORS_COUNT]{ 104,100,96,82 };

int movementDuration;
int maxDuration;
int delayMillis;
boolean dockingSwitchState;
float distance;
unsigned long counts;

int sensorInTest;

// use only 1 interrupt and count the flanks as we know the current direction
void wheelChange() {
	wheelPulseCounter += 1;         
}


void initializeUltrasonicSensors() {

	Wire.begin();		// master needs no address

	// try to send start measure message to subdevice
	Wire.beginTransmission(ULTRASONIC_DISTANCE_DEVICE_I2C_ADDRESS);
	Wire.write(1);
	Wire.endTransmission();

	// allow subdevice to capture echos
	delay(500);

	// try to get distances
	Serial.println(F("request ultrasonic distances from Arduino nano over I2C"));
	//int numBytes = Wire.requestFrom(ULTRASONIC_DISTANCE_DEVICE_I2C_ADDRESS, 1 + ULTRASONIC_DISTANCE_SENSORS_COUNT);
	Wire.requestFrom(ULTRASONIC_DISTANCE_DEVICE_I2C_ADDRESS, 1 + ULTRASONIC_DISTANCE_SENSORS_COUNT);

	// first byte is validity mask
	if (Wire.available()) ultrasonicDistanceSensorValidity = Wire.read();
	Serial.print(F("validity: ")); Serial.print(ultrasonicDistanceSensorValidity, BIN); Serial.print(F(", distances: "));

	// following reads are distances for each usSensor
	int usSensorId = 0;
	while (Wire.available()) {
		ultrasonicDistanceSensorValues[usSensorId] = Wire.read();
		Serial.print(ultrasonicDistanceSensorValues[usSensorId]); Serial.print(", ");
		usSensorId++;
	}
	Serial.println();

	// send stop measure command
	Serial.println(F("stop measuring ultrasonic distances"));
	Wire.beginTransmission(ULTRASONIC_DISTANCE_DEVICE_I2C_ADDRESS);
	Wire.write(2);
	Wire.endTransmission();

	Serial.println(F("ultrasonic sensors available"));
}

//////////////////////////////////////////////////////////////
void setup()
{
	// the power pins
	for (int i = 30; i < 34; i++) {
		digitalWrite(i, HIGH);		// should switch relais off (apply before setting pinmode!)
		pinMode(i, OUTPUT);
	}

	// show Relais Board 1 is working by activating IN4 for 1 second 
	digitalWrite(PIN_RELAIS_TEST, LOW);
	delay(1000);
	digitalWrite(PIN_RELAIS_TEST, HIGH);

	Serial.begin(115200);

	Serial.print(F("C0 cartControlArduino ")); Serial.println(version);

	// wait for received configuration values
	unsigned long timeoutWait = millis() + 5000;
	while (!configurationComplete && millis() < timeoutWait) { 
		delay(500);
		checkCommand();
	}
	if (millis() > timeoutWait) {
		Serial.println(F("no configuration values received, using default values"));
	} else {
		Serial.println(F("configuration values received, Arduino continues setup"));
	}
	// log the active values
	Serial.print(F(" FLOOR_MAX_OBSTACLE: ")); Serial.println(FLOOR_MAX_OBSTACLE);
	Serial.print(F(" FLOOR_MAX_ABYSS: ")); Serial.println(FLOOR_MAX_ABYSS);
	Serial.print(F(" NUM_REPEATED_MEASURES: ")); Serial.println(NUM_REPEATED_MEASURES);
	Serial.print(F(" DELAY_BETWEEN_ANALOG_READS: ")); Serial.println(DELAY_BETWEEN_ANALOG_READS);
	Serial.print(F(" MIN_MEASURE_CYCLE_DURATION: ")); Serial.println(MIN_MEASURE_CYCLE_DURATION);
	Serial.print(F(" finalDockingMoveDistance: ")); Serial.println(finalDockingMoveDistance);

	setupFahren();

	delay(500);
	pinMode(CHARGE_6V_BATTERY_PIN, OUTPUT);
	// set docking switch port to input
	pinMode(DOCKING_SWITCH_PIN, INPUT);
	int dockingSwitch = digitalRead(DOCKING_SWITCH_PIN);

	// if docking switch is not active run distance checks with the sensors
	Serial.println(F("check for activated docking switch"));
	setupSwipeServos(dockingSwitch);

	tableSetup();

	// the imu's
	while (!platformImu.setAddressAndName(platformBnoAddress, "platformImu", "!I1")) {
		Serial.println(F("could not connect with platform imu"));
		delay(100);
	}
	Serial.println(F("connected with platform imu"));
	// read a few times to get stable values
	for (int i=0; i<5; i++) {platformImu.getYaw();}
	sendImuValues(platformImu);

	while (!headImu.setAddressAndName(headBnoAddress, "headImu", "!I2")) {
		Serial.println(F("could not connect with head imu"));
		delay(100);
	}
	Serial.println(F("connected with head imu"));
	// read a few times to get stable values
	for (int i=0; i<5; i++) {platformImu.getYaw();}
	sendImuValues(headImu);

	loadFloorReferenceDistances();

	initializeUltrasonicSensors();

	// set power relais pins to output
	pinMode(CHARGE_LAPTOP_BATTERY_PIN, OUTPUT);
	pinMode(CHARGE_12V_BATTERY_PIN, OUTPUT);

		// set initial docking switch state to undocked
	isDocked = false;

	// and deactivate power relais
	digitalWrite(CHARGE_LAPTOP_BATTERY_PIN, HIGH);
	digitalWrite(CHARGE_6V_BATTERY_PIN, HIGH);
	digitalWrite(CHARGE_12V_BATTERY_PIN, HIGH);

	// make shure to show voltage with first movement after restart
	last12VCheckMillis = 0;

	// set wheel encoder pins and interrupt routines
	pinMode(WHEEL_ENCODER_PIN_A, INPUT_PULLUP);

	// wheel encoder counts by interrupt
	attachInterrupt(digitalPinToInterrupt(WHEEL_ENCODER_PIN_A), wheelChange, CHANGE);

	Serial.println(F("Arduino setup done"));
	Serial.println("!A0");		//cart ready");
}


void loop()
{
	loopStartMs = millis();
	loopCount++;
	
	//Serial.print(F("loopCount: ");  Serial.println(loopCount);
	// check for new commands
	checkCommand();

	handleCartMovement();

	// check for changed imu values
	// limit updates to 10 per second
	if (headImu.changedBnoSensorData()) {
		sendImuValues(headImu);
	}
	if (platformImu.changedBnoSensorData()) {
		sendImuValues(platformImu);
	}

	// check for heartbeat from controlling computer
	unsigned long diff = millis() - lastMsg;
	if (cartControlActive && diff > 15000UL) {  // 500, use longer times for manual tests
		Serial.print(F("timeout heartbeat messages, stop cart, time: ")); 
		Serial.print(millis()); Serial.print(F(" lastMsg: ")); Serial.print(lastMsg); 
		Serial.print(F(" diff: "));  Serial.println(diff);
		cartControlActive = false;
		stopCart(true, "no heartbeat from server");
	}


	// check docking switch change
	dockingSwitchState = digitalRead(DOCKING_SWITCH_PIN);
	if (dockingSwitchState != isDocked) {
		isDocked = dockingSwitchState;

			if (isDocked) {
				Serial.println("!D1");		// cart is docked

				// aktivate power relais
				digitalWrite(CHARGE_LAPTOP_BATTERY_PIN, LOW);
				digitalWrite(CHARGE_6V_BATTERY_PIN, LOW);
				digitalWrite(CHARGE_12V_BATTERY_PIN, LOW);
			}
			else {
				Serial.println("!D2");		// cart has undocked

				// deactivate power relais
				digitalWrite(CHARGE_LAPTOP_BATTERY_PIN, HIGH);
				digitalWrite(CHARGE_6V_BATTERY_PIN, HIGH);
				digitalWrite(CHARGE_12V_BATTERY_PIN, HIGH);
			}
	}

	// this compensates runtime differences in loop cycles due to actions required
	// if all time is used up start next loop
	// if partial time is used wait until the next planned cycle start
	workMs = millis() - loopStartMs;

	// when not in move use a fixed cycle time
	// if the cart is not moving we do not need to check for move end and free move space
	if (plannedCartMovement == STOP) {
		delay(20);			//minimal wait
		avgWait += 20;
	}
	else {
		if (workMs < 20) {
			avgWait += (20 - workMs);
			delay(20 - workMs);
		}
	}

	// monitor cpu free time
	if (loopCount > 200) {
		//Serial.print(F("avg wait ms in main loop: ")); Serial.println(avgWait / loopCount);
		avgWait = 0;
		loopCount = 0;
	}
}
