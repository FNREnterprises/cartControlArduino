// 
// 
// 
#include "distance.h"
#include "cartControlArduino.h"
#include "drive.h"
#include "table.h"

#include <EEPROM.h>
#include <Servo.h>

#define min(a,b) ((a)<(b)?(a):(b))
#define cos30 0.866					// used to calc obstacle height
Servo swipeServos[SWIPE_SERVOS_COUNT];

// Configurable Values
int swipeServoStartAngle = 30;	// swipe servo start degrees
int swipeServoRange = 110;		// swipe servo range

//
// const for calculating distance from infrared sensor raw values
// 
ir_sensor GP2Y0A21YK = { 5461.0, -17.0, 2.0 };
ir_sensor GP2Y0A41YK = { 1500.0, -11.0, 0 };

// infrared distance sensors
irSensorStepValuesType irSensorStepData[IR_SENSORS_COUNT][NUM_MEASURE_STEPS];
int irSensorStepRaw[IR_SENSORS_COUNT][NUM_REPETITIONS_IR_MEASURE];	// raw values current step

int irSensorObstacleMaxValue;
int irSensorObstacleMaxSensor;
int irSensorAbyssMaxValue;
int irSensorAbyssMaxSensor;

// the list of used servos and sensors in current cartDirection
boolean servoInvolved[SWIPE_SERVOS_COUNT];
int involvedIrSensors[MAX_INVOLVED_IR_SENSORS];
int numInvolvedIrSensors;
int swipeStep;

// the swiping servo variables
int swipeDirection;			// to swipe servo horn from left to right (+1) and back (-1)
int currentMeasureStep;	    // 
int nextMeasureStep;
int nextSwipeServoAngle;
long swipeStepStartMillis = 0;

// the ultrasonic distance sensors
ultrasonicDistanceSensorDefinition ultrasonicDistanceSensor[ULTRASONIC_DISTANCE_SENSORS_COUNT]{
	// sensorId, enabled, name,                 trigPin, echoPin, distanceCorrection
	{  0,        true,    "us front left left   ", 2,       3,       0 },
	{  1,        true,    "us front left center ",  4,       5,       0 },
	{  2,        true,    "us front right center", 6,       7,       0 },
	{  3,        true,    "us front right right ",  8,       9,       0 },
};

int ultrasonicDistanceSensorValues[ULTRASONIC_DISTANCE_SENSORS_COUNT];
int ultrasonicDistanceSensorValidity;


// Achtung: Adafruit DC Motors benutzen pin 4,7,8 und 12
// Bei Kollision in der Benutzung der Pins st?rzt der Arduino ab
// Mega-PWM-Pins = 2..13, 44,45,46
static servoDefinition servoDefinitions[SWIPE_SERVOS_COUNT]{
	//    Id, name, installed, pin, degree offset (clockwise +)
		{ FL, "FL", true, 11,   0 },
		{ FC, "FC", true, 12,   0 },
		{ FR, "FR", true, 13,  10 },
		{ BL, "BL", true,  6, -8 },
		{ BC, "BC", true,  9, -12 },
		{ BR, "BR", true, 10,  30 }
};


// sensorId, sensorName, sensorTyp, sensorRange, installed, pin, servoId, floorDistance
// swiped sensors should show 15 cm, fixed sensors 18 cm
irSensorDefinition irSensorDefinitions[IR_SENSORS_COUNT]{
//	  sensorId;          sensorName[20];       TYPE; swipe; installed; sensorPin; servoId
	{ SWIPE_FRONT_LEFT,   "swipeFrontLeft  ",  A41,  true,  true,      A2,        FL},
	{ SWIPE_FRONT_CENTER, "swipeFrontCenter",  A41,  true,  true,      A7,        FC},
	{ SWIPE_FRONT_RIGHT,  "swipeFrontRight ",  A21,  true,  true,      A3,        FR},
	{ SWIPE_BACK_LEFT,    "swipeBackLeft   ",  A21,  true,  true,      A8,        BL},
	{ SWIPE_BACK_CENTER,  "swipeBackCenter ",  A21,  true,  true,      A12,       BC},
	{ SWIPE_BACK_RIGHT,   "swipeBackRight  ",  A21,  true,  true,      A10,       BR},
	{ STATIC_FRONT_LEFT,  "staticFrontLeft ",  A21,  false, true,      A0,        -1},
	{ STATIC_FRONT_RIGHT, "staticFrontRight",  A21,  false, true,      A1,        -1},
	{ STATIC_BACK_LEFT,   "staticBackLeft  ",  A21,  false, true,      A9,        -1},
	{ STATIC_BACK_RIGHT,  "staticBackRight ",  A21,  false, true,      A11,       -1}
};

int irSensorReferenceDistances[IR_SENSORS_COUNT][NUM_MEASURE_STEPS];

// DEFINITION of global variables
// sensorId with measured closest obstacle
//int maxObstacleSensorId;

// sensorId with measured largest abyss
//int maxAbyssSensorId;



// distance 021 20..80 cm
// convert infrared sensor values to mm
// mark invalid values as DISTANCE_UNKNOWN
int analogToDistanceA21(ir_sensor sensor, int adc_value)
{
	if (adc_value < 50 || adc_value > 800)
	{
		if (verbose) {
			Serial.print(F("A21, raw value ")); Serial.print(adc_value); Serial.print(F(" out of scope (50..800): "));
			Serial.print(adc_value);
			Serial.println();
		}
		return DISTANCE_UNKNOWN;	//not a valid raw value
	}
	return (sensor.a / (adc_value + sensor.b) - sensor.k) * 10;	// dist in mm
}


// convert infrared sensor A41 values to mm
// mark invalid values as DISTANCE_UNKNOWN
int analogToDistanceA41(int adc_value)
{
	if (adc_value < 50 || adc_value > 800)
	{
		if (verbose) {
			Serial.print(F("A41, raw value ")); Serial.print(adc_value); Serial.print(F(" out of scope (50..800): "));
			Serial.print(adc_value);
			Serial.println();
		}
		return DISTANCE_UNKNOWN;	//not a valid raw value
	}
	
	return 13 / (adc_value * 0.00048828125);
}


////////////////////////////////////////////////////////////////////////
void setupSwipeServos(int dockingSwitch) {

	Serial.println(F("setup swipe servos"));
	int Pin;
	for (int sensorId = 0; sensorId < SWIPE_SERVOS_COUNT; sensorId++) {
	//for (int sensorId = 2; sensorId < 3; sensorId++) {
		Pin = servoDefinitions[sensorId].servoPin;
		if (servoDefinitions[sensorId].installed) pinMode(Pin, OUTPUT);
	}

	// check for startup with activated docking switch
	// in that case do not run the initial check on the swipe servos as we are right 
	// in front of the power station
	if (dockingSwitch != 99) {

		Serial.print(F("for ir sensor testing ignore dockingSwitch value: ")); Serial.println(dockingSwitch);
		Serial.println(F("now do some repeated ir distance measures to verify infrared sensors"));
		Serial.println(F("move swipe servo to middle position for checking correct horn position"));
		for (int sensorId = 0; sensorId < IR_SENSORS_COUNT; sensorId++) {
			
			Serial.print(F("sensor: ")); Serial.println(irSensorDefinitions[sensorId].sensorName);

			if (irSensorDefinitions[sensorId].swipe) {
				swipeServos[sensorId].attach(servoDefinitions[sensorId].servoPin);
				swipeServos[sensorId].write(90+servoDefinitions[sensorId].servoDegreeOffset);
				delay(70);   // let the swipe servo reach its destination
			}

			// let the sensor adjust
			for (int i = 0; i < IR_MIN_READS_TO_ADJUST; i++){
				analogRead(irSensorDefinitions[sensorId].sensorPin);
				delay(2);
			}

			int distRaw;
			int distMm;
			int distMin = 500;
			int distMax = 0;

			for (int i = 0; i < 10; i++) {
				distRaw = analogRead(irSensorDefinitions[sensorId].sensorPin);
				if (irSensorDefinitions[sensorId].sensorType == A21) {
					distMm = analogToDistanceA21(GP2Y0A21YK, distRaw);
				} else {
					distMm = analogToDistanceA41(distRaw);
				}
				if (distMm < distMin) {distMin = distMm;}
				if (distMm > distMax) {distMax = distMm;}
				
				Serial.print(F("distRaw: ")); Serial.print(distRaw);
				Serial.print(F(", distMm: ")); Serial.print(distMm);
				Serial.println();
				delay(DELAY_BETWEEN_ANALOG_READS);
			}
			Serial.print(F("distMin: ")); Serial.print(distMin); 
			Serial.print(F(", distMax: ")); Serial.print(distMax);
			Serial.print(F(", range: ")); Serial.print(distMax-distMin);
			Serial.println();
		}
	}

	stopSwipe();

	Serial.println(F("setup swipe servos done"));
}



void attachServo(int servoId) {
	if (!swipeServos[servoId].attached()) {
		swipeServos[servoId].attach(servoDefinitions[servoId].servoPin);
	}
}

/////////////////////////
/////////////////////////
void nextSwipeServoStep() {

	int relAngle;

	nextMeasureStep = currentMeasureStep + swipeDirection;
	
	pr(millis() - moveRequestReceivedMillis); pr(" ms, ");
	prt("nextSwipeStep, current: "); pr(currentMeasureStep);
	prt(", next: "); pr(nextMeasureStep);
	prl();

	if (nextMeasureStep == NUM_MEASURE_STEPS - 1) {
		swipeDirection = -1;
	}
	if (nextMeasureStep == 0) {
		swipeDirection = 1;
	}

	if (currentMeasureStep == 0 || currentMeasureStep == NUM_MEASURE_STEPS) {
		// all scan steps done, update cartControl
		//Serial.print(F("!F0,from_drive, Step")); Serial.println(currentMeasureStep);
	}

	// the relative Angle for the measuring
	// each servo can have an individual degree offset compensating for servo horn mount
	// and servo build differences.
	// the offset can be used to get an identical direction of the sensors for each measure step
	// correct offset can easily be verified looking at the sensor in the "rest" position (swipeStartAngle)
	// adjust the offset in the servoDefinitions
	relAngle = int(nextMeasureStep * (swipeServoRange / (NUM_MEASURE_STEPS-1)));

	// for all involved servos of the requested cartDirection rotate to next step
	for (int servoId = 0; servoId < SWIPE_SERVOS_COUNT; servoId++) {
		if (servoInvolved[servoId]) {

			// swipe servo for measurement in different directions
			attachServo(servoId);		// conditional attach

			nextSwipeServoAngle =  relAngle 
				+ swipeServoStartAngle
				+ servoDefinitions[servoId].servoDegreeOffset;

			swipeServos[servoId].write(nextSwipeServoAngle);
		}
	}
	//Serial.print(F("swipe servo moved to nextMeasureStep: ")); Serial.print(nextMeasureStep);
	//Serial.println();

	swipeStepStartMillis = millis();
	return;
}



///////////////////////////////////////////////////
// check timestamps in distance sensor values 
///////////////////////////////////////////////////
bool isIrSensorDataCurrent() {

	unsigned long oldestMeasureInvolvedSensors = millis();
	int oldestSensor;
	int oldestStep;
	bool isCurrent;

	// check involved irSensors
	for (int item = 0; item < numInvolvedIrSensors; item++) {
		int sensorId = involvedIrSensors[item];

		int numStepsToCheck = irSensorDefinitions[sensorId].swipe ? NUM_MEASURE_STEPS : 1;
		unsigned long oldestMeasureThisSensor = millis();
		int oldestStepThisSensor;

		for (int step = 0; step < numStepsToCheck; step++) {
			if (irSensorStepData[sensorId][step].lastMeasureMillis < oldestMeasureThisSensor) {
				oldestMeasureThisSensor = irSensorStepData[sensorId][step].lastMeasureMillis;
				oldestSensor = sensorId;
				oldestStepThisSensor = step;
			};
		}

		if (false) {	// log only if really needed
			prt("oldest measure sensor: "); pr(getIrSensorName(sensorId));
			pr(", "); pr(millis() - oldestMeasureThisSensor);
			prt(", step: "); pr(oldestStepThisSensor);
			prl();
		}

		// update oldest of all involved sensors
		if (oldestMeasureThisSensor < oldestMeasureInvolvedSensors) {
			oldestMeasureInvolvedSensors = oldestMeasureThisSensor;
			oldestSensor = sensorId;
			oldestStep = oldestStepThisSensor;
		}
	}
	isCurrent = millis() - oldestMeasureInvolvedSensors < 2500;

	if (verbose) {
		pr(millis()-moveRequestReceivedMillis); pr(" ms, ");
		if (isCurrent) {
			prt("current sensor data available, oldest: "); pr(millis() - oldestMeasureInvolvedSensors);
		} else {
			prt("sensor data out of date, oldest: "); pr(millis() - oldestMeasureInvolvedSensors);
			pr(", "); pr(getIrSensorName(oldestSensor)); prt(", step: "); pr(oldestStep);
		}
		prl();
	}
	return isCurrent;
}



//////////////////////////////////////////////
// sort an array, modifies the passed in array
//////////////////////////////////////////////
void bubbleSort(int arr[], int numItems) {
	boolean swapped = true;
	int j = 0;
	int tmp;

	// repeat until no swappes occur anymore
	while (swapped) {
		swapped = false;
		j++;
		for (int i = 0; i < numItems - j; i++) {
			if (arr[i] > arr[i + 1]) {
				tmp = arr[i];
				arr[i] = arr[i + 1];
				arr[i + 1] = tmp;
				swapped = true;
			}
		}
	}
	//Serial.print(F("sorted: "));
	//for (int sensorId = 0; sensorId < numItems; sensorId++) {
	//	Serial.print(arr[sensorId]); Serial.print(", ");
	//}
	//Serial.println();
}


// swiping servos should have arrived at current step
// for all involved sensors do some reads to let sensor adjust to new distance
// then do a number of reads to allow to calc a median (done later in processing of the raw data)
// check range of repeated reads and retry max 3 times if values have a high deviation
// the raw values are stored in irSensorData array and the time of last measure is saved
void readIrSensorValues(int swipeStep) {

	int distRaw;
	int sensorId;
	
	pr(millis() - moveRequestReceivedMillis); prt(" ms, ");
	prt("readIrSensorValues, step: "); prl(swipeStep);

	for (int item = 0; item < numInvolvedIrSensors; item++) {
		sensorId = involvedIrSensors[item];

		// for static sensors set swipe step to 0
		int step = irSensorDefinitions[sensorId].swipe ? swipeStep : 0;

		// so some reads to let the sensor adjust to the new position
		for (int i = 0; i < IR_MIN_READS_TO_ADJUST; i++){
			analogRead(irSensorDefinitions[sensorId].sensorPin);
			delayMicroseconds(300);
		}

		int minValue = 1028;
		int maxValue = 0;
		int numTries = 0;
		int rangeLimit = 50;

		// repeat reading the sensor if we see a high deviation in values
		while (numTries < 3) {
			// now do a number of reads for getting a median and a value range
			for (int m = 0; m < NUM_REPEATED_MEASURES; m++) {

				distRaw = analogRead(irSensorDefinitions[sensorId].sensorPin);
				irSensorStepRaw[sensorId][m] = distRaw;
				// set min and max values of the repeated reads
				//if (distRaw < minValue) minValue = distRaw;
				//if (distRaw > maxValue) maxValue = distRaw;
				// use bubble sort and get rid of lowest and highest values
				
				/*
				// log every measurement in sensor test and verbose mode
				if (sensorInTest == sensorId && verbose) {
					Serial.print(getInfraredSensorName(sensorId));
					Serial.print(F(", swipeDir: "); Serial.print(swipeDirection);
					Serial.print(F(", swipeStep:"); Serial.print(step);
					Serial.print(F(", analogIn: A"); Serial.print(irSensorDefinitions[sensorId].sensorPin - 54);
					Serial.print(F(", measure: "); Serial.print(m);
					Serial.print(F(", raw: "); Serial.print(distRaw);
					Serial.println();
				}
				*/	
				delayMicroseconds(500);
			}
			bubbleSort(irSensorStepRaw[sensorId], NUM_REPEATED_MEASURES);
			minValue = irSensorStepRaw[sensorId][1];
			maxValue = irSensorStepRaw[sensorId][NUM_REPEATED_MEASURES-1];
			if (abs(maxValue - minValue) < rangeLimit) break;	// success in reading consistant values
			prt("retry sensor read: "); pr(sensorId); prl();
			numTries++;
		}
		// check for unsuccessful measure, log raw values
		if (numTries > 2) {
			prt("could not read consistent values: "); pr(getIrSensorName(sensorId));
			for (int i = 0; i < NUM_REPEATED_MEASURES; i++) {
				pr(irSensorStepRaw[sensorId][i]); prt(", ");
			}
			prl();
		}

		// set time of last read
		irSensorStepData[sensorId][step].lastMeasureMillis = millis();		
	}
}


/////////////////////////////////////////////////////////
// based on the collected raw values of the sensors in irSensorData
// find the median raw value
// calculate the distance in mm
// eval the new min/max values of the sensor
/////////////////////////////////////////////////////////
void processNewRawValues(int swipeStep) {

	int sensorId;
	int medianRaw;
	int distMm;
	float heightFactor;

	pr(millis() - moveRequestReceivedMillis); prt(" ms, ");
	prt("processNewRawValues, step: "); prl(swipeStep);
	
	// process distance data only for sensors involved in movement
	for (int item = 0; item < numInvolvedIrSensors; item++) {
		sensorId = involvedIrSensors[item];

		// for static sensors set swipe step to 0
		int step = irSensorDefinitions[sensorId].swipe ? swipeStep : 0;

		int refDist = irSensorReferenceDistances[sensorId][step];
		int obstacleHeight = 0;
		int abyssDepth = 0;


		// for each sensor sort the raw reads and use median as distance
		//bubbleSort(irSensorStepRaw[sensorId], NUM_REPEATED_MEASURES);
		medianRaw = irSensorStepRaw[sensorId][int(NUM_REPEATED_MEASURES / 2)];

		if (false) {
			prt("raw values: ");
			for (int i = 0; i < NUM_REPEATED_MEASURES; i++) {
				pr(irSensorStepRaw[sensorId][i]); prt(", ");
			}
			prt("range: ");
			pr(irSensorStepRaw[sensorId][NUM_REPEATED_MEASURES-1]-irSensorStepRaw[sensorId][0]);
			prt(", median: "); pr(medianRaw);
			prl();
		}

		// calculate mm from raw median for the sensortype used
		if (irSensorDefinitions[sensorId].sensorType == A41) {
			distMm = analogToDistanceA41(medianRaw);
		}
		if (irSensorDefinitions[sensorId].sensorType == A21) {
			distMm = analogToDistanceA21(GP2Y0A21YK, medianRaw);
		}

		prt("sensorId: "); pr(sensorId); prt(", step: "); pr(step); 
		prt(", distMm: "); pr(distMm); prt(", refDist: "); prl(refDist);

		// check for valid distance and calculate obstacleHeight / abyssDepth
		if (distMm == DISTANCE_UNKNOWN) {
			// set distance to reference value, results in 0 obstacleHeight / 0 abyssDepth
			distMm = refDist; 
		}
		// set factor to calculate height/depth from distance
		heightFactor = irSensorDefinitions[sensorId].swipe ? cos30: 1;

		// set obstacle or abyss value
		if (distMm < refDist) obstacleHeight = (refDist - distMm) * heightFactor;	// always a positive value
		if (distMm > refDist) abyssDepth = (distMm - refDist) * heightFactor;		// always a positive value

		// update data
		//irSensorStepValuesType irSensorSwipeData[IR_SENSORS_COUNT][NUM_MEASURE_STEPS];
		irSensorStepData[sensorId][step].distMm = distMm;
		irSensorStepData[sensorId][step].obstacleHeight = obstacleHeight;
		irSensorStepData[sensorId][step].abyssDepth = abyssDepth;

		// in sensor test and verbose mode print the summary
		if (moveType == SENSORTEST && verbose) {		// in sensor test and verbose mode 
			pr(getIrSensorName(sensorId));
			prt(", swipeStep: "); pr(step);
			prt(", distance: "); pr(distMm);
			prt(", reference: "); pr(irSensorReferenceDistances[sensorId][step]);
			prt(", diff: "); pr(irSensorReferenceDistances[sensorId][step] - distMm);
			prl();
		}
	}

	// log the floor offset of involved sensors
	if (plannedCartMovement != STOP) {

		for (int item = 0; item < numInvolvedIrSensors; item++) {
			sensorId = involvedIrSensors[item];

			// check for last or first position in swipe
			if ((currentMeasureStep == NUM_MEASURE_STEPS) 
				|| currentMeasureStep == 0) {

				// in case of a sensor test log the measured distances
				if (moveType == SENSORTEST && sensorId == sensorInTest) {
					logIrDistanceValues(sensorId);
				}

				// do not log the first summary after the command start
				if (millis() - moveRequestReceivedMillis > 500) {

					prt("swipe summary: "); pr(getIrSensorName(sensorId));
					prt(", sensor obstacle max: "); pr(irSensorObstacleMaxValue);
					prt(", sensor abyss max: "); pr(irSensorAbyssMaxValue);
					prl();
				}
			}
		}
	}
}


void logMeasureStepResults() {

	int sensorId;
	int step;

	// !F1,[<sensorId>,<step>,<obstacleHeight>,<abyssDepth>]*involvedSensors
	pr("!F1");
	for (int item = 0; item < numInvolvedIrSensors; item++) {
		sensorId = involvedIrSensors[item];
		step = irSensorDefinitions[sensorId].swipe ? currentMeasureStep : 0;
		pr(","); pr(sensorId);
		pr(","); pr(step);
		pr(","); pr(irSensorStepData[sensorId][step].obstacleHeight);
		pr(","); pr(irSensorStepData[sensorId][step].abyssDepth);
	}
	prl();
}


// set max obstacle/abyss of involved sensors in all swipe steps
void evalIrSensorsMaxValues() {

	// update max values of obstacle and abyss with new data
	irSensorObstacleMaxValue = 0;
	irSensorAbyssMaxValue = 0;
	irSensorObstacleMaxSensor = -1;
	irSensorAbyssMaxSensor = -1;
	int sensorId;

	// for all involved sensors
	for (int item = 0; item < numInvolvedIrSensors; item++) {
		sensorId = involvedIrSensors[item];

		// swiping sensors have a number of swipe step results, static sensors only 1
		int numSteps = irSensorDefinitions[sensorId].swipe? NUM_MEASURE_STEPS : 1;

		//prt("sensor: "); pr(getIrSensorName(sensorId));
		//prt(", numSteps: "); pr(numSteps); prl();
		
		for (int step = 0; step < numSteps; step++) {

			//pr("step: "); pr(step); pr(", ");
			//pr("h: "); pr(irSensorStepData[sensorId][step].obstacleHeight);pr(", ");
			//pr("d: "); pr(irSensorStepData[sensorId][step].abyssDepth); pr(", ");
			//pr("t: "); pr(millis() - irSensorStepData[sensorId][step].lastMeasureMillis);
			//prl();

			// do not use outdated values
			if (millis() - irSensorStepData[sensorId][step].lastMeasureMillis > 2000) continue;

			if (irSensorStepData[sensorId][step].obstacleHeight > irSensorObstacleMaxValue) {
				irSensorObstacleMaxValue = irSensorStepData[sensorId][step].obstacleHeight;
				irSensorObstacleMaxSensor = sensorId;
			}

			if (irSensorStepData[sensorId][step].abyssDepth > irSensorAbyssMaxValue) {
				irSensorAbyssMaxValue = irSensorStepData[sensorId][step].abyssDepth;
				irSensorAbyssMaxSensor = sensorId;
			}
		}
	}
	pr(millis() - moveRequestReceivedMillis); pr(" ms, ");
	prt("evalIrSensorsMaxValues, "); 
	if (irSensorObstacleMaxSensor == -1) {
		prt(", no obstacle detected");
	}
	else {	
		prt("max obstacle: "); pr(irSensorObstacleMaxValue);
		prt(", sensor: "); pr(getIrSensorName(irSensorObstacleMaxSensor));
	}
	if (irSensorAbyssMaxSensor == -1) {
		prt(", no abyss detected");
	}
	else {	
		prt(", max abyss: "); pr(irSensorAbyssMaxValue);
		prt(", sensor: "); pr(getIrSensorName(irSensorAbyssMaxSensor));
	}
	prl();
}



/////////////////////////////
void logIrDistanceValues(int sensorId) {

	//int distance;

	// check type of sensor, swiping or fixed
	int numMeasureSteps = irSensorDefinitions[sensorId].swipe ? NUM_MEASURE_STEPS : 1;

	// !F3,<sensorId>,<NUM_MEASURE_STEPS>,[<distanceValues>,]
	Serial.print("!F3,"); Serial.print(sensorId);
	Serial.print(","); Serial.print(numMeasureSteps);

	// add the measured distances
	for (int swipeStep = 0; swipeStep < numMeasureSteps; swipeStep++) {
		Serial.print(","); Serial.print(irSensorStepData[sensorId][swipeStep].distMm);
	}
	Serial.println();
}



void resetServo(int servoId) {

	// move servo to start position (min)
	int offset = servoDefinitions[servoId].servoDegreeOffset;
	swipeServos[servoId].write(swipeServoStartAngle + offset);

	// will cause first move to servo reset position where it is after a stop
	currentMeasureStep = 0;
	nextMeasureStep = 0;
	swipeDirection = 1;
}


//////////////////////////////////////////
// stop the distance measure swipe
//////////////////////////////////////////
void stopSwipe() {

	for (int servoId = 0; servoId < SWIPE_SERVOS_COUNT; servoId++) {

		resetServo(servoId);
	}

	// allow servos to move to the reset position before detaching them
	delay(100);

	for (int servoId = 0; servoId < SWIPE_SERVOS_COUNT; servoId++) {
		swipeServos[servoId].detach();
	}
	//Serial.println("distance swipe stopped");
}


const char* getIrSensorName(int sensorId) {
	return irSensorDefinitions[sensorId].sensorName;
}


const char* getUsSensorName(int sensorId) {
	return ultrasonicDistanceSensor[sensorId].sensorName;
}


void loadFloorReferenceDistances() {

	//int eepromStartAddr;
	int byteValue;
	//int adjustedValue;

	// load saved reference distances from EEPROM
	Serial.println(F("ir sensor reference distances from eeprom "));
	for (int sensorId = 0; sensorId < IR_SENSORS_COUNT; sensorId++) {
	
		//Serial.print(sensorId); Serial.print(": "); Serial.print(irSensorDefinitions[sensorId].sensorName);
		//Serial.print(": ");
		int eepromStartAddr = sensorId * NUM_MEASURE_STEPS;

		pr("!F2,"); pr(sensorId); pr(","); pr(NUM_MEASURE_STEPS); pr(",");

		for (int swipeStep = 0; swipeStep < NUM_MEASURE_STEPS; swipeStep++) {
			byteValue = EEPROM.read(eepromStartAddr + swipeStep);
			irSensorReferenceDistances[sensorId][swipeStep] = byteValue;
			pr(byteValue); pr(",");
		}
		prl();

	}
}
