// 
// 
// 
#include <Arduino.h>
#include <EEPROM.h>

#include "cartControlArduino.h"
#include "communication.h"
#include "drive.h"
#include "bno055.h"
#include "distance.h"

// heartbeat
unsigned long lastMsg;

char msgCopyForParsing[64];	// once a new message is received, parse it from a copy of the receiver buffer
char receivedChars[64]; // an array to store the received data
int numChars = 0;
bool newData = false;

// fill buffer with message
// set newData flag to true, processing in motorizedBase.cpp

void sendImuValues(Imu imu) {

	long milliYaw = imu.getYaw() * 1000;
	long milliRoll = imu.getRoll() * 1000;
	long milliPitch = imu.getPitch() * 1000;

	pr(imu.getId()); pr(","); pr(milliYaw); pr(","); pr(milliRoll); pr(","); pr(milliPitch); prl();

	imu.setMillisLastPublished();
}


void recvWithEndMarker() {
	static byte ndx = 0;
	char endMarker = '\n';
	char rc;

	while (Serial.available() > 0 && newData == false) {
		rc = Serial.read();

		if (rc != endMarker) {
			receivedChars[ndx] = rc;
			ndx++;
			if (ndx >= 60) {
				ndx = 60 - 1;
			}
			numChars = ndx;
		}
		else {
			receivedChars[ndx] = '\0'; // terminate the string
			ndx = 0;
			newData = true;
			cartControlActive = true;
			lastMsg = millis();
			//Serial.print(F("message received, millis: "));
			//Serial.print(millis()); Serial.print(F(" lastMsg: "));
			//Serial.println(lastMsg);
		}
	}
}


void checkCommand() {

	//int dir;
	//int speed;
	int newSpeed;
	//double ox;
	//int servoId;
	int sensorId;
	//int testDirection;
	//char msgCopyForParsing[64];
	char * strtokIndx; // this is used by strtok() as an index

	int thisDir;
	int thisRequestedDistance;
	int thisRequestedRelAngle;
	int thisRequestedMaxSpeed;
	int thisMaxMoveDuration;
	//int thisFlagProtected;
	bool thisMoveProtected;

	int eepromStartAddr;
	int distance;

	MOVEMENT cartDirection;
	int testDuration = 1000;

	if (Serial.available() > 0) {
		recvWithEndMarker();
		//standalone: SerialMonitor, select Line Feed and send command, 
		// e.g. 1,1,200,2000 for go forward 2000 mm with speed 200
	}

	if (newData) {

		char mode = receivedChars[0];
		strncpy(msgCopyForParsing, receivedChars, numChars);

		// do not log the heartbeat and the orientation query
		if (mode != '5' && mode != '6' && mode != '9') {
			Serial.println((String) "new command, mode: " + mode + ", msg: " + receivedChars);
		}

		///////////////////////////////////////////////
		///////////////////////////////////////////////
		// if switch is not working check for new variable assignments within switch section and remove them
		///////////////////////////////////////////////
		///////////////////////////////////////////////
		//DECLARE ALL NEW VARIABLES USED IN SWITCH HERE OR USE {} for the switch block
		// switch (x) {
		//	case 1: {
		//		int y = 5;
		//		break;
		//	}
		//}
		int speedPercentage;
		MOVEMENT moveDirectionForSensorTest[] = {FORWARD,FORWARD,FORWARD,BACKWARD,BACKWARD,BACKWARD,LEFT,LEFT,RIGHT,RIGHT};
		int swipeStep;
		//int byteValue;

		//DECLARE ALL NEW VARIABLES USED IN SWITCH BEFORE THE SWITCH
		// or use brackets in case section

		switch (mode) {		
			
		case '1':  // move dir 1=forward .. 6=backward, e.g. 1,<dir d1>,<speed d3>,<distance d4>,<max duration d4>
			{
			// new move commands are only accepted when not in current move
			if (plannedCartMovement != STOP) {
				prtl("ignoring new move command while in move");
				return;
			}
			strtokIndx = strtok(msgCopyForParsing, ","); // msgId, "1"

			strtokIndx = strtok(NULL, ","); // direction
			thisDir = atoi(strtokIndx);

			strtokIndx = strtok(NULL, ","); // speed
			thisRequestedMaxSpeed = atoi(strtokIndx);

			strtokIndx = strtok(NULL, ","); // distance
			thisRequestedDistance = atoi(strtokIndx);

			strtokIndx = strtok(NULL, ","); // duration
			thisMaxMoveDuration = atoi(strtokIndx);

			strtokIndx = strtok(NULL, ","); // flagProtected
			thisMoveProtected = atoi(strtokIndx) == 1;

			thisRequestedRelAngle = 0;
			moveType = STRAIGHT;

			setPlannedCartMove(MOVEMENT(thisDir), thisRequestedDistance, thisRequestedRelAngle, thisRequestedMaxSpeed, thisMaxMoveDuration, thisMoveProtected);

			break;
			}

		case '2': { // rotate counterclockwise or left: 3 digits for relative angle, angle given in positive degrees
					// 2,<angle>,<speed>,<maxDuration>
			if (plannedCartMovement != STOP) {
				prtl("ignoring new rotate command while in move");
				return;
			}

			strtokIndx = strtok(msgCopyForParsing, ","); // msgId, "2"

			strtokIndx = strtok(NULL, ","); // angle
			thisRequestedRelAngle = atoi(strtokIndx);
			
			strtokIndx = strtok(NULL, ","); // speed
			thisRequestedMaxSpeed = atoi(strtokIndx);
			
			strtokIndx = strtok(NULL, ","); // duration
			thisMaxMoveDuration = atoi(strtokIndx);

			thisRequestedDistance = 0;

			moveType = ROTATE;
			setPlannedCartMove(ROTATE_COUNTERCLOCK, thisRequestedDistance, thisRequestedRelAngle, thisRequestedMaxSpeed, thisMaxMoveDuration, true);

			break;
		}

		case '3': { // rotate clockwise or right, 3 digits for relative angle, angle given in positive degrees
					// 3,<angle>,<speed>,<maxDuration>

			if (plannedCartMovement != STOP) {
				prtl("ignoring new rotate command while in move");
				return;
			}

			strtokIndx = strtok(msgCopyForParsing, ","); // msgId, "3"

			strtokIndx = strtok(NULL, ","); // angle
			thisRequestedRelAngle = atoi(strtokIndx);

			strtokIndx = strtok(NULL, ","); // speed
			thisRequestedMaxSpeed = atoi(strtokIndx);

			strtokIndx = strtok(NULL, ","); // duration
			thisMaxMoveDuration = atoi(strtokIndx);

			thisRequestedDistance = 0;
			moveType = ROTATE;

			setPlannedCartMove(ROTATE_CLOCKWISE, thisRequestedDistance, thisRequestedRelAngle, thisRequestedMaxSpeed, thisMaxMoveDuration, true);

			break;
		}

		case '4':  // stop
			stopCart(true, "stop requested by cartControl");
			break;


		case '5':  // getCartOrientation
				   // !A9,<orientation>
			//platformImu.changedBnoSensorData();
			sendImuValues(platformImu);

			//headImu.changedBnoSensorData();
			sendImuValues(headImu);

			break;

		case '6':  // set new speed

			strtokIndx = strtok(msgCopyForParsing, ","); // msgId, "6"

			strtokIndx = strtok(NULL, ","); // sensorId
			newSpeed = atoi(strtokIndx);

			Serial.print(F("set new speed received but ignored: "));
			Serial.print(newSpeed);
			Serial.println();
			break;

		case '7': { // test IR sensor

			if (plannedCartMovement != STOP) {
				prtl("ignoring test sensor command while in move");
				return;
			}

			strtokIndx = strtok(msgCopyForParsing, ","); // msgId, "7"

			strtokIndx = strtok(NULL, ","); // sensorId
			sensorInTest = atoi(strtokIndx); 

			prt("sensorInTest, Id: "); pr(sensorInTest);
			if (sensorInTest < 0 || sensorInTest > 9) {
				prtl(", invalid sensor Id, only 0..9 allowed");
				break;
			}
			prt(", sensorName: "); prl(irSensorDefinitions[sensorInTest].sensorName);

			// set move direction and duration based on sensor to test
			cartDirection = moveDirectionForSensorTest[sensorInTest];
			testDuration = irSensorDefinitions[sensorInTest].swipe ? 5000 : 600;	// do not use 500!!

			moveType = SENSORTEST;
			setPlannedCartMove(cartDirection, 0, 0, 2000, testDuration, true);

			break;
		}

		case '8':  // move table 8,<height>

			strtokIndx = strtok(msgCopyForParsing, ","); // msgId, "8"

			strtokIndx = strtok(NULL, ","); // height
			_requestedTableHeight = atoi(strtokIndx);

			_currentTableHeight = getTableHeight();
			Serial.print(F("request for table height: "));
			Serial.print(_requestedTableHeight);
			Serial.print(F(", current height: "));
			Serial.print(_currentTableHeight);
			Serial.println();
			delay(200);

			moveTableToHeight(_requestedTableHeight);

			break;


		case '9':  // heart beat
			cartControlActive = true;
			break;

		case 'a':	// config values for max obstacle, abyss, delay, finalDockingMoveDistance

			strtokIndx = strtok(msgCopyForParsing, ","); // msgId, "a"

			strtokIndx = strtok(NULL, ",");  // next item
			FLOOR_MAX_OBSTACLE = atoi(strtokIndx);     // convert this part to an integer

			strtokIndx = strtok(NULL, ",");  // next item
			FLOOR_MAX_ABYSS = atoi(strtokIndx);

			strtokIndx = strtok(NULL, ",");  // next item
			NUM_REPEATED_MEASURES = atoi(strtokIndx);
			if (NUM_REPEATED_MEASURES > NUM_REPETITIONS_IR_MEASURE) {
				NUM_REPEATED_MEASURES = NUM_REPETITIONS_IR_MEASURE;
				Serial.print(F("NUM_REPEATED_MEASURES ("));
				Serial.print(NUM_REPEATED_MEASURES);
				Serial.print(F(") limited to MAX_REPEATED_IR_MEASURES ("));
				Serial.print(NUM_REPETITIONS_IR_MEASURE);
				Serial.println();
			}

			strtokIndx = strtok(NULL, ",");  // next item
			DELAY_BETWEEN_ANALOG_READS = atoi(strtokIndx);

			strtokIndx = strtok(NULL, ",");  // next item
			MIN_MEASURE_CYCLE_DURATION = atoi(strtokIndx);

			strtokIndx = strtok(NULL, ",");  // next item
			finalDockingMoveDistance = atoi(strtokIndx);

			break;

		case 'b':	// speed calculation numbers

			strtokIndx = strtok(msgCopyForParsing, ","); // msgId, "d"

			strtokIndx = strtok(NULL, ","); // next item
			speedFactor = atof(strtokIndx);

			strtokIndx = strtok(NULL, ","); // next item			
			speedOffset = atof(strtokIndx);

			strtokIndx = strtok(NULL, ","); // next item			
			speedFactorSideway = atof(strtokIndx);

			strtokIndx = strtok(NULL, ","); // next item			
			speedFactorDiagonal = atof(strtokIndx);

			prt("msg b: cart speed calculation, factor: "); pr(speedFactor); prl();
			prt(" offset: "); pr(speedOffset); prl();
			prt(" factor sideway moves: "); pr(speedFactorSideway); prl();
			prt(" factor diagonal moves: "); pr(speedFactorDiagonal); prl();

			break;


		case 'c':	// speed adjustments for all motors in percent in order fr,fl,br,bl
			
			strtokIndx = strtok(msgCopyForParsing, ","); // msgId, "d"
			
			for (int i = 0; i < MOTORS_COUNT; i++) {
				strtokIndx = strtok(NULL, ","); // next item
				speedPercentage = atoi(strtokIndx);    // convert this part to an integer
				_speedUnifyer[i] = speedPercentage;
			}

			Serial.print(F("msg c: motor speed unifyers (fr,fl,br,bl): "));
			for (int i = 0; i < MOTORS_COUNT; i++) {
				Serial.print(_speedUnifyer[i]);
				Serial.print(",");
			}
			Serial.println();

			break;


		case 'd':	// all configuration data received
			// this enables setup to continue
			configurationComplete = true;
			Serial.println(F("msg d: configuration data complete"));

			break;


		case 'f':	// for given sensor set measured distances as floor distances

			prt("cmd f: set floor reference distances");prl();

			strtokIndx = strtok(msgCopyForParsing, ","); // msgId, "f"

			strtokIndx = strtok(NULL, ","); // next item
			sensorId = atoi(strtokIndx);    // sensorId
			prl(sensorId);
			
			eepromStartAddr = sensorId * NUM_MEASURE_STEPS;

			for (swipeStep = 0; swipeStep < NUM_MEASURE_STEPS; swipeStep++) {
				strtokIndx = strtok(NULL, ","); // next item
				distance = atoi(strtokIndx);    // value

				EEPROM.write(eepromStartAddr + swipeStep, distance);
				irSensorReferenceDistances[sensorId][swipeStep] = distance;
				
			}

			break;


		case 'v':	// set verbose state v,<0/1>
			if (receivedChars[2] == '0') {
				verbose = false;	// serial command
				Serial.println(F("verbose turned off"));
			}
			else {
				verbose = true;		// serial command
				Serial.println(F("verbose turned on"));
			}
			break;


		default:
			Serial.print(F("unknown serial command, mode: ")); Serial.println(char(mode));
			break;
		}

		newData = false;
	}
}
