// 
// 
// 

#include "cartControlArduino.h"
#include "table.h"
#include "bno055.h"
#include "drive.h"

#define SPEED_INCREMENT 3

TABLE_STATUS tableStatus = TABLE_STOP;

int speed1Up = 220;
int speed2Up = 200;
int speed1Down = 220;
int speed2Down = 180;
long _lastProgressCheckMillis;
int _prevTableHeight;
long _tableMoveStart;
bool tableVerbose = false;



void updateSensorSetPoints(int thisTableHeight) {
	int setPointDepthcam = thisTableHeight / 20 + 20 - 6;
}


void tableSetup()
{
	Serial.println(F("table setup"));
	analogWrite(PIN_TABLE_PWM1, 0);
	analogWrite(PIN_TABLE_PWM2, 0);

	pinMode(PIN_TABLE_DIR1, OUTPUT);
	pinMode(PIN_TABLE_DIR2, OUTPUT);

	pinMode(PIN_TABLE_POWER, OUTPUT);
	digitalWrite(PIN_TABLE_POWER, SWITCH_OFF);

	int tableHeight = getTableHeight();
	Serial.print(F("table setup done, current height: "));
	Serial.println(tableHeight);

	updateSensorSetPoints(tableHeight);
	Serial.println(F("table setup done"));
}

TABLE_STATUS getTableStatus() {
	return tableStatus;
}


int getTableHeight() {

	int heightSensorValue = 0;
	int numMeasures = 10;
	for (int i = 0; i < numMeasures; i++) {
		heightSensorValue += analogRead(PIN_TABLE_HEIGHT);
		delay(1);
	}
	// range of pot is 0..1023, table height is 60..90 cm, 90cm = 0
	int raw = heightSensorValue / numMeasures;
	int inverted = 1023 - raw;		// invert
	float mm = 250.0 / 1024.0 * inverted;	// the table height range in mm
	int height = mm + 670;			// the lower height of the table

	
	Serial.print(F("table raw height value: "));
	Serial.print(raw);
	Serial.print(F(", calculated height value: "));
	Serial.print(round(height));
	Serial.println();

	return round(height);
}

// move up until requested height reached or progress stopped
void tableUp() {

	if (tableStatus != UP) {

		tableStatus = UP;

		Serial.println(F("Move table up"));

		digitalWrite(PIN_TABLE_POWER, SWITCH_ON);
		delay(1000);

		digitalWrite(PIN_TABLE_DIR1, TABLE_UP);
		digitalWrite(PIN_TABLE_DIR2, TABLE_UP);
	}

	analogWrite(PIN_TABLE_PWM1, speed1Up);
	analogWrite(PIN_TABLE_PWM2, speed2Up);
	delay(100);

	while (millis() - _tableMoveStart < 5000 
		&& _currentTableHeight < _requestedTableHeight
		&& tableStatus == UP) {


		// get current roll of table
		platformImu.changedBnoSensorData();

		// compensate for motor speed differences using the bno055 roll value
		if (platformImu.getRoll() > 0.25) {

			// increase speed2 if possible
			if (speed2Up < 230) {
				speed2Up += SPEED_INCREMENT;
				if (tableVerbose) Serial.println(F("increase speed2Up"));
			}
			// else reduce speed1 but limit it
			if (speed1Up > 150) {
				speed1Up -= SPEED_INCREMENT;
				if (tableVerbose) Serial.println(F("decrease speed1Up"));
			}
		}
		if (platformImu.getRoll() < -0.25) {

			// increase speed1 if possible
			if (speed1Up < 230) {
				speed1Up += SPEED_INCREMENT;
				if (tableVerbose) Serial.println(F("increase speed1Up"));
			}
			if (speed2Up > 150) {
				speed2Up -= SPEED_INCREMENT;
				if (tableVerbose) Serial.println(F("decrease speed2Up"));
			}
		}
		if (tableVerbose) {
			Serial.print(F("table up, roll: "));
			Serial.print(platformImu.getRoll());
			Serial.print(F(", speed1Up: "));
			Serial.print(speed1Up);
			Serial.print(F(", speed2Up: "));
			Serial.print(speed2Up);
			Serial.println();
		}
		analogWrite(PIN_TABLE_PWM1, speed1Up);
		analogWrite(PIN_TABLE_PWM2, speed2Up);

	
		delay(50);
		
		_currentTableHeight = getTableHeight();
		
		// check for progress
		long currMillis = millis();
		if (currMillis - _lastProgressCheckMillis > 500L) {

			if (abs(_prevTableHeight - _currentTableHeight) < 10) {
				Serial.println(F("table up, no progress detected, stop"));
				tableStop();
			}
			_prevTableHeight = _currentTableHeight;
			_lastProgressCheckMillis = millis();
		}
	}
	// stop move
	if (millis() - _tableMoveStart >= 5000) {
		Serial.print(F("current timeout for move is 5s"));
	}
	if (_currentTableHeight >= _requestedTableHeight) {
		Serial.print(F("requested height reached"));
	}
	Serial.println();
	tableStop();
}


// move down until requested height reached or progress stopped
void tableDown() {

	if (tableStatus != DOWN) {

		tableStatus = DOWN;

		Serial.println(F("Table down"));

		digitalWrite(PIN_TABLE_POWER, SWITCH_ON);
		delay(50);

		digitalWrite(PIN_TABLE_DIR1, TABLE_DOWN);
		digitalWrite(PIN_TABLE_DIR2, TABLE_DOWN);
	}

	
	//while ((_currentTableHeight > _requestedTableHeight) && (tableStatus == DOWN)) {
	analogWrite(PIN_TABLE_PWM1, speed1Down);
	analogWrite(PIN_TABLE_PWM2, speed2Down);
	delay(100);

	while (millis() - _tableMoveStart < 5000 
		&& _currentTableHeight > _requestedTableHeight
		&& tableStatus == DOWN) {

		// get current roll of table
		platformImu.changedBnoSensorData();

		// compensate for motor speed differences using the bno055 roll value
		if (platformImu.getRoll() > 0.25) {

			// increase speed1 if possible
			if (speed1Down < 230) {
				speed1Down += SPEED_INCREMENT;
				if (tableVerbose) Serial.println(F("increase speed1"));
			}
			// and reduce speed2
			if (speed2Down > 150) {
				speed2Down -= SPEED_INCREMENT;
				if (tableVerbose) Serial.println(F("decrease speed2"));
			}
		}
		if (platformImu.getRoll() < -0.25) {

			if (speed2Down < 230) {
				speed2Down += SPEED_INCREMENT;
				if (tableVerbose) Serial.println(F("increase speed2"));
			}
			// and reduce speed1
			if (speed1Down > 150) {
				speed1Down -= SPEED_INCREMENT;
				if (tableVerbose) Serial.println(F("decrease speed1"));
			}

		}

		analogWrite(PIN_TABLE_PWM1, speed1Down);
		analogWrite(PIN_TABLE_PWM2, speed2Down);

		if (tableVerbose) {
			Serial.print(F(", table down, roll: "));
			Serial.print(platformImu.getRoll());
			Serial.print(F(", speed1Down: "));
			Serial.print(speed1Down);
			Serial.print(F(", speed2Down: "));
			Serial.print(speed2Down);
			Serial.println();
		}
		delay(50);

		_currentTableHeight = getTableHeight();

		// check for progress
		long currMillis = millis();
		if (currMillis - _lastProgressCheckMillis > 500L) {

			if (_prevTableHeight - _currentTableHeight < 2) {
				Serial.println(F("table down, no progress detected, stop"));
				tableStop();
			}
			_prevTableHeight = _currentTableHeight;
			_lastProgressCheckMillis = millis();
		}
	}
	// stop move
	if (millis() - _tableMoveStart >= 5000) {
		Serial.print(F("current timeout for move is 5s"));
	}
	if (_currentTableHeight < _requestedTableHeight) {
		Serial.print(F("requested height reached"));
	}
	Serial.println();
	tableStop();
}


void tableStop() {

	if (tableStatus != TABLE_STOP) {

		tableStatus = TABLE_STOP;
		Serial.println(F("Table stop")); 

		analogWrite(PIN_TABLE_PWM1, 0);
		analogWrite(PIN_TABLE_PWM2, 0);
		digitalWrite(PIN_TABLE_POWER, SWITCH_OFF);

		updateSensorSetPoints(getTableHeight());
	}
}



void moveTableToHeight(int requestedHeight) {

	Serial.print(F("moveTableToHeight: "));
	Serial.print(requestedHeight);
	Serial.println();

	// check for valid request range
	if (requestedHeight > 930) {
		Serial.println(F("max height is 930, command ignored"));
		return;
	}

	if (requestedHeight < 600) {
		Serial.println(F("min height is 600, command ignored"));
		return;
	}

	// check for valid current range
	_currentTableHeight = getTableHeight();
	if ((_currentTableHeight > 930) || (_currentTableHeight < 600)) {
		Serial.print(F("moveTable, not a valid current height"));
		Serial.print(_currentTableHeight);
		Serial.println();
		return;
	}

	// check for needed move
	_currentTableHeight = getTableHeight();
	_prevTableHeight = _currentTableHeight;
	_tableMoveStart = millis();
	_lastProgressCheckMillis = millis() + 1000;	// delay first progress check

	if (requestedHeight > _currentTableHeight) {
		Serial.println(F("table up"));
		tableUp();
	}
	else {
		Serial.println(F("table down"));
		tableDown();
	}
}


