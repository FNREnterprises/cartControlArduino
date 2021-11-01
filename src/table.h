// Table.h

#ifndef _TABLE_h
#define _TABLE_h

//#include "arduino.h"

#include "cartControlArduino.h"

extern int _currentTableHeight;
extern int _requestedTableHeight;
extern TABLE_STATUS _tableStatus;

void tableSetup();
int getTableHeight();
TABLE_STATUS getTableStatus();
void tableUp();
void tableDown();
void tableStop();

void moveTableToHeight(int height);

#endif

