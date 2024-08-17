
#ifndef INC_SUBSYS_WIRELESSCOMMUNICATION_TELEMETRY_EXTRACTVALUE_DRIVER_H_
#define INC_SUBSYS_WIRELESSCOMMUNICATION_TELEMETRY_EXTRACTVALUE_DRIVER_H_

#include "main.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

void extractValues_Carrier(const char* input, char* value1, char* value2);
void extractValues_GroundStation(const char* input, char* value3);

#endif /* INC_SUBSYS_WIRELESSCOMMUNICATION_TELEMETRY_EXTRACTVALUE_DRIVER_H_ */
