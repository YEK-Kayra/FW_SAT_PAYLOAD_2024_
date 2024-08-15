#include "SubSys_SeparationControl_Driver.h"


void SubSys_SeparationMechanism_Lock_PayloadToCarrier(){

	SubSys_Actuator_Servo_MoveTo(&dev_Servo_Separation,0);

}
void SubSys_SeparationMechanism_UnLock_PayloadFromCarrier(){

	SubSys_Actuator_Servo_MoveTo(&dev_Servo_Separation,90);

}

