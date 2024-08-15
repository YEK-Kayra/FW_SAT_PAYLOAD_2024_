#include "SubSys_ColorFilterControl_Driver.h"


void SubSys_ColorFilterMechanism_TurnTo(SubSys_ColorFilter_MagazineColor Color){

	switch(Color)
	{

		case Filter_None :
			SubSys_Actuator_Servo_MoveTo(&dev_Servo_ColorFilter, 0);
			break;

		case filter_Red :
			SubSys_Actuator_Servo_MoveTo(&dev_Servo_ColorFilter, 60);
			break;

		case filter_Green :
			SubSys_Actuator_Servo_MoveTo(&dev_Servo_ColorFilter, 120);
			break;

		case filter_Blue :
			SubSys_Actuator_Servo_MoveTo(&dev_Servo_ColorFilter, 180);
			break;

	}

}
