
#ifndef SAT_PAYLOAD_SUBSYS_DRIVERS_ACTUATOR_SERVO_H
#define SAT_PAYLOAD_SUBSYS_DRIVERS_ACTUATOR_SERVO_H

#include "main.h"

typedef struct
{

	TIM_HandleTypeDef *htim_X;
	uint32_t tim_channel_in;
	__IO uint32_t CCRx;

}Actuator_Servo_HandleTypeDef;

void Actuator_Servo_Init(Actuator_Servo_HandleTypeDef *dev);


#endif /* INC_SUBSYS_ACTUATOR_SERVO_DRIVER_H_ */
