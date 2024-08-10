/*
 * SubSys_SeparationControl_Driver.h
 *
 *  Created on: Aug 5, 2024
 *      Author: yunus
 */

#ifndef SAT_PAYLOAD_SUBSYS_DRIVERS_SEPARATION_CONTROL_H
#define SAT_PAYLOAD_SUBSYS_DRIVERS_SEPARATION_CONTROL_H

#include "main.h"

typedef struct
{

	TIM_HandleTypeDef *htim_X;
	uint32_t tim_ch_in;


}SeperationCntrl_HandleTypeDef;

void SepSys_Init(SeperationCntrl_HandleTypeDef *SeperationDev);

void SepSys_Servo_MoveTo(SeperationCntrl_HandleTypeDef *SeperationDev, uint16_t Angle);

void SepSys_UnlockMech(SeperationCntrl_HandleTypeDef *SeperationDev);
void SepSys_lockMech(SeperationCntrl_HandleTypeDef *SeperationDev);


#endif /* INC_SUBSYS_SEPARATIONCONTROL_DRIVER_H_ */
