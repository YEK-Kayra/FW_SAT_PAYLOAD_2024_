/*
 * SubSys_Sensor_Battery_Driver.h
 *
 *  Created on: Jul 19, 2024
 *      Author: yunus
 */

#ifndef SAT_PAYLOAD_SUBSYS_DRIVERS_SENSOR_BATTERY_H
#define SAT_PAYLOAD_SUBSYS_DRIVERS_SENSOR_BATTERY_H


/******************************************************************************
         			#### INCLUDES ####
******************************************************************************/

#include "main.h"
#include "stdio.h"
#include "stm32f4xx_hal_tim.h"


/******************************************************************************
         			#### DEFINITIONS ####
******************************************************************************/
#define ConstantOfReferanceVoltage 3.30
#define ResolationValueOfBits 4095


/******************************************************************************
         			#### EXTERNAL VARIABLES ####
******************************************************************************/
extern float BatteryVoltage;
extern uint8_t NumSerialBat;
extern TIM_HandleTypeDef htim3;	/*! Passive Buzzer PWM timer*/


/******************************************************************************
         			#### FUNCTIONS ####
******************************************************************************/
void  MeasBattery_Init(int NumSerialBat);
float ReadBatteryVoltage(ADC_HandleTypeDef *hadc);

#endif /* SAT_CARRIER_SUBSYS_DRIVERS_SUBSYS_INC_SUBSYS_SENSOR_BATTERY_DRIVER_H_ */
