#include "SubSys_Sensor_Battery_Driver.h"

uint16_t Value_ADC;
float maxVoltage;
float CriticalVoltageLimit;
float ConstantOfVoltage;
float MinLimitVoltage;

void  MeasBattery_Init(int NumSerialBat){

	maxVoltage = NumSerialBat * 4.2 ;

	ConstantOfVoltage = maxVoltage / ConstantOfReferanceVoltage;

	MinLimitVoltage = 3.5 * NumSerialBat;

}


/**
 * @fn float ReadingBatteryVoltage(ADC_HandleTypeDef *hadc)
 * @brief Read value of battery voltage by using ADC
 * @param hadc
 * @retval Value of Battery Voltage
 */

float ReadBatteryVoltage(ADC_HandleTypeDef *hadc){

   HAL_ADC_Start(hadc);

   if(HAL_ADC_PollForConversion(hadc, 1000)==HAL_OK)
   {

      Value_ADC = HAL_ADC_GetValue(hadc);
      BatteryVoltage = (Value_ADC * (ConstantOfReferanceVoltage / ResolationValueOfBits)  *  ConstantOfVoltage);

         if(BatteryVoltage <= MinLimitVoltage)
         {
        	PassiveBuzz_ON(&htim3, TIM_CHANNEL_1);		/*! System voltage level is within the critical range, alert is active */
        	while(1);
         }

   }

   HAL_ADC_Stop(hadc);
   return BatteryVoltage;
}
