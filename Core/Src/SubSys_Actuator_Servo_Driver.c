
#include "SubSys_Actuator_Servo_Driver.h"


/**
 * @brief Servo motor parameters will be initialized here
 * @param *dev,  Connect struct parameter htim_X, tim_channel_in as given below
 *
 * @retval none
 */
extern TIM_HandleTypeDef htim1;
void SubSys_Actuator_Servo_Init(Actuator_Servo_HandleTypeDef *dev){

	HAL_TIM_Base_Start(dev->htim_X);
	HAL_TIM_PWM_Start(dev->htim_X,dev->tim_channel_in);

}


/**
 * @brief Servo motor parameters will be DeInitialized here
 * @param *dev,  Connect struct parameter htim_X, tim_channel_in as given below
 *
 * @retval none
 */
void SubSys_Actuator_Servo_DeInit(Actuator_Servo_HandleTypeDef *dev){

	HAL_TIM_Base_Stop(dev->htim_X);
	HAL_TIM_PWM_Stop(dev->htim_X,dev->tim_channel_in);

}


/**
 * @brief   The servo motor will turn according to the angle parameter.
 * 		   The MG90S servo motor's datasheet is slightly incorrect regarding the duty cycle.
 * 		   The correct duty cycle range is between 2% and 12%.
 *
 * @param *dev, Connect struct parameter htim_X, tim_channel_in as given below
 * @param Angle The desired angle for the motor to turn.
 */
void SubSys_Actuator_Servo_MoveTo(Actuator_Servo_HandleTypeDef *dev, uint16_t Angle){

	if(dev == &dev_Servo_Separation){
		dev->htim_X->Instance->CCR2 = (((Angle*100)/180)+20);
	}

	if(dev == &dev_Servo_ColorFilter){
		dev->htim_X->Instance->CCR1 = (((Angle*100)/180)+20);
	}

}
