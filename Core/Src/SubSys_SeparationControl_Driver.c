#include "SubSys_SeparationControl_Driver.h"

extern TIM_HandleTypeDef htim1;


/**
 *
 */
void SepSys_Init(SeperationCntrl_HandleTypeDef *SeperationDev){

SeperationDev->htim_X = &htim1; // init dışına yazılacak
SeperationDev->tim_ch_in = TIM_CHANNEL_1; // init dışına yazılacak

HAL_TIM_Base_Start(SeperationDev->htim_X);
HAL_TIM_PWM_Start(SeperationDev->htim_X, SeperationDev->tim_ch_in);


SeperationDev->htim_X->Instance->CCR1 = 30; //0derece de durmal� offset i�in//30/////16
HAL_Delay(1000);

}

//Angle de�eri 0-180 de�eri aras�nda olacak.
void SepSys_Servo_MoveTo(SeperationCntrl_HandleTypeDef *SeperationDev, uint16_t Angle){

	/*!	((Angle + 60)/2) = CCRx value that we have to write  */
	SeperationDev->htim_X->Instance->CCR1 = ((Angle + 60)/2);
	HAL_Delay(150);

}

void SepSys_UnlockMech(SeperationCntrl_HandleTypeDef *SeperationDev){//75

	SeperationDev->htim_X->Instance->CCR1 = 75; //100   90dereece de durmal� buras� sanki ta��y�c� g�rev y�k�n� b�rakm�� gibi d���n
	HAL_Delay(100);

}

void SepSys_lockMech(SeperationCntrl_HandleTypeDef *SeperationDev){
	SeperationDev->htim_X->Instance->CCR1 = 120; //120   180dereceye gitmeli burada da g�rev y�k�n� ta��y�c�ya koymu�uz kilitleme komutu veriyoruz gibi d���n
	HAL_Delay(100);
}

