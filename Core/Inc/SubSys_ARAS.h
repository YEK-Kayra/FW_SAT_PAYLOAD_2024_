
#ifndef INC_SUBSYS_ARAS_H_
#define INC_SUBSYS_ARAS_H_

#include "main.h"



typedef struct {

int16_t CARR_LandSpeed;
int16_t PAYL_LandSpeed;
uint32_t CARR_Press;
float PAYL_GPS_Lat;
float PAYL_GPS_Long;
float PAYL_GPS_Alt;
uint8_t Seperation_State;

}Aras_Check_HandleTypeDef;

extern float  MS5611_VertSpeed;			/*! Vertical Speed data variable  			     */
extern float  GPS_Altitude;				/*! Vertical distance info of satellite beetween */
extern float  GPS_Longitude;			/*! Location info of satellite on the earth 	 */
extern float  GPS_Latitude;				/*! Location info of satellite on the earth 	 */
extern float  CarrierPressure;			/*! Carrier Unit's pressure data				 */
/**
 * Releated byte will be ==> xxxx xxxx
 * index:
 * 	 	 0-> Satt. velocity is not range of 12-14 m/s.
 * 		 1-> Payload velocity is not range of 6-8 m/s.
 * 		 2-> Unable to get carrier pressure
 * 		 3-> Unable to get payload location
 * 		 4-> Autonomous separation did not occur
 */
extern uint8_t SatelliteErrorCode;

extern uint8_t AutonomoSeparationStatus;


void ARAS_CheckSystem();

void SubSys_ArasCntrl_Sat_LandingSpeed(void);
void SubSys_ArasCntrl_Payload_LandingSpeed(void);
void SubSys_ArasCntrl_Carr_Pressure(void);
void SubSys_ArasCntrl_Payload_LocationData(void);
void SubSys_ArasCntrl_Sat_SeperationCheck(void);

#endif /* INC_SUBSYS_ARAS_H_ */
