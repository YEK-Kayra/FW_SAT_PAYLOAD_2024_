
#include "SubSys_Payload_FlightStatus.h"


void SubSys_SatelliteMission_Continue(){


	SubSys_Payload_MissionRetryLoop();

	/*! Model Satellite is on the ground*/
	if( ((-2 <= MS5611_VertSpeed) && (MS5611_VertSpeed <= 4)) && (SatelliteStatus == ReadyForLaunch) )
	{
		__NOP();
	}else{ /*! Ascent */
		SatelliteStatus = Ascent;

				if( ((0 <= MS5611_VertSpeed) && (MS5611_VertSpeed <= 3))  && (SatelliteStatus == Ascent) )	//TODO: Euler angles will be added
				{
					SatelliteStatus = ModelSatelliteDescent;
				}

				if( ((-14 <= MS5611_VertSpeed) && (MS5611_VertSpeed <= -12)) && (SatelliteStatus == ModelSatelliteDescent) )
				{
					SatelliteStatus = Separation;
					AutonomoSeparationStatus = Permission_OK;
				}

				if( ((390 <= MS5611_Altitude) && (MS5611_Altitude <= 410)) && (AutonomoSeparationStatus = Permission_OK) && (SatelliteStatus == Separation) )
				{
					SatelliteStatus = PayloadDescent;
					SubSys_SeparationMechanism_UnLock_PayloadFromCarrier();
				}

				if( ((0 <= MS5611_Altitude) && (MS5611_Altitude <= 2)) && (SatelliteStatus == PayloadDescent) )
				{
					SatelliteStatus = Recovery;
					PassiveBuzz_ON(&htim3, TIM_CHANNEL_1);		/*! The payload of the satellite has made contact with the ground */
				}

	     }

}



