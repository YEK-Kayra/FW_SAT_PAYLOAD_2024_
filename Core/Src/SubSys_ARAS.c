#include "SubSys_ARAS.h"


Aras_Check_HandleTypeDef CheckAras;
uint8_t total_err= 0;
/* !Hata durumunun yazılacağı değişken tanımlanır Başlangıç <00000>*/




void ARAS_CheckSystem()
{

	CheckAras.PAYL_LandSpeed  = MS5611_VertSpeed;
	CheckAras.CARR_Press      = CarrierPressure;	   //lora ile gelen veriden alınır
	CheckAras.PAYL_GPS_Lat 	  = GPS_Latitude ;
	CheckAras.PAYL_GPS_Long   = GPS_Longitude;
	CheckAras.PAYL_GPS_Alt    = GPS_Altitude;
	CheckAras.Seperation_State= AutonomoSeparationStatus; // Ayrılma yapılırsa high(1) yapılamadıysa low(0) olcak


	SubSys_ArasCntrl_Sat_LandingSpeed();       //Uydu iniş hızı 12-14m/s; değilse = 1xxxx , ise = 0xxxx

	SubSys_ArasCntrl_Payload_LandingSpeed();	  //Görev yükü iniş hızı 6-8m/s; değilse = x1xxx , ise = x0xxx

	SubSys_ArasCntrl_Carr_Pressure();		  //Taşıyıcıdan basınç verisi; alınamıyorsa = xx1xx , alınıyorsa = xx0xx

	SubSys_ArasCntrl_Payload_LocationData();   //Görev yükü konum verisi; alınamıyorsa = xxx1x , alınıyorsa = xxx0x

	SubSys_ArasCntrl_Sat_SeperationCheck();		  // Ayrılma; gerçekleşmediyse = xxxx1 , gerçekleştiyse = xxxx0

	/* !Son değerini alan "toplam" değişkeni fonksiyonun geri dönüş değeri olarak yazılır*/
	SatelliteErrorCode = total_err;
}


/*!
 * @brief Model Satellite (Payload & Carrier) Descent Speed Status
 */
void SubSys_ArasCntrl_Sat_LandingSpeed(void)
{


	if( ((12 <= CheckAras.PAYL_LandSpeed) && (CheckAras.PAYL_LandSpeed <= 14)) )
	{
		total_err += 0;
	}else{
		total_err += 16;
	}

}

/*!
 * @brief Payload Descent Speed Status
 */
void SubSys_ArasCntrl_Payload_LandingSpeed(void)
{
	if( ((6 <= CheckAras.PAYL_LandSpeed) && (CheckAras.PAYL_LandSpeed <= 8)) )
	{
		total_err += 0;
	}else{
		total_err += 8;
	}
}

/*!
 * @brief Carrier Open Air Pressure Status
 */
void SubSys_ArasCntrl_Carr_Pressure(void)
{

	if( (((CheckAras.CARR_Press)!=0) && ((CheckAras.CARR_Press)<=101325)) )
	{
		total_err += 0;
	}else{
		total_err += 4;
	}
}

/*!
 * @brief Payload Location Status
 */
void SubSys_ArasCntrl_Payload_LocationData(void)
{
	if( ((CheckAras.PAYL_GPS_Lat)!=0) && ((CheckAras.PAYL_GPS_Long)!=0) && ((CheckAras.PAYL_GPS_Alt)!=0) )
	{
		total_err += 0;
	}else{
		total_err += 2;
	}

}

/*!
 * @brief Model Satellite Separation Status
 */
void SubSys_ArasCntrl_Sat_SeperationCheck(void)
{
	if(CheckAras.Seperation_State == 1)
	{
		total_err += 0;
	}else{
		total_err += 1;
	}

}
