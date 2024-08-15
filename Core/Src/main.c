/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  @ATTENTION !
  * 	##Explains CubeMX Variables & IO
  *
  * 	--> ADC_HandleTypeDef hadc1;	 ==> It relates to the 	"SubSys_Sensor_Battery_Driver"
  *
  * 	--> I2C_HandleTypeDef hi2c1;   	 ==> It relates to the 	"SubSys_Sensor_TPGVH_Driver"
  * 	--> I2C_HandleTypeDef hi2c2;   	 ==> It relates to the 	"SubSys_Sensor_IMU_APP_Driver" & "SubSys_Sensor_IMU_STM32_Driver"
  *		--> I2C_HandleTypeDef hi2c3;   	 ==> It relates to the 	"SubSys_Sensor_RTC_Driver"
  *
  * 	--> SD_HandleTypeDef  hsd;	   	 ==> It relates to the 	"SubSys_SDcard_Driver"
  *		--> SDIO_CK frequency 			 ==> 48MHz fixed
  *
  * 	--> UART_HandleTypeDef huart1; 	 ==> It relates to the 	"SubSys_Sensor_GPS_Driver"
  *		--> UART_HandleTypeDef huart2; 	 ==> It relates to the 	"SubSys_WirelessCommunication_Setting_Driver" & "SubSys_WirelessCommunication_Telemetry_Driver"
  *
  *		--> TIM_HandleTypeDef htim1;  	 ==> It relates to the 	"SubSys_SeparationControl_Driver" 	& [(TIM_CHANNEL_2)]
  *		--> TIM_HandleTypeDef htim2;  	 ==> It relates to the 	"SubSys_ColorFilterControl_Driver" 	& [(TIM_CHANNEL_1)](Succes)
  *		--> TIM_HandleTypeDef htim3;  	 ==> It relates to the 	"SubSys_AlertControl_Driver" 		& [(TIM_CHANNEL_1)](Succes)
  *
  *		--> GPIO_TypeDef |PORT->B| -> |GPIO_PIN_13|			    ==>  (CAMERA SWITCH PIN)
  *
  *		--> GPIO_TypeDef |PORT->C| -> |GPIO_PIN_0|			    ==>  (GPS RST PIN)
  *								   -> |GPIO_PIN_1|&|GPIO_PIN_2| ==>  (IMU RST & INT PIN)
  *								   -> |GPIO_PIN_4|&|GPIO_PIN_5| ==>  (WirelessCom M0 & M1 CNFG PIN)
  *
  *		##Wireless Communication configuration parameters
  *
  *		Satellite Carrier Unit's adress will be 0x1923  and channel is 0x10
  *		Satellite Payload Unit's adress will be 0x1453  and channel is 0x05
  *		Satellite Ground Station adress will be 0x2023  and channel is 0x10
  *
  *
  *		##Clean Code explanation
  *		"The __CLOSED expression can be written at the end of the header guard.
  *		 If you remove this part, the related code will work and complied
  *
  *		 "The __notSuccessful , related part of the code didn't work or test
  *
  *		 Somewhere of the code has some variable that has '2' at end of the variable
  *		 as like "PAY_Pressure2", "PAY_VertHeight2".
  *		 that means these variable will be filled by Satellite's Carrier Unit
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "SubSys_Sensor_TPGVH_Driver.h"
#include "SubSys_Sensor_Battery_Driver.h"
#include "SubSys_AlertControl_Driver.h"
#include "SubSys_SDcard_Driver.h"
#include "SubSys_Sensor_GPS_Driver.h"
#include "SubSys_WirelessCommunication_Setting_Driver.h"
#include "SubSys_WirelessCommunication_Telemetry_Driver.h"
#include "SubSys_ARAS.h"
#include "SubSys_Sensor_RTC_Driver.h"

#include "SubSys_Actuator_Servo_Driver.h"
#include "SubSys_ColorFilterControl_Driver.h"
#include "SubSys_SeparationControl_Driver.h"

#include "SubSys_Sensor_IMU_APP_Driver.h"
#include "SubSys_Sensor_IMU_STM32_Driver.h"

#include "SubSys_Payload_FlightStatus.h"
#include "SubSys_Payload_PeriodicReattempt.h"

#include <stdlib.h>
#include <stdint.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/*
	 ===============================================================================
	                      ##### DEFINITIONS #####
	 ===============================================================================
*/

	/**!
	 * Pressure Temperature Sensor (MS5611) Definitions
	 */
	#define MS5611_I2C_ADDRESS_H 0xEE		/*! CSB pin is HIGH */
	#define MS5611_I2C_ADDRESS_L 0xEC 		/*! CSB pin is LOW  */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;
I2C_HandleTypeDef hi2c3;

SD_HandleTypeDef hsd;
DMA_HandleTypeDef hdma_sdio_rx;
DMA_HandleTypeDef hdma_sdio_tx;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
/*
	 ===============================================================================
	                      ##### MULTIPLE VARIABLE #####
	 ===============================================================================
*/

	/**!
	 * SubSys_Sensor_TPGVH_Driver variables
	 */
	MS5611_HandleTypeDef MS5611;	/*! MS5611 object					*/
	float  MS5611_Press=0.0;		/*! Pressure data variable 			*/
	float  MS5611_Temp=0.0;			/*! Temperature data variable 		*/
	float  MS5611_Altitude;			/*! Vertical Altitude data variable */
	float  MS5611_VertSpeed;		/*! Vertical Speed data variable    */
	float  MS5611_VertAcc;			/*! Vertical Acceleration variable  */
	float  MS5611_gForce;			/*! Vertical g force data variable  */
	float  SatCar_Mass;				/*! Total mass of Satellites Carrier module */

	/**!
	 * SubSys_Sensor_Battery_Driver variables
	 */
	uint8_t NumSerialBat=0;			/*! Number of serial connection battery */
	float BatteryVoltage=0.0;		/*! Voltage of the Satellite's battery  */

	/**!
	 * SubSys_Sensor_GPS_Driver variables
	 */
	float GPS_Altitude;				/*! Vertical distance info of satellite beetween */
	float GPS_Longitude;			/*! Location info of satellite on the earth 	 */
	float GPS_Latitude;				/*! Location info of satellite on the earth 	 */


	/**! We create 2 object for subsystem wireless communication,
	 * 	one of them is about configuration setting of wirelesscom device
	 * 	other one is about application object
	 */
	SubSys_WirelesscomConfig_HandleTypeDef dev_WirelessComConfig;
	SubSys_WirelessCom_APP_HandleTypeDef dev_WirelessComApp;

	/**!
	 * SubSys_Sensor_IMU_APP_Driver variables
	 */
	bno055_vector_t vectorIMU;
	float euler_roll;
	float euler_pitch;
	float euler_yaw;

	/**!
	 * SubSys_Sensor_RTC_Driver variables
	 */
	uint8_t date ;
	uint8_t month;
	uint16_t year ;
	uint8_t hour ;
	uint8_t minute;
	uint8_t second ;

	/**
	 * USB-TTL variables, collected datas are sent to Station PC
	 * (The code block has been commented out within the while loop)
	 */
	uint8_t TelemetryData[100] = { 0 };
	uint8_t WrittenBytes;

	/**!
	 * We will create two objects: one for the separation system and one for the color filtering system.
	 */
	Actuator_Servo_HandleTypeDef dev_Servo_Separation;
	Actuator_Servo_HandleTypeDef dev_Servo_ColorFilter;

/*
	 ===============================================================================
						  ##### SINGLE VARIABLE #####
	 ===============================================================================
*/

	/**
	 * 0: Ready for Launch (Before Rocket Ignition)
	 * 1: Ascent
	 * 2: Model Satellite Descent
	 * 3: Separation
	 * 4: Payload Descent
	 * 5: Recovery (Payload Ground Contact)
	 */
	uint8_t SatelliteStatus = ReadyForLaunch;	/*! First step at the beginnig */

	/**
	 * Releated byte will be ==> xxxx xxxx
	 * index:
	 * 	 	 0-> Satt. velocity is not range of 12-14 m/s.
	 * 		 1-> Payload velocity is not range of 6-8 m/s.
	 * 		 2-> Unable to get carrier pressure
	 * 		 3-> Unable to get payload location
	 * 		 4-> Autonomous separation did not occur
	 */
	uint8_t SatelliteErrorCode;

	uint8_t AutonomoSeparationStatus = Permission_NOT;	 /*! if the value is 0 that means there is no separation permission else permission is OK*/

	uint32_t SystemTick;   				 /*! All system units will be work together at 1Hz*/
	uint32_t NumberOfTelePacket = 0;		 			 /*! The value is incremented by +1 at the end of each satellite operation period */

	const uint32_t Race_TeamNo = 270061; 				 /*! It is a fixed number provided by the competition organization.*/

	float PAY2CAR_DiffHeight = 0;						 /*! Vertical Height value from Satellite payload to carrier unit*/
	float CarrierPressure;								 /*! Carrier Unit's pressure data*/
	float CarrierVertHeight;							 /*! Carrier Unit's vertical height data*/
	float GroundStation_IOTTemparature;					 /*! IOT mission , gets datas from Ground Station PC */

	char command_RHRH[4];								 /*! Rakam Harf Rakam Harf receiving datas from ground station pc */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM3_Init(void);
static void MX_SDIO_SD_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_I2C2_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_I2C3_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_ADC1_Init();
  MX_TIM3_Init();
  MX_SDIO_SD_Init();
  MX_FATFS_Init();
  MX_USART1_UART_Init();
  MX_I2C2_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_I2C3_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  /******>>> SENSOR BATTERY INIT BEGIN >>>******/
  	#ifdef SAT_PAYLOAD_SUBSYS_DRIVERS_SENSOR_BATTERY_H__CLOSED
  	NumSerialBat = 2;	/*! Number of serial connection battery */
  	MeasBattery_Init(NumSerialBat);
  	#endif
  /******<<< SENSOR BATTERY INIT END <<<******/


  /******>>> SENSOR TPGVH INITIALIZATION BEGIN >>>******/
  	#ifdef SAT_PAYLOAD_SUBSYS_DRIVERS_SENSOR_TPGVH_H__CLOSED
  	MS5611.I2C_ADDRESS = MS5611_I2C_ADDRESS_H;
  	MS5611.i2c = &hi2c1;
  	MS5611.Ref_Alt_Sel = 'm';
  	MS5611_Init(&MS5611);
  	#endif
  /******<<< SENSOR TPGVH INITIALIZATION END <<<******/


  /******>>> ALERT CONTROL INITIALIZATION BEGIN >>>******/
	#ifdef SAT_PAYLOAD_SUBSYS_DRIVER_ALERTCONTROL_H__CLOSED
	/**!
	 * @Attention!
	 *
	 * @Calculation ==> ![Duty cycle must be %50 according to  the datasheet]
	 * 					 We sets ARR value as 100 at the beginnig
	 * 					 Frequency  = (APB Tim Clock) / ((PSC)*(ARR))
	 *					 Duty Cycle = ((CCRx/ARR)*100)
	 *
	 * APB1 Timer Clock = 50MHz
	 * PSC = 183-1
	 * ARR = 100-1
	 * PSC and ARR's parameters for 2.73KHz passive buzzer module
	 */
	PassiveBuzz_Init(&htim3, TIM_CHANNEL_1);
	#endif
  /******<<< ALERT CONTROL INITIALIZATION END <<<******/


  /******>>> SD CARD INITIALIZATION BEGIN >>>******/
	#ifdef SAT_PAYLOAD_SUBSYS_DRIVERS_SDCARD_H__CLOSED
	/*! We create a buffer that contains the satellite's variables, and we fill it with variables from SD_Data objects */
	extern char SdDatasBuf[LineSize];

	/*!(@warning)	Don't write "E:" , "e:",  "e\" */
	SD_Mount("E/", 0);

	/*
	 * @instructions :
	 * 				First of all, We create a directiory as named "SAT_CAR",
	 * 				Secondly give a name for system txt document as named "STM32.TXT"
	 * 				Finally put our variable buffer
	 *
	 * @Attention!   : If you use lowercase letters, this function will reverse the name to uppercase letters as given below
	 * 					(e.g)CAR_Raw ==> CAR_RAW
	 */
	SD_Create_Dir_File("SAT_CAR", "SAT_CAR/STM32.TXT", SdDatasBuf);
	#endif
  /******<<< SD CARD INITIALIZATION END <<<******/


  /******>>> SENSOR GPS INITIALIZATION BEGIN >>>******/
	#ifdef SAT_PAYLOAD_SUBSYS_DRIVERS_SENSOR_GPS_H__CLOSED
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_SET);
	GPS_Init();
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET);
	#endif
  /******<<< SENSOR GPS INITIALIZATION END <<<******/


  /******>>> WIRELESS COMMUNICATION SETTING & TELEMETRY INITIALIZATION BEGIN >>>******/
	/**
	 * @brief : First of all, we upload initial settings into the wireless communication device.
	 * 		  After that, we determine Target Address high and low byte and Target Channel.
	 * 		  Some LoRa module has two pin as named M0 and M1. These provides selecting working mode
	 * 		  For E220400T30 are M0 and M1 pins.
	 * @note  : If you use dma for receiving and transmiting, fill it parameters that
	 *		  come after channel info
	 */
	 #ifdef SAT_PAYLOAD_SUBSYS_DRIVERS_WIRELESSCOMMUNICATION_SETTING_H__CLOSED
	 dev_WirelessComConfig.interface.huart = &huart2;
	 dev_WirelessComConfig.interface.GPIOx = GPIOB;
	 dev_WirelessComConfig.LORA_PIN_M0= GPIO_PIN_14;
	 dev_WirelessComConfig.LORA_PIN_M1= GPIO_PIN_13;
	 dev_WirelessComConfig.Mode_SW = DeepSleep; 		/*! Module goes to sleep, that provides you to configure settings */

	 SubSys_WirelessCom_Config_Init(&dev_WirelessComConfig);
	 #endif

	 #ifdef SAT_PAYLOAD_SUBSYS_DRIVERS_WIRELESSCOMMUNICATION_TELEMETRY_H
	 /*! Will be filled for your dev that use now*/
	 dev_WirelessComApp.huartX = &huart2;
	 dev_WirelessComConfig.Mode_SW = NormalMode; 		/*! UART and wireless channel are open, transparent transmission is on*/
	 SubSys_WirelessCom_Config_WORK_MODE(&dev_WirelessComConfig);

	 /*! Will be filled for the Ground Station(Target) Device */
	 dev_WirelessComApp.Target_ADDH = 0x20;
	 dev_WirelessComApp.Target_ADDL = 0x23;
	 dev_WirelessComApp.Target_Ch   = 0x10;

	  /*! Interrupt is active for receiving wireless data */
	  HAL_UART_Receive_IT(dev_WirelessComApp.huartX, dev_WirelessComApp.Buffer.Rx, sizeof(dev_WirelessComApp.Buffer.Rx));

	 #endif
  /******<<< WIRELESS COMMUNICATION SETTING & TELEMETRY INITIALIZATION END <<<******/


  /******>>> SENSOR IMU  INITIALIZATION BEGIN >>>******/
	#ifdef SAT_PAYLOAD_SUBSYS_DRIVERS_SENSOR_IMU_APP_H__notSuccesful

	 //*! BNO055 OR MPU9250 IMU SENSOR WİLL BE USED*//
	bno055_assignI2C(&hi2c2);
	bno055_setup();
	bno055_setOperationModeNDOF();
	#endif
  /******<<< SENSOR IMU INITIALIZATION END <<<******/


	/******>>> SERVO SYSTEM INITIALIZATION BEGIN >>>******/
#ifdef SAT_PAYLOAD_SUBSYS_DRIVERS_ACTUATOR_SERVO_H__CLOSED

	/*! Separation system Servo control parameters*/
	dev_Servo_Separation.htim_X 		= &htim1;
	dev_Servo_Separation.tim_channel_in = TIM_CHANNEL_2;
	SubSys_Actuator_Servo_Init(&dev_Servo_Separation);

	#ifdef SAT_PAYLOAD_SUBSYS_DRIVERS_ACTUATOR_SERVO_SEPARATION_TEST_STATUS__CLOSED
		SubSys_Actuator_Servo_MoveTo(&dev_Servo_Separation, 0);
		SubSys_Actuator_Servo_MoveTo(&dev_Servo_Separation, 45);
		SubSys_Actuator_Servo_MoveTo(&dev_Servo_Separation, 90);
		SubSys_Actuator_Servo_MoveTo(&dev_Servo_Separation, 135);
		SubSys_Actuator_Servo_MoveTo(&dev_Servo_Separation, 180);
	#endif

	/*! Color Filtering system Servo control parameters*/
	/*! Succes */
	dev_Servo_ColorFilter.htim_X = &htim2;
	dev_Servo_ColorFilter.tim_channel_in = TIM_CHANNEL_1;
	SubSys_Actuator_Servo_Init(&dev_Servo_ColorFilter);

	#ifdef SAT_PAYLOAD_SUBSYS_DRIVERS_ACTUATOR_SERVO_COLORFILTER_TEST_STATUS__CLOSED
		SubSys_Actuator_Servo_MoveTo(&dev_Servo_ColorFilter, 0);
		SubSys_Actuator_Servo_MoveTo(&dev_Servo_ColorFilter, 45);
		SubSys_Actuator_Servo_MoveTo(&dev_Servo_ColorFilter, 90);
		SubSys_Actuator_Servo_MoveTo(&dev_Servo_ColorFilter, 135);
		SubSys_Actuator_Servo_MoveTo(&dev_Servo_ColorFilter, 180);
	#endif

#endif
	/******<<< SERVO SYSTEM INITIALIZATION END <<<******/


	/******>>> RTC SYSTEM INITIALIZATION BEGIN >>>******/
	#ifdef SAT_PAYLOAD_SUBSYS_DRIVERS_SENSOR_RTC_H__CLOSED

		/*! Config i2c terminal and start the RTC */
		DS1307_Init(&hi2c3);

		/*! The code block required to set the date and time.*/
		#ifdef SAT_PAYLOAD_RTC_CONFIG_PERMISSION_HEADERGUARD__CLOSED
		DS1307_SetTimeZone(+3, 00);
		DS1307_SetDate(5);
		DS1307_SetMonth(7);
		DS1307_SetYear(2024);
		DS1307_SetHour(21);
		DS1307_SetMinute(56);
		DS1307_SetSecond(48);
		#endif

	#endif
	/******<<< RTC SYSTEML INITIALIZATION END <<<******/


	/******>>> SEPARATION CONTROL INITIALIZATION BEGIN >>>******/
	#ifdef SAT_PAYLOAD_SUBSYS_DRIVERS_SEPARATION_CONTROL_H__CLOSED
		HAL_Delay(5000);
		SubSys_SeparationMechanism_Lock_PayloadToCarrier();
	#endif
	/******<<< SEPARATION CONTROL INITIALIZATION END <<<******/


	/******>>> COLOR FILTER CONTROL INITIALIZATION BEGIN >>>******/
	#ifdef SAT_PAYLOAD_SUBSYS_DRIVERS_COLORFILTER_CONTROL_H__CLOSED
		/*! Will be added init code block in here */
		SubSys_ColorFilterMechanism_TurnTo(Filter_None);

		#ifdef SAT_PAYLOAD_SUBSYS_DRIVERS_COLORFILTER_CONTROL_TEST_STATUS__CLOSED
		SubSys_ColorFilterMechanism_TurnTo(Filter_None);
		SubSys_ColorFilterMechanism_TurnTo(filter_Red);
		SubSys_ColorFilterMechanism_TurnTo(filter_Green);
		SubSys_ColorFilterMechanism_TurnTo(filter_Blue);
		#endif

	#endif
	/******<<< COLOR FILTER CONTROL INITIALIZATION END <<<******/


	/******>>> CAMERA ACTIVE INITIALIZATION BEGIN >>>******/
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
	/******<<< CAMERA ACTIVE INITIALIZATION END <<<******/



  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	 SubSys_SatelliteMission_Continue();

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 72;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 3;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief I2C3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C3_Init(void)
{

  /* USER CODE BEGIN I2C3_Init 0 */

  /* USER CODE END I2C3_Init 0 */

  /* USER CODE BEGIN I2C3_Init 1 */

  /* USER CODE END I2C3_Init 1 */
  hi2c3.Instance = I2C3;
  hi2c3.Init.ClockSpeed = 100000;
  hi2c3.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C3_Init 2 */

  /* USER CODE END I2C3_Init 2 */

}

/**
  * @brief SDIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_SDIO_SD_Init(void)
{

  /* USER CODE BEGIN SDIO_Init 0 */

  /* USER CODE END SDIO_Init 0 */

  /* USER CODE BEGIN SDIO_Init 1 */

  /* USER CODE END SDIO_Init 1 */
  hsd.Instance = SDIO;
  hsd.Init.ClockEdge = SDIO_CLOCK_EDGE_RISING;
  hsd.Init.ClockBypass = SDIO_CLOCK_BYPASS_DISABLE;
  hsd.Init.ClockPowerSave = SDIO_CLOCK_POWER_SAVE_DISABLE;
  hsd.Init.BusWide = SDIO_BUS_WIDE_4B;
  hsd.Init.HardwareFlowControl = SDIO_HARDWARE_FLOW_CONTROL_DISABLE;
  hsd.Init.ClockDiv = 0;
  /* USER CODE BEGIN SDIO_Init 2 */

  /* USER CODE END SDIO_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 1440-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 1000-1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 1440-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1000-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 263-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 100-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream3_IRQn);
  /* DMA2_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream6_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_4|GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC0 PC1 PC4 PC5 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PC2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PB13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
