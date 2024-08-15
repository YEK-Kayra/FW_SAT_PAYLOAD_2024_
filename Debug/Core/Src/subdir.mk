################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/SubSys_ARAS.c \
../Core/Src/SubSys_Actuator_Servo_Driver.c \
../Core/Src/SubSys_AlertControl_Driver.c \
../Core/Src/SubSys_ColorFilterControl_Driver.c \
../Core/Src/SubSys_Payload_FlightStatus.c \
../Core/Src/SubSys_Payload_PeriodicReattempt.c \
../Core/Src/SubSys_SDcard_Driver.c \
../Core/Src/SubSys_Sensor_Battery_Driver.c \
../Core/Src/SubSys_Sensor_GPS_Driver.c \
../Core/Src/SubSys_Sensor_IMU_APP_Driver.c \
../Core/Src/SubSys_Sensor_RTC_Driver.c \
../Core/Src/SubSys_Sensor_TPGVH_Driver.c \
../Core/Src/SubSys_SeparationControl_Driver.c \
../Core/Src/SubSys_WirelessCommunication_Setting_Driver.c \
../Core/Src/SubSys_WirelessCommunication_Telemetry_Driver.c \
../Core/Src/main.c \
../Core/Src/stm32f4xx_hal_msp.c \
../Core/Src/stm32f4xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32f4xx.c 

OBJS += \
./Core/Src/SubSys_ARAS.o \
./Core/Src/SubSys_Actuator_Servo_Driver.o \
./Core/Src/SubSys_AlertControl_Driver.o \
./Core/Src/SubSys_ColorFilterControl_Driver.o \
./Core/Src/SubSys_Payload_FlightStatus.o \
./Core/Src/SubSys_Payload_PeriodicReattempt.o \
./Core/Src/SubSys_SDcard_Driver.o \
./Core/Src/SubSys_Sensor_Battery_Driver.o \
./Core/Src/SubSys_Sensor_GPS_Driver.o \
./Core/Src/SubSys_Sensor_IMU_APP_Driver.o \
./Core/Src/SubSys_Sensor_RTC_Driver.o \
./Core/Src/SubSys_Sensor_TPGVH_Driver.o \
./Core/Src/SubSys_SeparationControl_Driver.o \
./Core/Src/SubSys_WirelessCommunication_Setting_Driver.o \
./Core/Src/SubSys_WirelessCommunication_Telemetry_Driver.o \
./Core/Src/main.o \
./Core/Src/stm32f4xx_hal_msp.o \
./Core/Src/stm32f4xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32f4xx.o 

C_DEPS += \
./Core/Src/SubSys_ARAS.d \
./Core/Src/SubSys_Actuator_Servo_Driver.d \
./Core/Src/SubSys_AlertControl_Driver.d \
./Core/Src/SubSys_ColorFilterControl_Driver.d \
./Core/Src/SubSys_Payload_FlightStatus.d \
./Core/Src/SubSys_Payload_PeriodicReattempt.d \
./Core/Src/SubSys_SDcard_Driver.d \
./Core/Src/SubSys_Sensor_Battery_Driver.d \
./Core/Src/SubSys_Sensor_GPS_Driver.d \
./Core/Src/SubSys_Sensor_IMU_APP_Driver.d \
./Core/Src/SubSys_Sensor_RTC_Driver.d \
./Core/Src/SubSys_Sensor_TPGVH_Driver.d \
./Core/Src/SubSys_SeparationControl_Driver.d \
./Core/Src/SubSys_WirelessCommunication_Setting_Driver.d \
./Core/Src/SubSys_WirelessCommunication_Telemetry_Driver.d \
./Core/Src/main.d \
./Core/Src/stm32f4xx_hal_msp.d \
./Core/Src/stm32f4xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32f4xx.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su Core/Src/%.cyclo: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F411xE -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../FATFS/Target -I../FATFS/App -I../Middlewares/Third_Party/FatFs/src -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/SubSys_ARAS.cyclo ./Core/Src/SubSys_ARAS.d ./Core/Src/SubSys_ARAS.o ./Core/Src/SubSys_ARAS.su ./Core/Src/SubSys_Actuator_Servo_Driver.cyclo ./Core/Src/SubSys_Actuator_Servo_Driver.d ./Core/Src/SubSys_Actuator_Servo_Driver.o ./Core/Src/SubSys_Actuator_Servo_Driver.su ./Core/Src/SubSys_AlertControl_Driver.cyclo ./Core/Src/SubSys_AlertControl_Driver.d ./Core/Src/SubSys_AlertControl_Driver.o ./Core/Src/SubSys_AlertControl_Driver.su ./Core/Src/SubSys_ColorFilterControl_Driver.cyclo ./Core/Src/SubSys_ColorFilterControl_Driver.d ./Core/Src/SubSys_ColorFilterControl_Driver.o ./Core/Src/SubSys_ColorFilterControl_Driver.su ./Core/Src/SubSys_Payload_FlightStatus.cyclo ./Core/Src/SubSys_Payload_FlightStatus.d ./Core/Src/SubSys_Payload_FlightStatus.o ./Core/Src/SubSys_Payload_FlightStatus.su ./Core/Src/SubSys_Payload_PeriodicReattempt.cyclo ./Core/Src/SubSys_Payload_PeriodicReattempt.d ./Core/Src/SubSys_Payload_PeriodicReattempt.o ./Core/Src/SubSys_Payload_PeriodicReattempt.su ./Core/Src/SubSys_SDcard_Driver.cyclo ./Core/Src/SubSys_SDcard_Driver.d ./Core/Src/SubSys_SDcard_Driver.o ./Core/Src/SubSys_SDcard_Driver.su ./Core/Src/SubSys_Sensor_Battery_Driver.cyclo ./Core/Src/SubSys_Sensor_Battery_Driver.d ./Core/Src/SubSys_Sensor_Battery_Driver.o ./Core/Src/SubSys_Sensor_Battery_Driver.su ./Core/Src/SubSys_Sensor_GPS_Driver.cyclo ./Core/Src/SubSys_Sensor_GPS_Driver.d ./Core/Src/SubSys_Sensor_GPS_Driver.o ./Core/Src/SubSys_Sensor_GPS_Driver.su ./Core/Src/SubSys_Sensor_IMU_APP_Driver.cyclo ./Core/Src/SubSys_Sensor_IMU_APP_Driver.d ./Core/Src/SubSys_Sensor_IMU_APP_Driver.o ./Core/Src/SubSys_Sensor_IMU_APP_Driver.su ./Core/Src/SubSys_Sensor_RTC_Driver.cyclo ./Core/Src/SubSys_Sensor_RTC_Driver.d ./Core/Src/SubSys_Sensor_RTC_Driver.o ./Core/Src/SubSys_Sensor_RTC_Driver.su ./Core/Src/SubSys_Sensor_TPGVH_Driver.cyclo ./Core/Src/SubSys_Sensor_TPGVH_Driver.d ./Core/Src/SubSys_Sensor_TPGVH_Driver.o ./Core/Src/SubSys_Sensor_TPGVH_Driver.su ./Core/Src/SubSys_SeparationControl_Driver.cyclo ./Core/Src/SubSys_SeparationControl_Driver.d ./Core/Src/SubSys_SeparationControl_Driver.o ./Core/Src/SubSys_SeparationControl_Driver.su ./Core/Src/SubSys_WirelessCommunication_Setting_Driver.cyclo ./Core/Src/SubSys_WirelessCommunication_Setting_Driver.d ./Core/Src/SubSys_WirelessCommunication_Setting_Driver.o ./Core/Src/SubSys_WirelessCommunication_Setting_Driver.su ./Core/Src/SubSys_WirelessCommunication_Telemetry_Driver.cyclo ./Core/Src/SubSys_WirelessCommunication_Telemetry_Driver.d ./Core/Src/SubSys_WirelessCommunication_Telemetry_Driver.o ./Core/Src/SubSys_WirelessCommunication_Telemetry_Driver.su ./Core/Src/main.cyclo ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/stm32f4xx_hal_msp.cyclo ./Core/Src/stm32f4xx_hal_msp.d ./Core/Src/stm32f4xx_hal_msp.o ./Core/Src/stm32f4xx_hal_msp.su ./Core/Src/stm32f4xx_it.cyclo ./Core/Src/stm32f4xx_it.d ./Core/Src/stm32f4xx_it.o ./Core/Src/stm32f4xx_it.su ./Core/Src/syscalls.cyclo ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.cyclo ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32f4xx.cyclo ./Core/Src/system_stm32f4xx.d ./Core/Src/system_stm32f4xx.o ./Core/Src/system_stm32f4xx.su

.PHONY: clean-Core-2f-Src

