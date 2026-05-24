################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/airbrake_control.c \
../Core/Src/airbrake_deploy.c \
../Core/Src/command_sequence.c \
../Core/Src/encoder_app.c \
../Core/Src/encoder_homing.c \
../Core/Src/esc_app.c \
../Core/Src/esc_telem.c \
../Core/Src/fatfs.c \
../Core/Src/flight_estimator.c \
../Core/Src/flight_trigger.c \
../Core/Src/gpio.c \
../Core/Src/icache.c \
../Core/Src/main.c \
../Core/Src/rtc.c \
../Core/Src/sd_logger.c \
../Core/Src/sdmmc.c \
../Core/Src/spi.c \
../Core/Src/stm32h5xx_hal_msp.c \
../Core/Src/stm32h5xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32h5xx.c \
../Core/Src/tim.c \
../Core/Src/usart.c 

OBJS += \
./Core/Src/airbrake_control.o \
./Core/Src/airbrake_deploy.o \
./Core/Src/command_sequence.o \
./Core/Src/encoder_app.o \
./Core/Src/encoder_homing.o \
./Core/Src/esc_app.o \
./Core/Src/esc_telem.o \
./Core/Src/fatfs.o \
./Core/Src/flight_estimator.o \
./Core/Src/flight_trigger.o \
./Core/Src/gpio.o \
./Core/Src/icache.o \
./Core/Src/main.o \
./Core/Src/rtc.o \
./Core/Src/sd_logger.o \
./Core/Src/sdmmc.o \
./Core/Src/spi.o \
./Core/Src/stm32h5xx_hal_msp.o \
./Core/Src/stm32h5xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32h5xx.o \
./Core/Src/tim.o \
./Core/Src/usart.o 

C_DEPS += \
./Core/Src/airbrake_control.d \
./Core/Src/airbrake_deploy.d \
./Core/Src/command_sequence.d \
./Core/Src/encoder_app.d \
./Core/Src/encoder_homing.d \
./Core/Src/esc_app.d \
./Core/Src/esc_telem.d \
./Core/Src/fatfs.d \
./Core/Src/flight_estimator.d \
./Core/Src/flight_trigger.d \
./Core/Src/gpio.d \
./Core/Src/icache.d \
./Core/Src/main.d \
./Core/Src/rtc.d \
./Core/Src/sd_logger.d \
./Core/Src/sdmmc.d \
./Core/Src/spi.d \
./Core/Src/stm32h5xx_hal_msp.d \
./Core/Src/stm32h5xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32h5xx.d \
./Core/Src/tim.d \
./Core/Src/usart.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su Core/Src/%.cyclo: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m33 -std=gnu11 -DUSE_HAL_DRIVER -DSTM32H562xx -DFX_INCLUDE_USER_DEFINE_FILE -c -I../Core/Inc -I../Drivers/STM32H5xx_HAL_Driver/Inc -I../Drivers/STM32H5xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H5xx/Include -I../Drivers/CMSIS/Include -I../FileX/App -I../FileX/Target -I../Middlewares/ST/filex/common/inc -I../Middlewares/ST/filex/ports/generic/inc -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/airbrake_control.cyclo ./Core/Src/airbrake_control.d ./Core/Src/airbrake_control.o ./Core/Src/airbrake_control.su ./Core/Src/airbrake_deploy.cyclo ./Core/Src/airbrake_deploy.d ./Core/Src/airbrake_deploy.o ./Core/Src/airbrake_deploy.su ./Core/Src/command_sequence.cyclo ./Core/Src/command_sequence.d ./Core/Src/command_sequence.o ./Core/Src/command_sequence.su ./Core/Src/encoder_app.cyclo ./Core/Src/encoder_app.d ./Core/Src/encoder_app.o ./Core/Src/encoder_app.su ./Core/Src/encoder_homing.cyclo ./Core/Src/encoder_homing.d ./Core/Src/encoder_homing.o ./Core/Src/encoder_homing.su ./Core/Src/esc_app.cyclo ./Core/Src/esc_app.d ./Core/Src/esc_app.o ./Core/Src/esc_app.su ./Core/Src/esc_telem.cyclo ./Core/Src/esc_telem.d ./Core/Src/esc_telem.o ./Core/Src/esc_telem.su ./Core/Src/fatfs.cyclo ./Core/Src/fatfs.d ./Core/Src/fatfs.o ./Core/Src/fatfs.su ./Core/Src/flight_estimator.cyclo ./Core/Src/flight_estimator.d ./Core/Src/flight_estimator.o ./Core/Src/flight_estimator.su ./Core/Src/flight_trigger.cyclo ./Core/Src/flight_trigger.d ./Core/Src/flight_trigger.o ./Core/Src/flight_trigger.su ./Core/Src/gpio.cyclo ./Core/Src/gpio.d ./Core/Src/gpio.o ./Core/Src/gpio.su ./Core/Src/icache.cyclo ./Core/Src/icache.d ./Core/Src/icache.o ./Core/Src/icache.su ./Core/Src/main.cyclo ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/rtc.cyclo ./Core/Src/rtc.d ./Core/Src/rtc.o ./Core/Src/rtc.su ./Core/Src/sd_logger.cyclo ./Core/Src/sd_logger.d ./Core/Src/sd_logger.o ./Core/Src/sd_logger.su ./Core/Src/sdmmc.cyclo ./Core/Src/sdmmc.d ./Core/Src/sdmmc.o ./Core/Src/sdmmc.su ./Core/Src/spi.cyclo ./Core/Src/spi.d ./Core/Src/spi.o ./Core/Src/spi.su ./Core/Src/stm32h5xx_hal_msp.cyclo ./Core/Src/stm32h5xx_hal_msp.d ./Core/Src/stm32h5xx_hal_msp.o ./Core/Src/stm32h5xx_hal_msp.su ./Core/Src/stm32h5xx_it.cyclo ./Core/Src/stm32h5xx_it.d ./Core/Src/stm32h5xx_it.o ./Core/Src/stm32h5xx_it.su ./Core/Src/syscalls.cyclo ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.cyclo ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32h5xx.cyclo ./Core/Src/system_stm32h5xx.d ./Core/Src/system_stm32h5xx.o ./Core/Src/system_stm32h5xx.su ./Core/Src/tim.cyclo ./Core/Src/tim.d ./Core/Src/tim.o ./Core/Src/tim.su ./Core/Src/usart.cyclo ./Core/Src/usart.d ./Core/Src/usart.o ./Core/Src/usart.su

.PHONY: clean-Core-2f-Src

