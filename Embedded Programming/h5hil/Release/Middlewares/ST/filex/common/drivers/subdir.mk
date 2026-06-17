################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
C:/Users/parsa/Desktop/UGASpaceport/Embedded\ Programming/h5/Middlewares/ST/filex/common/drivers/fx_stm32_sd_driver.c 

OBJS += \
./Middlewares/ST/filex/common/drivers/fx_stm32_sd_driver.o 

C_DEPS += \
./Middlewares/ST/filex/common/drivers/fx_stm32_sd_driver.d 


# Each subdirectory must supply rules for building sources it contributes
Middlewares/ST/filex/common/drivers/fx_stm32_sd_driver.o: C:/Users/parsa/Desktop/UGASpaceport/Embedded\ Programming/h5/Middlewares/ST/filex/common/drivers/fx_stm32_sd_driver.c Middlewares/ST/filex/common/drivers/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m33 -std=gnu11 -DUSE_HAL_DRIVER -DSTM32H562xx -DFX_INCLUDE_USER_DEFINE_FILE -DHIL_MODE -c -I../Core/Inc -I"C:/Users/parsa/Desktop/UGASpaceport/Embedded Programming/h5hil/../h5/Drivers/STM32H5xx_HAL_Driver/Inc" -I"C:/Users/parsa/Desktop/UGASpaceport/Embedded Programming/h5hil/../h5/Drivers/STM32H5xx_HAL_Driver/Inc/Legacy" -I"C:/Users/parsa/Desktop/UGASpaceport/Embedded Programming/h5hil/../h5/Drivers/CMSIS/Device/ST/STM32H5xx/Include" -I"C:/Users/parsa/Desktop/UGASpaceport/Embedded Programming/h5hil/../h5/Drivers/CMSIS/Include" -I../FileX/App -I../FileX/Target -I"C:/Users/parsa/Desktop/UGASpaceport/Embedded Programming/h5hil/../h5/Middlewares/ST/filex/common/inc" -I"C:/Users/parsa/Desktop/UGASpaceport/Embedded Programming/h5hil/../h5/Middlewares/ST/filex/ports/generic/inc" -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Middlewares-2f-ST-2f-filex-2f-common-2f-drivers

clean-Middlewares-2f-ST-2f-filex-2f-common-2f-drivers:
	-$(RM) ./Middlewares/ST/filex/common/drivers/fx_stm32_sd_driver.cyclo ./Middlewares/ST/filex/common/drivers/fx_stm32_sd_driver.d ./Middlewares/ST/filex/common/drivers/fx_stm32_sd_driver.o ./Middlewares/ST/filex/common/drivers/fx_stm32_sd_driver.su

.PHONY: clean-Middlewares-2f-ST-2f-filex-2f-common-2f-drivers

