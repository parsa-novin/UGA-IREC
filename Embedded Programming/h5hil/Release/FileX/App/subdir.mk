################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../FileX/App/app_filex.c 

OBJS += \
./FileX/App/app_filex.o 

C_DEPS += \
./FileX/App/app_filex.d 


# Each subdirectory must supply rules for building sources it contributes
FileX/App/%.o FileX/App/%.su FileX/App/%.cyclo: ../FileX/App/%.c FileX/App/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m33 -std=gnu11 -DUSE_HAL_DRIVER -DSTM32H562xx -DFX_INCLUDE_USER_DEFINE_FILE -DHIL_MODE -c -I../Core/Inc -I"C:/Users/parsa/Desktop/UGASpaceport/Embedded Programming/h5hil/../h5/Drivers/STM32H5xx_HAL_Driver/Inc" -I"C:/Users/parsa/Desktop/UGASpaceport/Embedded Programming/h5hil/../h5/Drivers/STM32H5xx_HAL_Driver/Inc/Legacy" -I"C:/Users/parsa/Desktop/UGASpaceport/Embedded Programming/h5hil/../h5/Drivers/CMSIS/Device/ST/STM32H5xx/Include" -I"C:/Users/parsa/Desktop/UGASpaceport/Embedded Programming/h5hil/../h5/Drivers/CMSIS/Include" -I../FileX/App -I../FileX/Target -I"C:/Users/parsa/Desktop/UGASpaceport/Embedded Programming/h5hil/../h5/Middlewares/ST/filex/common/inc" -I"C:/Users/parsa/Desktop/UGASpaceport/Embedded Programming/h5hil/../h5/Middlewares/ST/filex/ports/generic/inc" -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-FileX-2f-App

clean-FileX-2f-App:
	-$(RM) ./FileX/App/app_filex.cyclo ./FileX/App/app_filex.d ./FileX/App/app_filex.o ./FileX/App/app_filex.su

.PHONY: clean-FileX-2f-App

