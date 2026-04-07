################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/.claude/worktrees/optimistic-driscoll/Embedded\ Programming/h7/Core/Src/main.c \
../Core/Src/.claude/worktrees/optimistic-driscoll/Embedded\ Programming/h7/Core/Src/runcam_device.c \
../Core/Src/.claude/worktrees/optimistic-driscoll/Embedded\ Programming/h7/Core/Src/stm32h7xx_hal_msp.c \
../Core/Src/.claude/worktrees/optimistic-driscoll/Embedded\ Programming/h7/Core/Src/stm32h7xx_it.c \
../Core/Src/.claude/worktrees/optimistic-driscoll/Embedded\ Programming/h7/Core/Src/syscalls.c \
../Core/Src/.claude/worktrees/optimistic-driscoll/Embedded\ Programming/h7/Core/Src/sysmem.c \
../Core/Src/.claude/worktrees/optimistic-driscoll/Embedded\ Programming/h7/Core/Src/system_stm32h7xx.c 

OBJS += \
./Core/Src/.claude/worktrees/optimistic-driscoll/Embedded\ Programming/h7/Core/Src/main.o \
./Core/Src/.claude/worktrees/optimistic-driscoll/Embedded\ Programming/h7/Core/Src/runcam_device.o \
./Core/Src/.claude/worktrees/optimistic-driscoll/Embedded\ Programming/h7/Core/Src/stm32h7xx_hal_msp.o \
./Core/Src/.claude/worktrees/optimistic-driscoll/Embedded\ Programming/h7/Core/Src/stm32h7xx_it.o \
./Core/Src/.claude/worktrees/optimistic-driscoll/Embedded\ Programming/h7/Core/Src/syscalls.o \
./Core/Src/.claude/worktrees/optimistic-driscoll/Embedded\ Programming/h7/Core/Src/sysmem.o \
./Core/Src/.claude/worktrees/optimistic-driscoll/Embedded\ Programming/h7/Core/Src/system_stm32h7xx.o 

C_DEPS += \
./Core/Src/.claude/worktrees/optimistic-driscoll/Embedded\ Programming/h7/Core/Src/main.d \
./Core/Src/.claude/worktrees/optimistic-driscoll/Embedded\ Programming/h7/Core/Src/runcam_device.d \
./Core/Src/.claude/worktrees/optimistic-driscoll/Embedded\ Programming/h7/Core/Src/stm32h7xx_hal_msp.d \
./Core/Src/.claude/worktrees/optimistic-driscoll/Embedded\ Programming/h7/Core/Src/stm32h7xx_it.d \
./Core/Src/.claude/worktrees/optimistic-driscoll/Embedded\ Programming/h7/Core/Src/syscalls.d \
./Core/Src/.claude/worktrees/optimistic-driscoll/Embedded\ Programming/h7/Core/Src/sysmem.d \
./Core/Src/.claude/worktrees/optimistic-driscoll/Embedded\ Programming/h7/Core/Src/system_stm32h7xx.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/.claude/worktrees/optimistic-driscoll/Embedded\ Programming/h7/Core/Src/main.o: ../Core/Src/.claude/worktrees/optimistic-driscoll/Embedded\ Programming/h7/Core/Src/main.c Core/Src/.claude/worktrees/optimistic-driscoll/Embedded\ Programming/h7/Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0plus -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32C051xx -c -I../Core/Inc -I../Drivers/STM32C0xx_HAL_Driver/Inc -I../Drivers/STM32C0xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32C0xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"Core/Src/.claude/worktrees/optimistic-driscoll/Embedded Programming/h7/Core/Src/main.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Core/Src/.claude/worktrees/optimistic-driscoll/Embedded\ Programming/h7/Core/Src/runcam_device.o: ../Core/Src/.claude/worktrees/optimistic-driscoll/Embedded\ Programming/h7/Core/Src/runcam_device.c Core/Src/.claude/worktrees/optimistic-driscoll/Embedded\ Programming/h7/Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0plus -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32C051xx -c -I../Core/Inc -I../Drivers/STM32C0xx_HAL_Driver/Inc -I../Drivers/STM32C0xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32C0xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"Core/Src/.claude/worktrees/optimistic-driscoll/Embedded Programming/h7/Core/Src/runcam_device.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Core/Src/.claude/worktrees/optimistic-driscoll/Embedded\ Programming/h7/Core/Src/stm32h7xx_hal_msp.o: ../Core/Src/.claude/worktrees/optimistic-driscoll/Embedded\ Programming/h7/Core/Src/stm32h7xx_hal_msp.c Core/Src/.claude/worktrees/optimistic-driscoll/Embedded\ Programming/h7/Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0plus -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32C051xx -c -I../Core/Inc -I../Drivers/STM32C0xx_HAL_Driver/Inc -I../Drivers/STM32C0xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32C0xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"Core/Src/.claude/worktrees/optimistic-driscoll/Embedded Programming/h7/Core/Src/stm32h7xx_hal_msp.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Core/Src/.claude/worktrees/optimistic-driscoll/Embedded\ Programming/h7/Core/Src/stm32h7xx_it.o: ../Core/Src/.claude/worktrees/optimistic-driscoll/Embedded\ Programming/h7/Core/Src/stm32h7xx_it.c Core/Src/.claude/worktrees/optimistic-driscoll/Embedded\ Programming/h7/Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0plus -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32C051xx -c -I../Core/Inc -I../Drivers/STM32C0xx_HAL_Driver/Inc -I../Drivers/STM32C0xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32C0xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"Core/Src/.claude/worktrees/optimistic-driscoll/Embedded Programming/h7/Core/Src/stm32h7xx_it.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Core/Src/.claude/worktrees/optimistic-driscoll/Embedded\ Programming/h7/Core/Src/syscalls.o: ../Core/Src/.claude/worktrees/optimistic-driscoll/Embedded\ Programming/h7/Core/Src/syscalls.c Core/Src/.claude/worktrees/optimistic-driscoll/Embedded\ Programming/h7/Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0plus -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32C051xx -c -I../Core/Inc -I../Drivers/STM32C0xx_HAL_Driver/Inc -I../Drivers/STM32C0xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32C0xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"Core/Src/.claude/worktrees/optimistic-driscoll/Embedded Programming/h7/Core/Src/syscalls.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Core/Src/.claude/worktrees/optimistic-driscoll/Embedded\ Programming/h7/Core/Src/sysmem.o: ../Core/Src/.claude/worktrees/optimistic-driscoll/Embedded\ Programming/h7/Core/Src/sysmem.c Core/Src/.claude/worktrees/optimistic-driscoll/Embedded\ Programming/h7/Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0plus -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32C051xx -c -I../Core/Inc -I../Drivers/STM32C0xx_HAL_Driver/Inc -I../Drivers/STM32C0xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32C0xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"Core/Src/.claude/worktrees/optimistic-driscoll/Embedded Programming/h7/Core/Src/sysmem.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Core/Src/.claude/worktrees/optimistic-driscoll/Embedded\ Programming/h7/Core/Src/system_stm32h7xx.o: ../Core/Src/.claude/worktrees/optimistic-driscoll/Embedded\ Programming/h7/Core/Src/system_stm32h7xx.c Core/Src/.claude/worktrees/optimistic-driscoll/Embedded\ Programming/h7/Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0plus -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32C051xx -c -I../Core/Inc -I../Drivers/STM32C0xx_HAL_Driver/Inc -I../Drivers/STM32C0xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32C0xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"Core/Src/.claude/worktrees/optimistic-driscoll/Embedded Programming/h7/Core/Src/system_stm32h7xx.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Core-2f-Src-2f--2e-claude-2f-worktrees-2f-optimistic-2d-driscoll-2f-Embedded-20-Programming-2f-h7-2f-Core-2f-Src

clean-Core-2f-Src-2f--2e-claude-2f-worktrees-2f-optimistic-2d-driscoll-2f-Embedded-20-Programming-2f-h7-2f-Core-2f-Src:
	-$(RM) ./Core/Src/.claude/worktrees/optimistic-driscoll/Embedded\ Programming/h7/Core/Src/main.cyclo ./Core/Src/.claude/worktrees/optimistic-driscoll/Embedded\ Programming/h7/Core/Src/main.d ./Core/Src/.claude/worktrees/optimistic-driscoll/Embedded\ Programming/h7/Core/Src/main.o ./Core/Src/.claude/worktrees/optimistic-driscoll/Embedded\ Programming/h7/Core/Src/main.su ./Core/Src/.claude/worktrees/optimistic-driscoll/Embedded\ Programming/h7/Core/Src/runcam_device.cyclo ./Core/Src/.claude/worktrees/optimistic-driscoll/Embedded\ Programming/h7/Core/Src/runcam_device.d ./Core/Src/.claude/worktrees/optimistic-driscoll/Embedded\ Programming/h7/Core/Src/runcam_device.o ./Core/Src/.claude/worktrees/optimistic-driscoll/Embedded\ Programming/h7/Core/Src/runcam_device.su ./Core/Src/.claude/worktrees/optimistic-driscoll/Embedded\ Programming/h7/Core/Src/stm32h7xx_hal_msp.cyclo ./Core/Src/.claude/worktrees/optimistic-driscoll/Embedded\ Programming/h7/Core/Src/stm32h7xx_hal_msp.d ./Core/Src/.claude/worktrees/optimistic-driscoll/Embedded\ Programming/h7/Core/Src/stm32h7xx_hal_msp.o ./Core/Src/.claude/worktrees/optimistic-driscoll/Embedded\ Programming/h7/Core/Src/stm32h7xx_hal_msp.su ./Core/Src/.claude/worktrees/optimistic-driscoll/Embedded\ Programming/h7/Core/Src/stm32h7xx_it.cyclo ./Core/Src/.claude/worktrees/optimistic-driscoll/Embedded\ Programming/h7/Core/Src/stm32h7xx_it.d ./Core/Src/.claude/worktrees/optimistic-driscoll/Embedded\ Programming/h7/Core/Src/stm32h7xx_it.o ./Core/Src/.claude/worktrees/optimistic-driscoll/Embedded\ Programming/h7/Core/Src/stm32h7xx_it.su ./Core/Src/.claude/worktrees/optimistic-driscoll/Embedded\ Programming/h7/Core/Src/syscalls.cyclo ./Core/Src/.claude/worktrees/optimistic-driscoll/Embedded\ Programming/h7/Core/Src/syscalls.d ./Core/Src/.claude/worktrees/optimistic-driscoll/Embedded\ Programming/h7/Core/Src/syscalls.o ./Core/Src/.claude/worktrees/optimistic-driscoll/Embedded\ Programming/h7/Core/Src/syscalls.su ./Core/Src/.claude/worktrees/optimistic-driscoll/Embedded\ Programming/h7/Core/Src/sysmem.cyclo ./Core/Src/.claude/worktrees/optimistic-driscoll/Embedded\ Programming/h7/Core/Src/sysmem.d ./Core/Src/.claude/worktrees/optimistic-driscoll/Embedded\ Programming/h7/Core/Src/sysmem.o ./Core/Src/.claude/worktrees/optimistic-driscoll/Embedded\ Programming/h7/Core/Src/sysmem.su ./Core/Src/.claude/worktrees/optimistic-driscoll/Embedded\ Programming/h7/Core/Src/system_stm32h7xx.cyclo ./Core/Src/.claude/worktrees/optimistic-driscoll/Embedded\ Programming/h7/Core/Src/system_stm32h7xx.d ./Core/Src/.claude/worktrees/optimistic-driscoll/Embedded\ Programming/h7/Core/Src/system_stm32h7xx.o ./Core/Src/.claude/worktrees/optimistic-driscoll/Embedded\ Programming/h7/Core/Src/system_stm32h7xx.su

.PHONY: clean-Core-2f-Src-2f--2e-claude-2f-worktrees-2f-optimistic-2d-driscoll-2f-Embedded-20-Programming-2f-h7-2f-Core-2f-Src

