################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/.claude/worktrees/optimistic-driscoll/Legacy\ Code/esp-testing/uart_select/main/uart_select_example_main.c 

OBJS += \
./Core/Src/.claude/worktrees/optimistic-driscoll/Legacy\ Code/esp-testing/uart_select/main/uart_select_example_main.o 

C_DEPS += \
./Core/Src/.claude/worktrees/optimistic-driscoll/Legacy\ Code/esp-testing/uart_select/main/uart_select_example_main.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/.claude/worktrees/optimistic-driscoll/Legacy\ Code/esp-testing/uart_select/main/uart_select_example_main.o: ../Core/Src/.claude/worktrees/optimistic-driscoll/Legacy\ Code/esp-testing/uart_select/main/uart_select_example_main.c Core/Src/.claude/worktrees/optimistic-driscoll/Legacy\ Code/esp-testing/uart_select/main/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0plus -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32C051xx -c -I../Core/Inc -I../Drivers/STM32C0xx_HAL_Driver/Inc -I../Drivers/STM32C0xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32C0xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"Core/Src/.claude/worktrees/optimistic-driscoll/Legacy Code/esp-testing/uart_select/main/uart_select_example_main.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Core-2f-Src-2f--2e-claude-2f-worktrees-2f-optimistic-2d-driscoll-2f-Legacy-20-Code-2f-esp-2d-testing-2f-uart_select-2f-main

clean-Core-2f-Src-2f--2e-claude-2f-worktrees-2f-optimistic-2d-driscoll-2f-Legacy-20-Code-2f-esp-2d-testing-2f-uart_select-2f-main:
	-$(RM) ./Core/Src/.claude/worktrees/optimistic-driscoll/Legacy\ Code/esp-testing/uart_select/main/uart_select_example_main.cyclo ./Core/Src/.claude/worktrees/optimistic-driscoll/Legacy\ Code/esp-testing/uart_select/main/uart_select_example_main.d ./Core/Src/.claude/worktrees/optimistic-driscoll/Legacy\ Code/esp-testing/uart_select/main/uart_select_example_main.o ./Core/Src/.claude/worktrees/optimistic-driscoll/Legacy\ Code/esp-testing/uart_select/main/uart_select_example_main.su

.PHONY: clean-Core-2f-Src-2f--2e-claude-2f-worktrees-2f-optimistic-2d-driscoll-2f-Legacy-20-Code-2f-esp-2d-testing-2f-uart_select-2f-main

