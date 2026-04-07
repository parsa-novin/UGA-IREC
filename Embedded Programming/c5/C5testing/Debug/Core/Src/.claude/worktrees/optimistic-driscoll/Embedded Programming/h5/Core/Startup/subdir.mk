################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
S_SRCS += \
../Core/Src/.claude/worktrees/optimistic-driscoll/Embedded\ Programming/h5/Core/Startup/startup_stm32h562ritx.s 

OBJS += \
./Core/Src/.claude/worktrees/optimistic-driscoll/Embedded\ Programming/h5/Core/Startup/startup_stm32h562ritx.o 

S_DEPS += \
./Core/Src/.claude/worktrees/optimistic-driscoll/Embedded\ Programming/h5/Core/Startup/startup_stm32h562ritx.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/.claude/worktrees/optimistic-driscoll/Embedded\ Programming/h5/Core/Startup/startup_stm32h562ritx.o: ../Core/Src/.claude/worktrees/optimistic-driscoll/Embedded\ Programming/h5/Core/Startup/startup_stm32h562ritx.s Core/Src/.claude/worktrees/optimistic-driscoll/Embedded\ Programming/h5/Core/Startup/subdir.mk
	arm-none-eabi-gcc -mcpu=cortex-m0plus -g3 -DDEBUG -c -x assembler-with-cpp -MMD -MP -MF"Core/Src/.claude/worktrees/optimistic-driscoll/Embedded Programming/h5/Core/Startup/startup_stm32h562ritx.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@" "$<"

clean: clean-Core-2f-Src-2f--2e-claude-2f-worktrees-2f-optimistic-2d-driscoll-2f-Embedded-20-Programming-2f-h5-2f-Core-2f-Startup

clean-Core-2f-Src-2f--2e-claude-2f-worktrees-2f-optimistic-2d-driscoll-2f-Embedded-20-Programming-2f-h5-2f-Core-2f-Startup:
	-$(RM) ./Core/Src/.claude/worktrees/optimistic-driscoll/Embedded\ Programming/h5/Core/Startup/startup_stm32h562ritx.d ./Core/Src/.claude/worktrees/optimistic-driscoll/Embedded\ Programming/h5/Core/Startup/startup_stm32h562ritx.o

.PHONY: clean-Core-2f-Src-2f--2e-claude-2f-worktrees-2f-optimistic-2d-driscoll-2f-Embedded-20-Programming-2f-h5-2f-Core-2f-Startup

