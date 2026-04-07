################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
S_SRCS += \
../Core/Src/.claude/worktrees/optimistic-driscoll/Embedded\ Programming/h7/Core/Startup/startup_stm32h7a3ritx.s 

OBJS += \
./Core/Src/.claude/worktrees/optimistic-driscoll/Embedded\ Programming/h7/Core/Startup/startup_stm32h7a3ritx.o 

S_DEPS += \
./Core/Src/.claude/worktrees/optimistic-driscoll/Embedded\ Programming/h7/Core/Startup/startup_stm32h7a3ritx.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/.claude/worktrees/optimistic-driscoll/Embedded\ Programming/h7/Core/Startup/startup_stm32h7a3ritx.o: ../Core/Src/.claude/worktrees/optimistic-driscoll/Embedded\ Programming/h7/Core/Startup/startup_stm32h7a3ritx.s Core/Src/.claude/worktrees/optimistic-driscoll/Embedded\ Programming/h7/Core/Startup/subdir.mk
	arm-none-eabi-gcc -mcpu=cortex-m0plus -g3 -DDEBUG -c -x assembler-with-cpp -MMD -MP -MF"Core/Src/.claude/worktrees/optimistic-driscoll/Embedded Programming/h7/Core/Startup/startup_stm32h7a3ritx.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@" "$<"

clean: clean-Core-2f-Src-2f--2e-claude-2f-worktrees-2f-optimistic-2d-driscoll-2f-Embedded-20-Programming-2f-h7-2f-Core-2f-Startup

clean-Core-2f-Src-2f--2e-claude-2f-worktrees-2f-optimistic-2d-driscoll-2f-Embedded-20-Programming-2f-h7-2f-Core-2f-Startup:
	-$(RM) ./Core/Src/.claude/worktrees/optimistic-driscoll/Embedded\ Programming/h7/Core/Startup/startup_stm32h7a3ritx.d ./Core/Src/.claude/worktrees/optimistic-driscoll/Embedded\ Programming/h7/Core/Startup/startup_stm32h7a3ritx.o

.PHONY: clean-Core-2f-Src-2f--2e-claude-2f-worktrees-2f-optimistic-2d-driscoll-2f-Embedded-20-Programming-2f-h7-2f-Core-2f-Startup

