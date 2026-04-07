################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
S_SRCS += \
../Core/Src/.claude/worktrees/optimistic-driscoll/Embedded\ Programming/c5/C5testing/Core/Startup/startup_stm32c051k8tx.s 

OBJS += \
./Core/Src/.claude/worktrees/optimistic-driscoll/Embedded\ Programming/c5/C5testing/Core/Startup/startup_stm32c051k8tx.o 

S_DEPS += \
./Core/Src/.claude/worktrees/optimistic-driscoll/Embedded\ Programming/c5/C5testing/Core/Startup/startup_stm32c051k8tx.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/.claude/worktrees/optimistic-driscoll/Embedded\ Programming/c5/C5testing/Core/Startup/startup_stm32c051k8tx.o: ../Core/Src/.claude/worktrees/optimistic-driscoll/Embedded\ Programming/c5/C5testing/Core/Startup/startup_stm32c051k8tx.s Core/Src/.claude/worktrees/optimistic-driscoll/Embedded\ Programming/c5/C5testing/Core/Startup/subdir.mk
	arm-none-eabi-gcc -mcpu=cortex-m0plus -g3 -DDEBUG -c -x assembler-with-cpp -MMD -MP -MF"Core/Src/.claude/worktrees/optimistic-driscoll/Embedded Programming/c5/C5testing/Core/Startup/startup_stm32c051k8tx.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@" "$<"

clean: clean-Core-2f-Src-2f--2e-claude-2f-worktrees-2f-optimistic-2d-driscoll-2f-Embedded-20-Programming-2f-c5-2f-C5testing-2f-Core-2f-Startup

clean-Core-2f-Src-2f--2e-claude-2f-worktrees-2f-optimistic-2d-driscoll-2f-Embedded-20-Programming-2f-c5-2f-C5testing-2f-Core-2f-Startup:
	-$(RM) ./Core/Src/.claude/worktrees/optimistic-driscoll/Embedded\ Programming/c5/C5testing/Core/Startup/startup_stm32c051k8tx.d ./Core/Src/.claude/worktrees/optimistic-driscoll/Embedded\ Programming/c5/C5testing/Core/Startup/startup_stm32c051k8tx.o

.PHONY: clean-Core-2f-Src-2f--2e-claude-2f-worktrees-2f-optimistic-2d-driscoll-2f-Embedded-20-Programming-2f-c5-2f-C5testing-2f-Core-2f-Startup

