#ifndef COMMAND_SEQUENCE_H
#define COMMAND_SEQUENCE_H

#include <stdint.h>

/* Initialize command sequence detector */
void Command_Sequence_Init(void);

/* Process a character for sequence detection */
void Command_Sequence_Process_Char(char c);

/* Check if deployment sequence was triggered */
uint8_t Command_Sequence_Check_Deploy_Trigger(void);

/* Clear deployment trigger */
void Command_Sequence_Clear_Deploy_Trigger(void);

/* Check if homing was triggered */
uint8_t Command_Sequence_Check_Homing_Trigger(void);

/* Clear homing trigger */
void Command_Sequence_Clear_Homing_Trigger(void);

#endif /* COMMAND_SEQUENCE_H */
