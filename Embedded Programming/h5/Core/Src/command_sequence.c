#include "command_sequence.h"
#include <stdint.h>
#include <ctype.h>

/* Sequence detection states */
typedef enum {
    SEQ_IDLE,
    SEQ_GOT_6,    /* Got '6', waiting for '7' */
} SequenceState_t;

static volatile SequenceState_t s_seqState = SEQ_IDLE;
static volatile uint8_t s_deployTrigger = 0;
static volatile uint8_t s_homingTrigger = 0;

void Command_Sequence_Init(void)
{
    s_seqState = SEQ_IDLE;
    s_deployTrigger = 0;
    s_homingTrigger = 0;
}

void Command_Sequence_Process_Char(char c)
{
    /* Convert to uppercase for case-insensitive comparison */
    char upper_c = (char)toupper((unsigned char)c);

    /* Check for homing command 'H' */
    if (upper_c == 'H')
    {
        s_homingTrigger = 1;
        s_seqState = SEQ_IDLE;  /* Reset sequence state */
        return;
    }

    /* Check for deployment sequence '6' then '7' */
    switch (s_seqState)
    {
        case SEQ_IDLE:
            if (c == '6')
            {
                s_seqState = SEQ_GOT_6;
            }
            break;

        case SEQ_GOT_6:
            if (c == '7')
            {
                /* Sequence complete! */
                s_deployTrigger = 1;
                s_seqState = SEQ_IDLE;
            }
            else
            {
                /* Any other character breaks the chain */
                s_seqState = SEQ_IDLE;

                /* Check if this character is '6' to start new sequence */
                if (c == '6')
                {
                    s_seqState = SEQ_GOT_6;
                }
            }
            break;
    }
}

uint8_t Command_Sequence_Check_Deploy_Trigger(void)
{
    return s_deployTrigger;
}

void Command_Sequence_Clear_Deploy_Trigger(void)
{
    s_deployTrigger = 0;
}

uint8_t Command_Sequence_Check_Homing_Trigger(void)
{
    return s_homingTrigger;
}

void Command_Sequence_Clear_Homing_Trigger(void)
{
    s_homingTrigger = 0;
}
