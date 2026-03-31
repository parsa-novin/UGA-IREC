#ifndef SD_LOGGER_H
#define SD_LOGGER_H

#include <stdint.h>

/* Initialize SD card logging system */
uint8_t SD_Logger_Init(void);

/* Start logging (50 Hz) */
void SD_Logger_Start(void);

/* Stop logging */
void SD_Logger_Stop(void);

/* Task to be called in main loop */
void SD_Logger_Task(void);

/* Check if logging is active */
uint8_t SD_Logger_Is_Active(void);

#endif /* SD_LOGGER_H */
