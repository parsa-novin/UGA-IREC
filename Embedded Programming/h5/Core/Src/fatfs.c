#include "fatfs.h"

#ifndef FATFS_MIDDLEWARE_ENABLED

/* Stub implementation when FatFS middleware is not enabled */
char SDPath[4] = "0:/";

void MX_FATFS_Init(void)
{
    /* No FatFS middleware - SD logging will be disabled */
    /* To enable: Add FatFS middleware in STM32CubeMX and define FATFS_MIDDLEWARE_ENABLED */
}

#else

/* Full implementation when FatFS middleware is enabled */
char SDPath[4];
FATFS SDFatFS;
FIL SDFile;

void MX_FATFS_Init(void)
{
    /* Link the SD disk I/O driver */
    if (FATFS_LinkDriver(&SD_Driver, SDPath) == 0)
    {
        /* Successfully linked */
    }
}

#endif
