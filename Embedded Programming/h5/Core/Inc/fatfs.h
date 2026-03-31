#ifndef FATFS_H
#define FATFS_H

#include <stdint.h>

/* NOTE: This file works WITHOUT FatFS middleware enabled!
 * The system will compile and run with UART logging.
 * To enable SD card logging later, add FatFS middleware in STM32CubeMX
 * and define FATFS_MIDDLEWARE_ENABLED in your project settings.
 */

#ifndef FATFS_MIDDLEWARE_ENABLED

/* Minimal type definitions for compilation without FatFS middleware */
typedef struct {
    uint8_t dummy;
} FATFS;

typedef struct {
    uint8_t dummy;
} FIL;

typedef enum {
    FR_OK = 0,
    FR_DISK_ERR,
    FR_NOT_READY,
    FR_NO_FILE,
    FR_NOT_OPENED,
    FR_INVALID_OBJECT
} FRESULT;

typedef uint32_t UINT;

/* File access mode flags */
#define FA_READ             0x01
#define FA_WRITE            0x02
#define FA_CREATE_NEW       0x04
#define FA_CREATE_ALWAYS    0x08
#define FA_OPEN_ALWAYS      0x10

/* Stub functions when FatFS is not available */
static inline FRESULT f_mount(FATFS* fs, const char* path, uint8_t opt) { (void)fs; (void)path; (void)opt; return FR_OK; }
static inline FRESULT f_open(FIL* fp, const char* path, uint8_t mode) { (void)fp; (void)path; (void)mode; return FR_NOT_READY; }
static inline FRESULT f_write(FIL* fp, const void* buff, UINT btw, UINT* bw) { (void)fp; (void)buff; (void)btw; if(bw) *bw=0; return FR_NOT_READY; }
static inline FRESULT f_sync(FIL* fp) { (void)fp; return FR_NOT_READY; }
static inline FRESULT f_close(FIL* fp) { (void)fp; return FR_OK; }

extern char SDPath[4];

#else

/* When FatFS middleware is enabled, include the real headers */
#include "ff.h"
#include "ff_gen_drv.h"
#include "sd_diskio.h"

extern char SDPath[4];
extern FATFS SDFatFS;
extern FIL SDFile;

#endif

/* Common function regardless of middleware status */
void MX_FATFS_Init(void);

#endif /* FATFS_H */
