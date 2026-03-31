#ifndef AIRBRAKE_CONFIG_H
#define AIRBRAKE_CONFIG_H

/* ============================================================================
 * AIRBRAKE DEPLOYMENT CONFIGURATION
 * ============================================================================
 * Edit these values to customize the system behavior
 */

/* DEPLOYMENT SEQUENCE TIMING (in milliseconds) */
#define DEPLOY_INITIAL_WAIT_MS      2800    /* Wait before starting deployment */
#define DEPLOY_70DEG_DURATION_MS    3000    /* Hold 70° for this long */
#define DEPLOY_40DEG_DURATION_MS    3000    /* Hold 40° for this long */
#define DEPLOY_15DEG_DURATION_MS    3000    /* Hold 15° for this long */

/* DEPLOYMENT ANGLES (in degrees) */
#define DEPLOY_ANGLE_STAGE1         70.0f   /* First deployment angle */
#define DEPLOY_ANGLE_STAGE2         40.0f   /* Second deployment angle */
#define DEPLOY_ANGLE_STAGE3         15.0f   /* Third deployment angle */
#define DEPLOY_ANGLE_RETRACT        0.0f    /* Retracted position */

/* ANGLE-TO-POSITION CALIBRATION CURVE COEFFICIENTS
 * Based on: angle = C0 + C1*deltaZ + C2*deltaZ^2
 * Where deltaZ is in mm (position_um / 1000)
 */
#define ANGLE_CURVE_C0              0.383f
#define ANGLE_CURVE_C1              2.53f
#define ANGLE_CURVE_C2              -0.0154f

/* SD CARD LOGGING */
#define SD_LOG_RATE_HZ              50      /* Logging frequency (Hz) */
#define SD_LOG_FILENAME             "flight_log.csv"

/* ENCODER HOMING */
#define HOMING_CURRENT_THRESHOLD_MA 2000    /* Current spike threshold (mA) */
#define HOMING_SPEED_PERCENT        30      /* Homing speed (0-100%) */
#define HOMING_WAIT_TIME_MS         500     /* Wait after hitting limit (ms) */
#define HOMING_SAMPLE_TIME_MS       50      /* Min time before checking current */

/* COMMAND SEQUENCES */
#define CMD_DEPLOY_CHAR1            '6'     /* First char of deploy sequence */
#define CMD_DEPLOY_CHAR2            '7'     /* Second char of deploy sequence */
#define CMD_HOMING_CHAR             'H'     /* Homing command character */

#endif /* AIRBRAKE_CONFIG_H */
