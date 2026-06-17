#ifndef FLIGHT_ESTIMATOR_H
#define FLIGHT_ESTIMATOR_H

#include "esc_app.h"
#include <stdint.h>

/* ============================================================================
 * L2200G motor parameters (Aerotech, 75mm)
 * Replace MOTOR_TOTAL_IMPULSE_NS if pre-flight eng file differs.
 * ========================================================================== */
#define MOTOR_PROP_MASS_KG          2.866f
#define MOTOR_CASE_MASS_KG          1.608f
#define MOTOR_TOTAL_IMPULSE_NS      5910.0f

/* Rocket dry mass without motor hardware */
#define ROCKET_BODY_MASS_KG         18.7f

/* Total initial mass (rocket + full motor) */
#define ESTIMATOR_M0_KG             (ROCKET_BODY_MASS_KG + MOTOR_CASE_MASS_KG + MOTOR_PROP_MASS_KG)

/* Mass after burnout (rocket + empty motor casing) */
#define ESTIMATOR_M_DRY_KG          (ROCKET_BODY_MASS_KG + MOTOR_CASE_MASS_KG)

/* If live-integrated impulse deviates from nominal by more than this
 * fraction at burnout, the live value is used in all post-burn calculations. */
#define MOTOR_IMPULSE_DEVIATION_THR 0.10f

/* Reference area [m²] — pi * (0.078359 m)^2 */
#define ESTIMATOR_REF_AREA_M2       0.019284f

typedef enum {
    EST_IDLE    = 0,    /* on pad, calibrating baro ground reference */
    EST_BOOST,          /* motor burning — integrating impulse        */
    EST_COAST,          /* coasting to apogee — KF + apogee predict   */
    EST_DESCENT,        /* past apogee — brakes retracted             */
} EstimatorPhase_t;

typedef struct {
    float           altitude_m;     /* estimated altitude AGL [m]        */
    float           velocity_mps;   /* estimated vertical velocity [m/s]  */
    float           apogee_est_m;   /* predicted apogee AGL [m]           */
    float           mass_kg;        /* current rocket mass [kg]           */
    float           impulse_ns;     /* integrated motor impulse [N·s]     */
    uint8_t         impulse_live;   /* 1 if live impulse is being used    */
    EstimatorPhase_t phase;
} EstimatorState_t;

/* Init — must be called once before any other function */
void Estimator_Init(void);

/* Phase transitions — called from flight_trigger.c */
void Estimator_SetBoost(void);   /* launch detected   */
void Estimator_SetCoast(void);   /* burnout detected  */

/* Main task — call every ESC_App_Task() iteration */
void Estimator_Task(void);

/* Query current state (safe to call from any context) */
void Estimator_GetState(EstimatorState_t *out);

/* Inject current deploy level so the coast predictor uses actual drag */
void  Estimator_SetDeployLevel(float level);
float Estimator_GetDeployLevel(void);

#endif /* FLIGHT_ESTIMATOR_H */
