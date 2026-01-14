#include "reading.h"
#include "adc.h"
#include "tim.h"
#include "cmsis_os.h"
#include <stdlib.h>
#include <math.h>
#include "drv_encoder.h"
#include "lib_battery.h"
#include "tmpsensor.h"
#include "temperature.h"

/* -------------------------------------------------------------------------- */
/* TYPES & ENUMS                                                              */
/* -------------------------------------------------------------------------- */

typedef enum {
    READING_STATE_CHECK_ENCODERS,
    READING_STATE_CHECK_SENSORS,
    READING_STATE_ACQUIRE_MUTEX,
    READING_STATE_UPDATE_PARTIAL_STATE,
    READING_STATE_RELEASE_MUTEX
} ReadingState_t;

/* -------------------------------------------------------------------------- */
/* DEFINES                                                                    */
/* -------------------------------------------------------------------------- */
#define ENCODER_TICKS_REV      2448
#define ENCODER_PERIOD_MS      10
#define ENCODER_TOLERANCE_RPM  50.0f
#define TEMP_ADC_INTREF_3V3    1501 /* Valore ADC teorico per 1.21V con VREF 3.3V */

/* -------------------------------------------------------------------------- */
/* GLOBAL VARIABLES                                                           */
/* -------------------------------------------------------------------------- */



/* Task Handle (defined in app_freertos.c, declared extern here if needed, 
   but we use the one passed or global) */
// extern osThreadId_t readingTaskHandle; 

/* External Variables */
extern osMutexId_t partialStateMutexHandle;
extern osMutexId_t commDataMutexHandle;

/* Current State */
static ReadingState_t currentState = READING_STATE_CHECK_ENCODERS;

/* Task Local Variables (Persistent) */
static Encoder_Handle_t h_enc_fl;
static Encoder_Handle_t h_enc_fr;
static Encoder_Handle_t h_enc_rl;
static Encoder_Handle_t h_enc_rr;
static Battery_Handle_t h_batt;

/* Shared State Buffers */
static int32_t vel_fl = 0;
static int32_t vel_fr = 0;
static int32_t vel_rl = 0;
static int32_t vel_rr = 0;
static float temperatura = 0.0f;
static float battery_level = 0.0f;
static uint8_t degraded = 0;
static uint8_t emergency = 0;
static uint8_t encoder_flag = ENC_OK;
static uint8_t temperature_flag = 0;
static uint8_t status_battery = BATTERY_OK;

/* -------------------------------------------------------------------------- */
/* PRIVATE FUNCTION PROTOTYPES                                                */
/* -------------------------------------------------------------------------- */

static void up_state(float temp, int32_t v_fl, int32_t v_fr, int32_t v_rl, int32_t v_rr, float batt, uint8_t deg, uint8_t emerg);
static uint8_t check_encoders(int32_t *v_fl, int32_t *v_fr, int32_t *v_rl, int32_t *v_rr);

/* -------------------------------------------------------------------------- */
/* TASK FUNCTIONS                                                             */
/* -------------------------------------------------------------------------- */

void Reading_Init(void)
{
    /* --- INITIALIZATION --- */
    degraded = 0;
    emergency = 0;
    currentState = READING_STATE_CHECK_ENCODERS;

    /* Initialize Encoders */
    Drv_Encoder_Init(&h_enc_fl, &htim1, ENCODER_TICKS_REV, ENCODER_PERIOD_MS);
    Drv_Encoder_Init(&h_enc_fr, &htim2, ENCODER_TICKS_REV, ENCODER_PERIOD_MS);
    Drv_Encoder_Init(&h_enc_rl, &htim3, ENCODER_TICKS_REV, ENCODER_PERIOD_MS);
    Drv_Encoder_Init(&h_enc_rr, &htim5, ENCODER_TICKS_REV, ENCODER_PERIOD_MS);

    /* Initialize Battery (Handles ADC3 Calibration and DMA Start) */
    Batt_Init(&h_batt, &hadc3, 10.0f, 10.5f, 12.6f);

}

void Reading_Step(void)
{
    switch(currentState)
    {
        case READING_STATE_CHECK_ENCODERS:
            /* Update Encoders */
            Drv_Encoder_Update(&h_enc_fl);
            Drv_Encoder_Update(&h_enc_fr);
            Drv_Encoder_Update(&h_enc_rl);
            Drv_Encoder_Update(&h_enc_rr);

            /* Get RPMs */
            float rpm_fl = Drv_Encoder_GetRPM(&h_enc_fl);
            float rpm_fr = Drv_Encoder_GetRPM(&h_enc_fr);
            float rpm_rl = Drv_Encoder_GetRPM(&h_enc_rl);
            float rpm_rr = Drv_Encoder_GetRPM(&h_enc_rr);

            /* Store for up_state */
            vel_fl = (int32_t)rpm_fl;
            vel_fr = (int32_t)rpm_fr;
            vel_rl = (int32_t)rpm_rl;
            vel_rr = (int32_t)rpm_rr;

            encoder_flag = check_encoders(&vel_fl, &vel_fr, &vel_rl, &vel_rr);

            if (encoder_flag != ENC_NOT_OK) {
                degraded = 0;
            } else {
                degraded = 1;
            }
            currentState = READING_STATE_CHECK_SENSORS;
            break;

        case READING_STATE_CHECK_SENSORS:
            /* Read Temperature */
            /* Get data from Temperature Task */
            temperatura = Temperature_GetValue();
            temperature_flag = Temperature_GetFlag();
            
            /* Check Battery */
            Batt_Update(&h_batt);
            battery_level = h_batt.voltage_batt;
            if (h_batt.is_critical) status_battery = BATTERY_NOT_OK;
            else status_battery = BATTERY_OK;

            /* Determine Emergency State */
            if (status_battery == BATTERY_OK && temperature_flag == TEMP_OK) {
                emergency = 0;
            } else {
                emergency = 1;
            }
            
            currentState = READING_STATE_ACQUIRE_MUTEX;
            break;

        case READING_STATE_ACQUIRE_MUTEX:
            /* Acquire Mutex */
            if (osMutexAcquire(partialStateMutexHandle, 2) == osOK) {
                currentState = READING_STATE_UPDATE_PARTIAL_STATE;
            }
            break;

        case READING_STATE_UPDATE_PARTIAL_STATE:
            /* Update Shared State */
            up_state(temperatura, vel_fl, vel_fr, vel_rl, vel_rr, battery_level, degraded, emergency);
            
            currentState = READING_STATE_RELEASE_MUTEX;
            break;

        case READING_STATE_RELEASE_MUTEX:
            /* Release Mutex */
            osMutexRelease(partialStateMutexHandle);
            currentState = READING_STATE_CHECK_ENCODERS;
            break;

        default:
            currentState = READING_STATE_CHECK_ENCODERS;
            break;
    }
}

/* -------------------------------------------------------------------------- */
/* HELPER FUNCTIONS                                                           */
/* -------------------------------------------------------------------------- */

static uint8_t check_encoders(int32_t *v_fl, int32_t *v_fr, int32_t *v_rl, int32_t *v_rr) {
    
    int32_t tolerance = 1.5; // Increased tolerance as raw counters might differ more than 1

    // Check Front Axle (FL - FR)
    if (abs(*v_fl - *v_fr) > tolerance) {
        return ENC_NOT_OK;
    }

    // Check Rear Axle (RL - RR)
    if (abs(*v_rl - *v_rr) > tolerance) {
        return ENC_NOT_OK;
    }

    // Check Left Side (FL - RL)
    if (abs(*v_fl - *v_rl) > tolerance) {
        return ENC_NOT_OK;
    }

    // Check Right Side (FR - RR)
    if (abs(*v_fr - *v_rr) > tolerance) {
        return ENC_NOT_OK;
    }

    return ENC_OK;
}


static void up_state(float temp, int32_t v_fl, int32_t v_fr, int32_t v_rl, int32_t v_rr, float batt, uint8_t deg, uint8_t emerg) {
    partial_state_b1.temperatura = temp;
    partial_state_b1.b1_vel_fl = v_fl;
    partial_state_b1.b1_vel_fr = v_fr;
    partial_state_b1.b1_vel_rl = v_rl;
    partial_state_b1.b1_vel_rr = v_rr;
    partial_state_b1.battery_level = batt;
    partial_state_b1.degradato = deg;
    partial_state_b1.emergenza = emerg;
}
