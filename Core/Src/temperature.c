#include "temperature.h"
#include "adc.h"
#include "tmpsensor.h"
#include "cmsis_os.h"

/* --- COSTANTI E DEFINIZIONI --- */
#define TEMP_ADC_INTREF_3V3    1501  /* Valore ADC teorico per 1.21V con VREF 3.3V */
#define TEMP_THRESHOLD         40.0f /* Soglia di temperatura in °C */
#define TEMP_TIMEOUT_MS        120000 /* 120 secondi */

/* --- STATI DELLA FSM --- */
typedef enum {
    TEMP_STATE_WAIT_FLAG,
    TEMP_STATE_START_TIMER,
    TEMP_STATE_ELAPSED_TIMER
} TempState_t;

/* --- VARIABILI LOCALI (STATIC) --- */
static TempState_t currentState = TEMP_STATE_WAIT_FLAG;
static float temperature = 0.0f;
static uint8_t temperature_flag = TEMP_OK;
static uint32_t timer_start = 0;

/* --- IMPLEMENTAZIONE FUNZIONI --- */

void Temperature_Init(void) {
    /* Inizializzazione Stato */
    currentState = TEMP_STATE_WAIT_FLAG;
    temperature_flag = TEMP_OK;
    temperature = 0.0f;
    timer_start = 0;

    /* Calibrazione ADC (se non già effettuata altrove) */
    HAL_ADCEx_Calibration_Start(&hadc5, ADC_SINGLE_ENDED);
}

void Temperature_Step(void) {
    /* 1. Lettura Temperatura (Start) */
    HAL_ADC_Start(&hadc5);
    if (HAL_ADC_PollForConversion(&hadc5, 10) == HAL_OK) {
        uint16_t raw_temp = HAL_ADC_GetValue(&hadc5);
        temperature = (float)TMPSENSOR_getTemperature(raw_temp, TEMP_ADC_INTREF_3V3);
    }
    HAL_ADC_Stop(&hadc5);

    /* 2. Macchina a Stati */
    switch (currentState) {
        case TEMP_STATE_WAIT_FLAG:
            /* Monitoraggio continuo */
            if (temperature >= TEMP_THRESHOLD) {
                timer_start = HAL_GetTick(); /* Avvia Timer */
                currentState = TEMP_STATE_START_TIMER;
            }
            break;

        case TEMP_STATE_START_TIMER:
            /* Caso 1: La temperatura torna normale (Reset) */
            if (temperature < TEMP_THRESHOLD) {
                temperature_flag = TEMP_OK; /* Reset Flag */
                currentState = TEMP_STATE_WAIT_FLAG;
            }
            /* Caso 2: Timeout scaduto (Surriscaldamento confermato) */
            else if ((HAL_GetTick() - timer_start) >= TEMP_TIMEOUT_MS) {
                currentState = TEMP_STATE_ELAPSED_TIMER;
            }
            break;

        case TEMP_STATE_ELAPSED_TIMER:
            /* Entry Action */
            temperature_flag = TEMP_NOT_OK;

            /* Transitions: GOTO WAIT_FLAG */
            /* Nota: Se la temperatura è ancora alta, al prossimo ciclo
               tornerà in START_TIMER, mantenendo il flag attivo finché non scende */
            currentState = TEMP_STATE_WAIT_FLAG;
            break;

        default:
            currentState = TEMP_STATE_WAIT_FLAG;
            break;
    }
}

float Temperature_GetValue(void) {
    return temperature;
}

uint8_t Temperature_GetFlag(void) {
    return temperature_flag;
}
