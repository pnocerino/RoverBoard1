
#include "lib_battery.h"
#include <stdio.h> // Per snprintf

// Funzione privata per calcolare la percentuale (mapping)
static uint8_t _calc_percent(float volts, float min, float max) {
    if (volts >= max) return 100;
    if (volts <= min) return 0;

    float range = max - min;
    float percentage = ((volts - min) / range) * 100.0f;
    return (uint8_t)percentage;
}

void Batt_Init(Battery_Handle_t *hbatt, ADC_HandleTypeDef *hadc, float ratio, float min_v, float max_v) {
    // 1. Salva configurazione
    hbatt->hadc = hadc;
    hbatt->divider_ratio = ratio;
    hbatt->min_voltage = min_v;
    hbatt->max_voltage = max_v;
    hbatt->alarm_threshold = BATT_DEFAULT_ALARM_THRESHOLD;

    // 2. Azzera stati
    hbatt->raw_value = 0;
    hbatt->voltage_batt = 0.0f;
    hbatt->level_percent = 0;
    hbatt->is_critical = 0;

    // 3. Esegui Calibrazione ADC (Importante per precisione)
    if (HAL_ADCEx_Calibration_Start(hbatt->hadc, ADC_SINGLE_ENDED) != HAL_OK) {
        // Gestione errore opzionale
    }

    // 4. Avvia ADC in DMA (Circular Mode) scrivendo nel buffer della struct
    HAL_ADC_Start_DMA(hbatt->hadc, (uint32_t*)hbatt->dma_buffer, 1);
}

void Batt_Update(Battery_Handle_t *hbatt) {
    // A. Leggi dal buffer DMA (sempre aggiornato dall'hardware)
    hbatt->raw_value = hbatt->dma_buffer[0];

    // B. Calcola tensione al pin
    hbatt->voltage_pin = (hbatt->raw_value * ADC_REF_VOLTAGE) / ADC_RESOLUTION;

    // C. Calcola tensione batteria reale
    hbatt->voltage_batt = hbatt->voltage_pin * hbatt->divider_ratio;

    // D. Calcola percentuale
    hbatt->level_percent = _calc_percent(hbatt->voltage_batt, hbatt->min_voltage, hbatt->max_voltage);

    // E. Aggiorna flag allarme
    if (hbatt->level_percent < hbatt->alarm_threshold) {
        hbatt->is_critical = 1;
    } else {
        hbatt->is_critical = 0;
    }
}
