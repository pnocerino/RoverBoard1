#ifndef DRV_ENCODER_H
#define DRV_ENCODER_H

#include "main.h" // Per le definizioni HAL
#include <stdint.h>

/**
 * @brief Struttura di configurazione e stato dell'encoder.
 * Mantiene lo stato incapsulato per permettere istanze multiple.
 */
typedef struct {
    TIM_HandleTypeDef *htim;       // Handle del timer hardware
    uint32_t ticks_per_rev;        // Impulsi per rivoluzione (es. 2448)
    uint32_t sampling_period_ms;   // Periodo di campionamento desiderato

    // Variabili di stato interne (non modificare manualmente)
    uint32_t _last_time_ms;
    uint32_t _last_counter_val;
    float    _current_rpm;
} Encoder_Handle_t;

// Prototipi
void Drv_Encoder_Init(Encoder_Handle_t *h_enc, TIM_HandleTypeDef *htim, uint32_t ticks, uint32_t period_ms);
uint8_t Drv_Encoder_Update(Encoder_Handle_t *h_enc); // Restituisce 1 se il calcolo è avvenuto
float Drv_Encoder_GetRPM(Encoder_Handle_t *h_enc);
float Drv_Encoder_GetRPM_FixedDT(Encoder_Handle_t *h_enc, float dt_seconds);

#endif // DRV_ENCODER_H
