#include "drv_encoder.h"

void Drv_Encoder_Init(Encoder_Handle_t *h_enc, TIM_HandleTypeDef *htim, uint32_t ticks, uint32_t period_ms) {
    h_enc->htim = htim;
    h_enc->ticks_per_rev = ticks;
    h_enc->sampling_period_ms = period_ms;

    h_enc->_last_time_ms = HAL_GetTick();
    h_enc->_last_counter_val = 0;
    h_enc->_current_rpm = 0.0f;

    // Avvia il timer encoder hardware
    HAL_TIM_Encoder_Start(htim, TIM_CHANNEL_ALL);
}

uint8_t Drv_Encoder_Update(Encoder_Handle_t *h_enc) {
    uint32_t now = HAL_GetTick();

    // Controllo non bloccante del tempo trascorso
    if ((now - h_enc->_last_time_ms) >= h_enc->sampling_period_ms) {

        uint32_t curr_counter = __HAL_TIM_GET_COUNTER(h_enc->htim);

        /* * TRUCCO PROFESSIONALE PER OVERFLOW 16-BIT:
         * Eseguendo il cast a (int16_t) della differenza, il compilatore C
         * gestisce automaticamente l'overflow/underflow aritmetico in complemento a due.
         * Non servono if/else complessi per gestire il passaggio 65535 -> 0.
         */
        int16_t diff = (int16_t)(curr_counter - h_enc->_last_counter_val);

        // Calcolo RPM
        float sampling_seconds = h_enc->sampling_period_ms / 1000.0f;
        float revs = (float)diff / (float)h_enc->ticks_per_rev;
        h_enc->_current_rpm = (revs / sampling_seconds) * 60.0f;

        // Aggiornamento stato
        h_enc->_last_counter_val = curr_counter;
        h_enc->_last_time_ms = now;

        return 1; // Nuovo dato disponibile
    }

    return 0; // Nessun aggiornamento in questo ciclo
}

float Drv_Encoder_GetRPM_FixedDT(Encoder_Handle_t *h_enc, float dt_seconds) {
    uint32_t curr_counter = __HAL_TIM_GET_COUNTER(h_enc->htim);
    int16_t diff = (int16_t)(curr_counter - h_enc->_last_counter_val);

    // Aggiorna subito lo stato per la prossima lettura
    h_enc->_last_counter_val = curr_counter;

    // Calcolo RPM
    float revs = (float)diff / (float)h_enc->ticks_per_rev;
    float rpm = (revs / dt_seconds) * 60.0f;

    h_enc->_current_rpm = rpm; // Salva anche nello struct
    return rpm;
}

float Drv_Encoder_GetRPM(Encoder_Handle_t *h_enc) {
    return h_enc->_current_rpm;
}
