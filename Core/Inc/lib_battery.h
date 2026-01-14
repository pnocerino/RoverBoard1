#ifndef LIB_BATTERY_H_
#define LIB_BATTERY_H_

#include "main.h" // Per accedere a HAL e tipi standard

// --- CONFIGURAZIONE ---
// Soglia critica di default (es. 23%)
#define BATT_DEFAULT_ALARM_THRESHOLD  23
#define ADC_RESOLUTION                4095.0f
#define ADC_REF_VOLTAGE               3.3f

// --- STRUTTURA HANDLE ---
typedef struct {
    // --- PUNTATORI HARDWARE (Configurazione) ---
    ADC_HandleTypeDef *hadc;      // Puntatore all'ADC (es. &hadc3)

    // --- PARAMETRI FISICI (Configurazione) ---
    float divider_ratio;          // Rapporto partitore (es. 4.32)
    float min_voltage;            // 0% Batteria (es. 11.3V)
    float max_voltage;            // 100% Batteria (es. 12.6V)
    uint8_t alarm_threshold;      // % sotto la quale scatta l'allarme

    // --- BUFFER DMA INTERNO ---
    // Usiamo un array di 1 elemento per il DMA circolare
    uint32_t dma_buffer[1];

    // --- DATI DI USCITA (Output) ---
    uint32_t raw_value;           // Valore grezzo ADC
    float voltage_pin;            // Tensione al pin del micro
    float voltage_batt;           // Tensione reale batteria
    uint8_t level_percent;        // Percentuale 0-100
    uint8_t is_critical;          // Flag 1=Allarme, 0=OK

} Battery_Handle_t;

// --- PROTOTIPI FUNZIONI ---

// Inizializza l'hardware e la struttura
void Batt_Init(Battery_Handle_t *hbatt, ADC_HandleTypeDef *hadc, float ratio, float min_v, float max_v);

// Aggiorna i calcoli (da chiamare nel loop principale)
void Batt_Update(Battery_Handle_t *hbatt);

// Formatta una stringa pronta per la UART (opzionale, per comodità)
// buffer deve essere almeno di 64 byte
void Batt_GetMessage(Battery_Handle_t *hbatt, char *buffer, size_t len);

#endif /* LIB_BATTERY_H_ */