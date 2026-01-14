#ifndef LIB_PID_H
#define LIB_PID_H

#include <stdint.h>

typedef struct {
    // Guadagni
    float Kp;
    float Ki;
    float Kd;

    // Parametri temporali e limiti
    float T_sample_s;    // Tempo di campionamento in secondi
    float output_min;
    float output_max;

    // Stato interno (Memoria)
    float integral_term;
    float prev_measurement;
} PID_Handle_t;

/**
 * @brief Inizializza il controller PID
 */
void PID_Init(PID_Handle_t *h_pid, float Kp, float Ki, float Kd, float T_sample_s, float out_min, float out_max);

/**
 * @brief Resetta l'integrale (utile quando si ferma il motore)
 */
void PID_Reset(PID_Handle_t *h_pid);

/**
 * @brief Calcola l'output del PID
 * @param setpoint: Il valore desiderato (es. RPM target)
 * @param measurement: Il valore letto (es. RPM attuali)
 * @return Output calcolato
 */
float PID_Compute(PID_Handle_t *h_pid, float setpoint, float measurement);

#endif // LIB_PID_H
