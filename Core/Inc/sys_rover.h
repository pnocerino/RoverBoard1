#ifndef SYS_ROVER_H
#define SYS_ROVER_H

#include "main.h"
#include "lib_pid.h"
#include "drv_encoder.h"
#include "drv_motor_pwm.h"

/* --- 1. DEFINIZIONI INDICIZZAZIONE --- */
#define FRONT_LEFT  0
#define FRONT_RIGHT 1
#define REAR_LEFT   2
#define REAR_RIGHT  3

#define AXLE_FRONT  0
#define AXLE_REAR   1

/* --- 2. PARAMETRI FISICI & CONVERSIONI --- */
// MODIFICA QUI IL DIAMETRO DELLE TUE RUOTE (in Metri)
#define WHEEL_DIAMETER_M   0.16f
#define PI_CONST           3.14159265f

// Macro per convertire m/s <-> RPM
// Da m/s a RPM: (Speed * 60) / (PI * Diametro)
#define MS_TO_RPM(ms)      ((ms * 60.0f) / (PI_CONST * WHEEL_DIAMETER_M))

// Da RPM a m/s: (RPM * PI * Diametro) / 60
#define RPM_TO_MS(rpm)     ((rpm * PI_CONST * WHEEL_DIAMETER_M) / 60.0f)

#define RPM_TO_DEGS(rpm)	(rpm * 6)


/* --- 3. STRUTTURA PRINCIPALE ROVER --- */
typedef struct {
    // --- LIVELLO 1: HARDWARE & PID RUOTE ---
    MotorPWM_Handle_t motors[4];
    Encoder_Handle_t  encoders[4];
    PID_Handle_t      pids_wheels[4];

    // --- LIMITI FISICI ---
    float wheel_max_rpm[4];       // Limite hardware misurato per ogni ruota
    float system_speed_limit;     // Il "collo di bottiglia" del sistema (minimo dei 4 sopra)

    // --- LIVELLO 2: PID ASSI (Avantreno/Retrotreno) ---
    PID_Handle_t      pid_axle_front;
    PID_Handle_t      pid_axle_rear;

    // --- LIVELLO 3: PID MASTER (Velocità Lineare) ---
    PID_Handle_t      pid_master;

    // --- STATO DEL SISTEMA ---
    // Variabili Utente (m/s)
    float target_vel_ms;          // Target richiesto in m/s
    float measured_vel_ms;        // Velocità reale misurata in m/s

    // Variabili Interne (RPM)
    float target_global_rpm;      // Setpoint Lineare (RPM) dopo il clamping
    float measured_global_rpm;    // Feedback Lineare (RPM) medio

    // Variabile Sterzo (RPM Differenziale)
    float target_steering_rpm;    // Componente di rotazione (+ Destra, - Sinistra)

    // Output intermedi per debug
    float out_master_cmd;         // Uscita del Master PID

    float uturn_angle;
    uint8_t is_uturning;

    float smoothed_vel_ms;        // Velocità filtrata (Ramp) effettivamente usata
    uint8_t is_brake_assisted;       // 1 = Frenata assistita attiva
    float deceleration_rate;

} Rover_Handle_t;


/* --- 4. PROTOTIPI FUNZIONI --- */

// Inizializzazione e Configurazione
void Rover_Init(Rover_Handle_t *h_rover);
void Rover_UpdateLimits(Rover_Handle_t *h_rover);

// Comandi di Movimento (High Level)
void Rover_SetSpeed(Rover_Handle_t *h_rover, float speed_ms_target); // Movimento Lineare
void Rover_Stop(Rover_Handle_t *h_rover);                            // Stop Immediato
void Rover_Rotate_Spot(Rover_Handle_t *h_rover, float rpm_diff);     // Rotazione su se stesso
void Rover_Rotate_Pivot(Rover_Handle_t *h_rover, float rpm_speed, int pivot_side); // Rotazione su perno
void Rover_UTurn(Rover_Handle_t *h_rover, float rpm);

// Core Loop (Da chiamare nell'Interrupt Timer)
void Rover_Update_Loop(Rover_Handle_t *h_rover);


#endif // SYS_ROVER_H
