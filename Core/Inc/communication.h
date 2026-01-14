#ifndef COMUNICATION_H
#define COMUNICATION_H

#ifdef __cplusplus
extern "C" {
#endif

/* -------------------------------------------------------------------------- */
/* INCLUDES                                                                   */
/* -------------------------------------------------------------------------- */
#include <stdint.h>
#include "cmsis_os.h" /* Per TaskHandle_t */
#include "controller.h"

/* -------------------------------------------------------------------------- */
/* DEFINES                                                                    */
/* -------------------------------------------------------------------------- */
#define DIR_INIT            (0U)
#define DIR_STOP_EMERGENCY  (1U)
#define DIR_FORWARD         (2U)
#define DIR_REV             (3U)
#define DIR_LEFT_SPOT       (4U)
#define DIR_RIGHT_SPOT      (5U)
#define DIR_LEFT_PIVOT      (6U)
#define DIR_RIGHT_PIVOT     (7U)

#define LED_OFF             (8U)
#define LED_ON              (9U)
#define LED_AUTO            (10U)
#define LED_EMERGENCY		(11U)

#define TASK_COMUNICATION_PERIOD_MS (60U)
#define DT							0.01f   // 10ms (frequenza di chiamata)
#define MEMORY_WINDOW       		2.0    // 2 secondi di memoria ostacoli
#define MAX_SPEED           		1.34
#define DEADZONE            		0.10
#define OBSTACLE_THRESH_STOP 		75.0  // 75 cm
#define DIST_FAR_MIN        		150.0  // 1.5 m
#define DIST_FAR_MAX        		300.0  // 3.0 m
#define OBSTACLE_THRESH_STOP_EM 	400.0f

/* -------------------------------------------------------------------------- */
/* ENUMS                                                                      */
/* -------------------------------------------------------------------------- */
typedef enum {
    COMM_STATE_RECEIVE_RTS_B2 = 0,
    COMM_STATE_SEND_CTS,
    COMM_STATE_READ_HANDSHAKE,
    COMM_STATE_SEND_RTS_B1,
    COMM_STATE_SEND_HANDSHAKE1,
    COMM_STATE_REACH_CONNECTION,
    COMM_STATE_START,
    COMM_STATE_RECEIVE_PACKET_B2,
    COMM_STATE_READ_PACKET,
    COMM_STATE_PACKET_RECEIVED,
    COMM_STATE_IMPOSSIBLE_RECEIVE_FROM_B2,
    COMM_STATE_DEATH_B1,
    COMM_STATE_SERIALIZE_PACKET_B1,
    COMM_STATE_SEND_PACKET_B1,
    COMM_STATE_PACKET_SENT,
    COMM_STATE_IMPOSSIBLE_SEND_FROM_B1,
    COMM_STATE_GLOBAL_STATE_RECONSTRUCTION,
    COMM_STATE_TAKE_DECISION_B1,
    COMM_STATE_RECEIVE_DECISION_B2,
    COMM_STATE_READ_DECISION_B2,
    COMM_STATE_DECISION_RECEIVED,
    COMM_STATE_SERIALIZE_DECISION_B1,
    COMM_STATE_DECISION_SEND,
    COMM_STATE_DECISION_SENT,
    COMM_STATE_ACTUATE_DECISION
} CommState_t;

/* -------------------------------------------------------------------------- */
/* STRUCTS                                                                    */
/* -------------------------------------------------------------------------- */

typedef struct  __attribute__((packed)) {
    float b1_vel_fl;
    float b1_vel_fr;
    float b1_vel_rl;
    float b1_vel_rr;
    uint8_t battery_level;
    uint8_t temperatura;
    uint8_t emergenza;
    uint8_t degradato;
} Partial_State_B1_Bus;


typedef struct {
    Controller_Bus controller;
    uint16_t dL;
    uint16_t dC;
    uint16_t dR;
    uint8_t angolo;
    uint8_t degradato;
    uint8_t emergenza;
} Partial_State_B2_Bus;

typedef struct __attribute__((packed)) {
    int16_t setpoint;
    uint8_t dir;
    uint8_t led;
} Decision_t;

typedef struct {
    double b1_vel_fl;
    double b1_vel_fr;
    double b1_vel_rl;
    double b1_vel_rr;
    uint8_t b1_battery;
    uint8_t b1_temp;
    uint8_t b1_emergenza;
    uint8_t b1_degradato;
    uint16_t sonar_left;
    Controller_Bus controller;
    uint16_t sonar_center;
    uint16_t sonar_right;
    uint8_t b2_emergenza;
    uint8_t b2_degradato;
    uint8_t system_emergenza;
    uint8_t angolo;
    uint8_t system_degradato;
} Global_State_t;

/* -------------------------------------------------------------------------- */
/* EXPORTED VARIABLES & FUNCTIONS                                             */
/* -------------------------------------------------------------------------- */
extern TaskHandle_t xCommTaskHandle;

void Communication_Init(void);
void Communication_Step(void);
void Communication_GetActuationData(Global_State_t* g, Decision_t* d, uint8_t* disc);

#ifdef __cplusplus
}
#endif

#endif /* COMUNICATION_H */
