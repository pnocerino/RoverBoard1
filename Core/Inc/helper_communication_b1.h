#ifndef HELPER_COMUNICATION_B2_H
#define HELPER_COMUNICATION_B2_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "communication.h"

/* Defines used by helpers and main task */
#define CRC_POLY                (0x1021U)
#define CRC_INIT                (0xFFFFU)

/* Function Prototypes */
uint16_t calculate_crc16_local(const uint8_t* data, uint16_t length);
Decision_t decisionStop(Global_State_t* state);
uint8_t ReadUart_packet_B2(uint8_t* rx_buffer, uint16_t rx_len, Partial_State_B2_Bus* out_bus, int16_t* out_seq_num);
uint8_t ReadUart_decision_B2(uint8_t* rx_buffer, uint16_t rx_len, Decision_t* out_decision, int16_t* out_seq_num);
void serializePartialStateB1(Partial_State_B1_Bus* state, uint32_t sequence, uint8_t* buffer);
void serializeDecisionB1(Decision_t* decision, uint32_t sequence, uint8_t* buffer);
Global_State_t reconstructGlobalState(Partial_State_B1_Bus b1, Partial_State_B2_Bus b2);
Decision_t calculateDecision(Global_State_t* state_global);
uint8_t CheckDiscordance(Decision_t decision_A, Decision_t decision_B);

#ifdef __cplusplus
}
#endif

#endif /* HELPER_COMUNICATION_B2_H */
