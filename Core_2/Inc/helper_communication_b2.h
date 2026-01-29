#ifndef HELPER_COMUNICATION_B2_H
#define HELPER_COMUNICATION_B2_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "communication.h"

#define CRC_POLY                (0x1021U)
#define CRC_INIT                (0xFFFFU)

uint16_t calculate_crc16_local(const uint8_t* data, uint16_t length);
uint8_t ReadUart_packet_B1_Local(uint8_t *rx_buffer, uint16_t rx_len, Partial_State_B1_Bus *out_bus, int16_t *out_seq_num);
void serializePartialStateB2_Local(Partial_State_B2_Bus *state, uint8_t seq_num, uint8_t *buffer);
Global_State_t reconstructGlobalState(Partial_State_B1_Bus state_b1, Partial_State_B2_Bus state_b2);
Decision_t calculateDecision(Global_State_t* global);
Decision_t calculateDecision_B2(Partial_State_B2_Bus* partial_state);
uint8_t ReadUart_Decision_Local(uint8_t *rx_buffer, uint16_t rx_len, Decision_t *out_decision, int16_t *out_seq_num);
void serializeDecisionB2_Local(Decision_t *decision, uint8_t sequence_num, uint8_t *buffer);
uint8_t CheckDiscordance_Local(Decision_t decision_A, Decision_t decision_B);
#ifdef __cplusplus
}
#endif

#endif /* HELPER_COMUNICATION_B2_H */
