#include "helper_communication_b1.h"
#include "communication.h"
#include <string.h>
#include <math.h>

uint16_t calculate_crc16_local(const uint8_t* data, uint16_t length) {
    uint16_t crc = CRC_INIT;
    uint16_t poly = CRC_POLY;
    for (uint16_t i = 0; i < length; i++) {
        uint16_t curr = (uint16_t)data[i];
        crc ^= (curr << 8);
        for (uint8_t j = 0; j < 8; j++) {
            if ((crc & 0x8000U) != 0U) {
                crc = (crc << 1) ^ poly;
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}

Decision_t decisionStop(Global_State_t *state) {
    Decision_t decision = {0};

    decision.setpoint = 0;
    decision.dir = DIR_STOP_EMERGENCY;
    decision.led = LED_EMERGENCY;
    decision.decision_board = 1;
    decision.system_mode = EMERGENCY;

    return decision;
}

uint8_t CheckDiscordance(Decision_t decision_A, Decision_t decision_B) {
    uint8_t discordant = 0;

    if ((decision_A.dir != decision_B.dir) ||
        (decision_A.setpoint != decision_B.setpoint) ||
        (decision_A.led != decision_B.led) ||
        (decision_A.decision_board != decision_B.decision_board) ||
        (decision_A.system_mode != decision_B.system_mode)) {
        discordant = 1;
    }
    return discordant;
}

uint8_t ReadUart_decision_B2(uint8_t* rx_buffer, uint16_t rx_len, Decision_t* out_decision, int16_t* out_seq_num) {
    const uint8_t packet_len = 11;
    const uint8_t SOF = 0xAAU;
    const uint8_t EOF_BYTE = 0x55U;
    const uint8_t CMD_ID = 0x30U;

    if (rx_len < packet_len) return 0;

    int32_t found_index = -1;
    int32_t limit = (int32_t)rx_len - (int32_t)packet_len + 1;

    for (int32_t i = 0; i < limit; i++) {
        if (rx_buffer[i] == SOF) {
            found_index = i;
            break;
        }
    }

    if (found_index == -1) return 0;

    uint8_t* full_packet = &rx_buffer[found_index];

    uint8_t r_seq = full_packet[1];
    uint8_t r_cmd = full_packet[2];
    uint8_t r_crc_lo = full_packet[8];
    uint8_t r_crc_hi = full_packet[9];
    uint16_t r_crc_val = (uint16_t)r_crc_lo + ((uint16_t)r_crc_hi << 8);
    uint8_t r_eof = full_packet[10];

    if (r_eof != EOF_BYTE) return 0;
    if (r_cmd != CMD_ID) return 0;

    if (calculate_crc16_local(&full_packet[1], 7) != r_crc_val) return 0;

    *out_seq_num = (int16_t)r_seq;

    out_decision->dir = full_packet[4];
    out_decision->setpoint = (int16_t)(full_packet[5] | (full_packet[6] << 8));
    out_decision->led = full_packet[7];

    return 1;
}

uint8_t ReadUart_packet_B2(uint8_t* rx_buffer, uint16_t rx_len, Partial_State_B2_Bus* out_bus, int16_t* out_seq_num) {
    const uint8_t packet_len = 30;
    const uint8_t payload_len = 25;
    const uint8_t SOF = 0xAAU;
    const uint8_t EOF_BYTE = 0x55U;
    const uint8_t CMD_ID = 0x21U;

    if (rx_len < packet_len) return 0;

    int32_t found_index = -1;
    int32_t limit = (int32_t)rx_len - (int32_t)packet_len + 1;

    for (int32_t i = 0; i < limit; i++) {
        if (rx_buffer[i] == SOF) {
            found_index = i;
            break;
        }
    }

    if (found_index == -1) return 0;

    uint8_t* full_packet = &rx_buffer[found_index];

    uint8_t r_seq = full_packet[1];
    uint8_t r_cmd = full_packet[2];
    uint8_t r_len = full_packet[3];
    uint8_t r_eof = full_packet[29];

    if (r_eof != EOF_BYTE || r_cmd != CMD_ID || r_len != payload_len) return 0;

    *out_seq_num = (int16_t)r_seq;

    uint8_t* payload = &full_packet[4];

    /* A. POPOLAMENTO CONTROLLER */
    uint16_t ry_raw = (uint16_t)payload[0] | ((uint16_t)payload[1] << 8);
    out_bus->controller.ry = (int16_t)ry_raw;

    out_bus->controller.l2 = (uint16_t)payload[2] | ((uint16_t)payload[3] << 8);
    out_bus->controller.r2 = (uint16_t)payload[4] | ((uint16_t)payload[5] << 8);

    out_bus->controller.cross = payload[6];
    out_bus->controller.circle = payload[7];
    out_bus->controller.square = payload[8];
    out_bus->controller.triangle = payload[9];
    out_bus->controller.l1 = payload[10];
    out_bus->controller.r1 = payload[11];
    out_bus->controller.battery = payload[12];

    /* B. POPOLAMENTO SENSORI E STATI */
    out_bus->dL = (uint16_t)payload[13] | ((uint16_t)payload[14] << 8);
    out_bus->dC = (uint16_t)payload[15] | ((uint16_t)payload[16] << 8);
    out_bus->dR = (uint16_t)payload[17] | ((uint16_t)payload[18] << 8);
    out_bus->angolo = payload[19];

    out_bus->controller_status = payload[20];
    out_bus->sonar_status = payload[21];
    out_bus->mpu_status = payload[22];
    out_bus->degradato = payload[23];
    out_bus->emergenza = payload[24];

    return 1;
}

void serializePartialStateB1(Partial_State_B1_Bus* state, uint32_t sequence, uint8_t* buffer) {
    const uint8_t SOF = 0xAAU;
    const uint8_t EOF_BYTE = 0x55U;
    const uint8_t CMD_FULL_STATE = 0x20U;
    const uint8_t PAYLOAD_LEN = 27U;

    uint8_t idx = 0;

    /* --- Header --- */
    buffer[idx++] = SOF;
    buffer[idx++] = (uint8_t)(sequence & 0xFF);
    buffer[idx++] = CMD_FULL_STATE;
    buffer[idx++] = PAYLOAD_LEN;

    /* --- Payload --- */
    /* A. Velocità (Single -> 4 bytes ciascuno) */
    float v_fl = (float)state->b1_vel_fl;
    float v_fr = (float)state->b1_vel_fr;
    float v_rl = (float)state->b1_vel_rl;
    float v_rr = (float)state->b1_vel_rr;

    memcpy(&buffer[idx], &v_fl, 4); idx += 4;
    memcpy(&buffer[idx], &v_fr, 4); idx += 4;
    memcpy(&buffer[idx], &v_rl, 4); idx += 4;
    memcpy(&buffer[idx], &v_rr, 4); idx += 4;

    /* B. Dati scalari (uint8 -> 1 byte) */
    buffer[idx++] = (uint8_t)state->battery_level;
    buffer[idx++] = (uint8_t)state->temperatura;

    /* C. Photoresistor (uint32 -> 4 bytes) */
    uint32_t photo = state->photoresistor;
    memcpy(&buffer[idx], &photo, 4); idx += 4;

    /* D. Stati e Flag (uint8 -> 1 byte) */
    buffer[idx++] = (uint8_t)state->encoder_status;
    buffer[idx++] = (uint8_t)state->battery_status;
    buffer[idx++] = (uint8_t)state->temp_status;
    buffer[idx++] = (uint8_t)state->emergency;
    buffer[idx++] = (uint8_t)state->degraded;
    
    /* --- Footer --- */
    buffer[idx++] = EOF_BYTE;
}

Global_State_t reconstructGlobalState(Partial_State_B1_Bus b1, Partial_State_B2_Bus b2) {
    Global_State_t state = {0};

    /* 1. CALCOLO LOGICA DI SISTEMA */
    uint8_t is_sys_emergency = (b1.emergency > 0) || (b2.emergenza > 0);

    /* 2. COSTRUZIONE STRUTTURA GLOBALE */
    state.b1 = b1;

    state.b2 = b2;

    /* B. Assegnazione Flag di Sistema */
    state.system_emergenza = is_sys_emergency;
    state.comunication_degradato = 0;

    return state;
}

Decision_t calculateDecision(Global_State_t* state_global) {
    /* --- STATO PERSISTENTE --- */
    static Controller_Bus prev_ctrl = {0};
    static uint8_t led_state = LED_OFF;
    static float timer_memory_L = 0.0f;
    static float timer_memory_R = 0.0f;

    /* --- INIZIALIZZAZIONE OUTPUT --- */
    Decision_t decision = {0};
    decision.setpoint = 0;
    decision.dir = DIR_INIT;
    decision.led = led_state;
    decision.decision_board = 0;
    decision.system_mode = 0;

    uint8_t avoiding_obstacle = 0;

    /* --- CONTROLLO SICUREZZA --- */
    if (state_global->system_emergenza == 1) {
        decision.dir = DIR_STOP_EMERGENCY;
        decision.led = LED_EMERGENCY;
        decision.setpoint = 0;
        decision.system_mode = EMERGENCY;
        decision.decision_board = 2;
        prev_ctrl = state_global->b2.controller;
        return decision;
    }

    /* --- Lettura Sensori e Joystick --- */
    double dist_C = state_global->b2.dC;
    double dist_L = state_global->b2.dL;
    double dist_R = state_global->b2.dR;

    Controller_Bus current_ctrl = state_global->b2.controller;

    double input_y = (double)current_ctrl.ry / 512.0;
    if (current_ctrl.ry < 0) input_y = 0.0;

    double val_r2 = (double)current_ctrl.r2 / 1020.0;
    double val_l2 = (double)current_ctrl.l2 / 1020.0;
    double input_x = val_r2 - val_l2;

    if (fabs(input_y) < DEADZONE) input_y = 0.0;
    if (fabs(input_x) < DEADZONE) input_x = 0.0;

    float vel = 0.0f;

    /* --- AGGIORNAMENTO TIMER --- */
    if (timer_memory_L > 0.0f) timer_memory_L -= DT;
    if (timer_memory_R > 0.0f) timer_memory_R -= DT;

    /* --- LOGICA LED --- */
    uint8_t curr_r1 = (current_ctrl.r1 == 1);
    uint8_t prev_r1 = (prev_ctrl.r1 == 1);
    uint8_t curr_l1 = (current_ctrl.l1 == 1);
    uint8_t prev_l1 = (prev_ctrl.l1 == 1);

    if (curr_r1 && !prev_r1) {
        if (led_state == LED_OFF) led_state = LED_ON;
        else if (led_state == LED_ON) led_state = LED_AUTO;
        else if (led_state == LED_AUTO) led_state = LED_OFF;
    } else if (curr_l1 && !prev_l1) {
        if (led_state == LED_OFF) led_state = LED_AUTO;
        else if (led_state == LED_AUTO) led_state = LED_ON;
        else if (led_state == LED_ON) led_state = LED_OFF;
    }
    decision.led = led_state;

    /* --- LOGICA OSTACOLI PRIORITARIA (DEGRADATO) --- */
    uint8_t is_degraded = (state_global->b1.degraded == 1) || (state_global->b2.degradato == 1);

    if (is_degraded) {
        if (dist_C < 400.0 || dist_L < 400.0 || dist_R < 400.0) {
            decision.dir = DIR_STOP_EMERGENCY;
            decision.system_mode = DEGRADED;
            if (state_global->b1.encoder_status == 1) {
                decision.decision_board = 1;
            } else {
                decision.decision_board = 2;
            }
            decision.setpoint = 0;
            prev_ctrl = current_ctrl;
            return decision;
        }
    }

    /* --- LOGICA OSTACOLI NORMALE --- */
    if (input_y >= 0.0) {
        if (dist_C < OBSTACLE_THRESH_STOP || dist_L < OBSTACLE_THRESH_STOP || dist_R < OBSTACLE_THRESH_STOP) {
            avoiding_obstacle = 1;
            decision.dir = DIR_STOP_EMERGENCY;
            vel = 0.0f;
        } else {
            if (dist_L >= DIST_FAR_MIN && dist_L <= DIST_FAR_MAX) timer_memory_L = MEMORY_WINDOW;
            if (dist_R >= DIST_FAR_MIN && dist_R <= DIST_FAR_MAX) timer_memory_R = MEMORY_WINDOW;

            if (dist_C >= DIST_FAR_MIN && dist_C <= DIST_FAR_MAX) {
                if (timer_memory_L > 0.0f && timer_memory_L >= timer_memory_R) {
                    avoiding_obstacle = 1;
                    decision.dir = DIR_LEFT_SPOT;
                    vel = MAX_SPEED * 0.5f;
                } else if (timer_memory_R > 0.0f && timer_memory_R > timer_memory_L) {
                    avoiding_obstacle = 1;
                    decision.dir = DIR_RIGHT_SPOT;
                    vel = MAX_SPEED * 0.5f;
                }
            }
        }
    }

    /* --- GUIDA MANUALE --- */
    if (!avoiding_obstacle) {
        uint8_t curr_sqr = (current_ctrl.square == 1);
        uint8_t prev_sqr = (prev_ctrl.square == 1);
        uint8_t curr_cir = (current_ctrl.circle == 1);
        uint8_t prev_cir = (prev_ctrl.circle == 1);
        uint8_t curr_tri = (current_ctrl.triangle == 1);
        uint8_t prev_tri = (prev_ctrl.triangle == 1);
        uint8_t curr_crs = (current_ctrl.cross == 1);
        uint8_t prev_crs = (prev_ctrl.cross == 1);

        float steering_vel_spot = 0.0f;
        float steering_vel_pivot = 0.0f;
        float linear_vel = 0.0f;
        float vel_rev = 0.0f;

        if (curr_sqr && !prev_sqr && !curr_cir && !curr_tri && !curr_crs) {
            steering_vel_spot = (float)input_x * 0.5f;
        } else if (curr_cir && !prev_cir && !curr_sqr && !curr_tri && !curr_crs) {
            steering_vel_pivot = (float)input_x * 0.5f;
        } else if (curr_cir && !prev_cir && curr_crs && !prev_crs && !curr_sqr && !curr_tri) {
            vel_rev = -0.5f;
        } else if (curr_tri && !prev_tri && !curr_cir && !curr_crs && !curr_sqr) {
            linear_vel = 0.0f;
        } else {
            linear_vel = (float)input_y * MAX_SPEED;
        }

        if (fabs(steering_vel_spot) > 0.1f) {
            decision.dir = (steering_vel_spot > 0) ? DIR_RIGHT_SPOT : DIR_LEFT_SPOT;
            vel = fabs(steering_vel_spot);
        } else if (fabs(steering_vel_pivot) > 0.1f) {
            decision.dir = (steering_vel_pivot > 0) ? DIR_RIGHT_PIVOT : DIR_LEFT_PIVOT;
            vel = fabs(steering_vel_pivot);
        } else if (fabs(linear_vel) > 0.1f) {
            decision.dir = DIR_FORWARD;
            vel = fabs(linear_vel);
        } else if (vel_rev == -0.5f) {
            decision.dir = DIR_REV;
            vel = fabs(vel_rev);
        } else if (linear_vel == 0.0f && input_y > 0.0) {
            decision.dir = DIR_STOP_EMERGENCY;
            vel = 0.0f;
        } else {
            decision.dir = DIR_INIT;
            vel = 0.0f;
        }
    }

    /* --- APPLICAZIONE DEGRADATA --- */
    if (is_degraded) {
        decision.setpoint = (int16_t)(vel * 0.5f * 1000.0f);
        decision.system_mode = DEGRADED;
        if (state_global->b1.encoder_status == 1) {
            decision.decision_board = 1;
        } else {
            decision.decision_board = 2;
        }
    } else {
        decision.system_mode = NORMAL;
        decision.setpoint = (int16_t)(vel * 1000.0f);
    }

    prev_ctrl = current_ctrl;
    return decision;
}


void serializeDecisionB1(Decision_t* decision, uint32_t sequence, uint8_t* buffer) {
    const uint8_t TOTAL_PACKET_SIZE = 15U;
    const uint8_t SOF = 0xAAU;
    const uint8_t EOF_BYTE = 0x55U;
    const uint8_t CMD_DECISION = 0x30U;
    const uint8_t PAYLOAD_LEN = 6U;

    /* Initialize buffer with zeros (Padding) */
    memset(buffer, 0, TOTAL_PACKET_SIZE);

    buffer[0] = SOF;
    buffer[1] = (uint8_t)(sequence & 0xFF);
    buffer[2] = CMD_DECISION;
    buffer[3] = PAYLOAD_LEN;

    buffer[4] = (uint8_t)((uint16_t)decision->setpoint & 0xFFU);
    buffer[5] = (uint8_t)(((uint16_t)decision->setpoint >> 8) & 0xFFU);
    buffer[6] = decision->dir;
    buffer[7] = decision->led;
    buffer[8] = decision->decision_board;
    buffer[9] = decision->system_mode;

    buffer[10] = EOF_BYTE;
}
