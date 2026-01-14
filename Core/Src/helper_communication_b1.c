#include <helper_communication_b1.h>
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

uint8_t ReadUart_packet_B1(uint8_t* rx_buffer, uint16_t rx_len, Partial_State_B1_Bus* out_bus, int16_t* out_seq_num) {
    const uint8_t packet_len = PACKET_B1_SIZE;
    const uint8_t SOF = 0xAAU;
    const uint8_t EOF_BYTE = 0x55U;
    const uint8_t CMD_ID = 0x20U;

    if (rx_len < packet_len) return 0;

    int32_t found_index = -1;
    int32_t limit = (int32_t)rx_len - (int32_t)packet_len + 1;

    /* Search SOF */
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
    
    uint8_t r_crc_lo = full_packet[12];
    uint8_t r_crc_hi = full_packet[13];
    uint16_t r_crc_val = (uint16_t)r_crc_lo + ((uint16_t)r_crc_hi << 8);
    
    uint8_t r_eof = full_packet[14];

    if (r_eof != EOF_BYTE) return 0;
    if (r_cmd != CMD_ID) return 0;

    /* CRC Check */
    if (calculate_crc16_local(&full_packet[1], 11) != r_crc_val) return 0;

    *out_seq_num = (int16_t)r_seq;
    
    out_bus->temperatura = (double)full_packet[4];
    out_bus->b1_vel_fl = (double)full_packet[5];
    out_bus->b1_vel_fr = (double)full_packet[6];
    out_bus->b1_vel_rl = (double)full_packet[7];
    out_bus->b1_vel_rr = (double)full_packet[8];
    out_bus->battery_level = (double)full_packet[9];
    out_bus->degradato = (full_packet[10] != 0U) ? 1U : 0U;
    out_bus->emergenza = (full_packet[11] != 0U) ? 1U : 0U;

    return 1;
}

void serializePartialStateB1(Partial_State_B1_Bus* state, uint32_t sequence, uint8_t* buffer) {
    /*const uint8_t SOF = 0xAAU;
    const uint8_t EOF_BYTE = 0x55U;
    const uint8_t CMD_B2_STATE = 0x21U;
    const uint8_t PAYLOAD_LEN = 21U;

    uint8_t idx = 0;

    buffer[idx++] = SOF;
    buffer[idx++] = (uint8_t)(sequence & 0xFF);
    buffer[idx++] = CMD_B2_STATE;
    buffer[idx++] = PAYLOAD_LEN;

    buffer[idx++] = (uint8_t)((uint16_t)state->controller.ry & 0xFFU);
    buffer[idx++] = (uint8_t)(((uint16_t)state->controller.ry >> 8) & 0xFFU);
    
    buffer[idx++] = (uint8_t)(state->controller.l2 & 0xFFU);
    buffer[idx++] = (uint8_t)((state->controller.l2 >> 8) & 0xFFU);
    
    buffer[idx++] = (uint8_t)(state->controller.r2 & 0xFFU);
    buffer[idx++] = (uint8_t)((state->controller.r2 >> 8) & 0xFFU);

    buffer[idx++] = state->controller.square;
    buffer[idx++] = state->controller.triangle;
    buffer[idx++] = state->controller.circle;
    buffer[idx++] = state->controller.cross;
    buffer[idx++] = state->controller.l1;
    buffer[idx++] = state->controller.r1;

    buffer[idx++] = (uint8_t)(state->dL & 0xFFU);
    buffer[idx++] = (uint8_t)((state->dL >> 8) & 0xFFU);
    
    buffer[idx++] = (uint8_t)(state->dC & 0xFFU);
    buffer[idx++] = (uint8_t)((state->dC >> 8) & 0xFFU);
    
    buffer[idx++] = (uint8_t)(state->dR & 0xFFU);
    buffer[idx++] = (uint8_t)((state->dR >> 8) & 0xFFU);

    buffer[idx++] = state->angolo;
    buffer[idx++] = state->degradato;
    buffer[idx++] = state->emergenza;

    uint16_t crc = calculate_crc16_local(&buffer[1], idx - 1);
    
    buffer[idx++] = (uint8_t)(crc & 0xFFU);
    buffer[idx++] = (uint8_t)((crc >> 8) & 0xFFU);

    buffer[idx++] = EOF_BYTE;*/
}

Global_State_t reconstructGlobalState(Partial_State_B1_Bus b1, Partial_State_B2_Bus b2) {
    Global_State_t g = {0};

    g.b1_vel_fl    = b1.b1_vel_fl;
    g.b1_vel_fr    = b1.b1_vel_fr;
    g.b1_vel_rl    = b1.b1_vel_rl;
    g.b1_vel_rr    = b1.b1_vel_rr;
    g.b1_battery   = b1.battery_level;
    g.b1_temp      = b1.temperatura;
    g.b1_emergenza = b1.emergenza;
    g.b1_degradato = b1.degradato;

    g.controller = b2.controller;

    g.sonar_left   = (double)b2.dL;
    g.sonar_center = (double)b2.dC;
    g.sonar_right  = (double)b2.dR;

    g.b2_emergenza = b2.emergenza;
    g.b2_degradato = b2.degradato;
    g.angolo       = (double)b2.angolo;

    g.system_emergenza = g.b1_emergenza || g.b2_emergenza;

    return g;
}

Decision_t calculateDecision(Global_State_t* global) {

    /* --- GESTIONE STATO PERSISTENTE (MEMORIA) --- */
    static Controller_Bus prev_ctrl = {0};
    static uint8_t led_state = LED_OFF;
    static float timer_memory_L = 0.0f;
    static float timer_memory_R = 0.0f;

    /* --- INIZIALIZZAZIONE OUTPUT --- */
    Decision_t decision = {0};
    decision.setpoint = 0;
    decision.dir = DIR_INIT;
    decision.led = led_state;

    if (global->system_emergenza != 0U) {
        decision.dir = DIR_STOP_EMERGENCY;
        decision.setpoint = 0;
        return decision;
    }

    /* --- PARAMETRI SENSORI --- */
    float dist_C = (float)global->sonar_center;
    float dist_L = (float)global->sonar_left;
    float dist_R = (float)global->sonar_right;

    /* --- A. Lettura Joystick --- */
    Controller_Bus ctrl = global->controller;

    double input_y = (double)ctrl.ry / 512.0;
    if (ctrl.ry < 0) input_y = 0.0;

    double val_r2 = (double)ctrl.r2 / 1020.0;
    double val_l2 = (double)ctrl.l2 / 1020.0;
    double input_x = val_r2 - val_l2;

    if (fabs(input_y) < DEADZONE) input_y = 0.0;
    if (fabs(input_x) < DEADZONE) input_x = 0.0;

    uint8_t avoiding_obstacle = 0;
    float vel = 0.0f;

    /* =========================================================
       AGGIORNAMENTO TIMER (IL CUORE DEL NON-BLOCKING)
       ========================================================= */
    if (timer_memory_L > 0.0f) timer_memory_L -= DT;
    if (timer_memory_R > 0.0f) timer_memory_R -= DT;

    /* Degraded Mode Safety Check (< 400cm) */
    if (global->b1_degradato || global->b2_degradato) {
        if (dist_C < 400.0f || dist_L < 400.0f || dist_R < 400.0f) {
            avoiding_obstacle = 1;
            decision.dir = DIR_STOP_EMERGENCY;
            vel = 0.0f;
        }
    }

    /* =========================================================
       LOGICA OSTACOLI
       ========================================================= */
    if (input_y >= 0.0) {
        /* 1. STOP DI EMERGENZA (Priorità Massima) */
        if (dist_C < OBSTACLE_THRESH_STOP || dist_L < OBSTACLE_THRESH_STOP || dist_R < OBSTACLE_THRESH_STOP) {
            avoiding_obstacle = 1;
            decision.dir = DIR_STOP_EMERGENCY;
            vel = 0.0f;
        } else {
            /* 2. RILEVAZIONE LATERALE (ARMAMENTO TIMER) */
            if (dist_L >= DIST_FAR_MIN && dist_L <= DIST_FAR_MAX) {
                timer_memory_L = MEMORY_WINDOW;
            }
            if (dist_R >= DIST_FAR_MIN && dist_R <= DIST_FAR_MAX) {
                timer_memory_R = MEMORY_WINDOW;
            }

            /* 3. RILEVAZIONE CENTRALE (VERIFICA SEQUENZA) */
            if (dist_C >= DIST_FAR_MIN && dist_C <= DIST_FAR_MAX) {
                /* CASO A: Sequenza SINISTRA -> CENTRO */
                if (timer_memory_L > 0.0f && timer_memory_L >= timer_memory_R) {
                    avoiding_obstacle = 1;
                    decision.dir = DIR_LEFT_SPOT;
                    vel = MAX_SPEED * 0.5f;
                }
                /* CASO B: Sequenza DESTRA -> CENTRO */
                else if (timer_memory_R > 0.0f && timer_memory_R > timer_memory_L) {
                    avoiding_obstacle = 1;
                    decision.dir = DIR_RIGHT_SPOT;
                    vel = MAX_SPEED * 0.5f;
                }
            }
        }
    }

    /* --- 6. GUIDA NORMALE --- */
    if (!avoiding_obstacle) {
        /* Logica LED */
        if (ctrl.r1 && !prev_ctrl.r1) {
            if (led_state == LED_OFF) led_state = LED_ON;
            else if (led_state == LED_ON) led_state = LED_AUTO;
            else if (led_state == LED_AUTO) led_state = LED_OFF;
        } else if (ctrl.l1 && !prev_ctrl.l1) {
            if (led_state == LED_OFF) led_state = LED_AUTO;
            else if (led_state == LED_AUTO) led_state = LED_ON;
            else if (led_state == LED_ON) led_state = LED_OFF;
        }
        decision.led = led_state;

        float steering_vel_spot = 0.0f;
        float steering_vel_pivot = 0.0f;
        float linear_vel = 0.0f;
        float vel_rev = 0.0f;

        if (ctrl.square && !prev_ctrl.square && !ctrl.square && !ctrl.triangle && !ctrl.cross) {
            steering_vel_spot = (float)input_x * 0.5f;
        } else if (ctrl.circle && !prev_ctrl.circle && !ctrl.square && !ctrl.triangle && !ctrl.cross) {
            steering_vel_pivot = (float)input_x * 0.5f;
        } else if (ctrl.circle && !prev_ctrl.circle && ctrl.cross && !prev_ctrl.cross && !ctrl.square && !ctrl.triangle) {
            vel_rev = -0.5f;
        } else if (ctrl.triangle && !prev_ctrl.triangle && !ctrl.circle && !ctrl.cross && !ctrl.square) {
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
        } else if (linear_vel == 0.0f && input_y > 0.0f) {
            decision.dir = DIR_STOP_EMERGENCY;
            vel = 0.0f;
        } else {
            decision.dir = DIR_INIT;
            vel = 0.0f;
        }
    }

    /* --- 7. GESTIONE DEGRADATA --- */
    if (global->b1_degradato || global->b2_degradato) {
        vel = (float)input_y * 0.3f;
    }

    /* --- 8. OUTPUT --- */
    prev_ctrl = ctrl;
    /* Convert m/s to mm/s for int16 transmission */
    decision.setpoint = (int16_t)(vel * 1000.0f);

    return decision;
}

Decision_t calculateDecision_B2(Partial_State_B2_Bus* partial_state) {
    /* --- GESTIONE STATO PERSISTENTE --- */
    static Controller_Bus prev_ctrl = {0};
    
    /* --- 2. INIZIALIZZAZIONE OUTPUT --- */
    Decision_t decision = {0};
    decision.setpoint = 0;
    decision.dir = DIR_INIT;
    decision.led = LED_OFF; 
    
    /* --- 3. CONTROLLO SICUREZZA LOCALE --- */
    /* Verifica solo l'emergenza di questa scheda */
    if (partial_state->emergenza) {
        decision.dir = DIR_STOP_EMERGENCY;
        decision.setpoint = 0;
        return decision; 
    }
    
    /* --- 4. LETTURA SENSORI (Mapping B2) --- */
    
    float dist_C = (float)partial_state->dC; 
    float dist_L = (float)partial_state->dL;   
    float dist_R = (float)partial_state->dR;
    
    /* --- A. Lettura Joystick --- */
    Controller_Bus ctrl = partial_state->controller; 
    
    double input_y = (double)ctrl.ry / 512.0; 
    if (ctrl.ry < 0) input_y = 0.0; 
    
    double val_r2 = (double)ctrl.r2 / 1020.0;
    double val_l2 = (double)ctrl.l2 / 1020.0;
    double input_x = val_r2 - val_l2; 
    
    if (fabs(input_y) < DEADZONE) input_y = 0.0;
    if (fabs(input_x) < DEADZONE) input_x = 0.0;
    
    uint8_t avoiding_obstacle = 0;
    float vel = 0.0f;
 
    /* =========================================================
       LOGICA OSTACOLI
       ========================================================= */
    
    /* 1. CONTROLLO DEGRADATO (Solo Locale B2) */
    /* Se B2 è degradata, estende la zona di sicurezza a 4m */
    if ((dist_C < OBSTACLE_THRESH_STOP_EM) ||
        (dist_L < OBSTACLE_THRESH_STOP_EM) ||
        (dist_R < OBSTACLE_THRESH_STOP_EM)) {
        
        avoiding_obstacle = 1;
        decision.dir = DIR_STOP_EMERGENCY;
        vel = 0.0f;
    }

    /* --- 6. GUIDA NORMALE --- */
    if (!avoiding_obstacle) {
              
        float steering_vel_spot = 0.0f;
        float steering_vel_pivot = 0.0f;
        float linear_vel = 0.0f;
        float vel_rev = 0.0f;
        
        if (ctrl.square && !prev_ctrl.square && !ctrl.circle && !ctrl.triangle && !ctrl.cross) {
            steering_vel_spot = (float)input_x * 0.5f;
        } else if (ctrl.circle && !prev_ctrl.circle && !ctrl.square && !ctrl.triangle && !ctrl.cross) {
            steering_vel_pivot = (float)input_x * 0.5f;
        } else if (ctrl.circle && !prev_ctrl.circle && ctrl.square && !prev_ctrl.cross && !ctrl.square && !ctrl.triangle) {
            vel_rev = -0.5f;
        } else if (ctrl.triangle && !prev_ctrl.triangle && !ctrl.circle && !ctrl.cross && !ctrl.square) {
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
        } else if (linear_vel == 0.0f && input_y > 0.0f) {
            decision.dir = DIR_STOP_EMERGENCY;
            vel = 0.0f;
        } else {
            decision.dir = DIR_INIT;
            vel = 0.0f;
        }
    }
    
    /* --- 8. OUTPUT --- */
    prev_ctrl = ctrl;
    /* Scaling by 0.5 as per MATLAB request and converting to mm/s for int16 */
    decision.setpoint = (int16_t)(vel * 0.5f * 1000.0f);
    
    return decision;
}

uint8_t ReadUart_Decision(uint8_t* rx_buffer, uint16_t rx_len, Decision_t* out_decision, int16_t* out_seq_num) {
    const uint8_t packet_len = DECISION_PROTOCOL_SIZE;
    const uint8_t payload_len = 4U;
    const uint8_t SOF = 0xAAU;
    const uint8_t EOF_BYTE = 0x55U;
    const uint8_t CMD_DECISION = 0x30U;

    if (rx_len < packet_len) return 0;

    /* Synchronization */
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
    
    /* Indices based on MATLAB spec:
       Byte 0: SOF
       Byte 1: Seq
       Byte 2: Cmd
       Byte 3: Len
       Byte 4: Dir
       Byte 5: SpL
       Byte 6: SpH
       Byte 7: Led
       Byte 8: CRC LO
       Byte 9: CRC HI
       Byte 10: EOF
    */
    uint8_t r_crc_lo = full_packet[8];
    uint8_t r_crc_hi = full_packet[9];
    uint8_t r_eof = full_packet[10];
    uint16_t r_crc_val = (uint16_t)r_crc_lo | ((uint16_t)r_crc_hi << 8);

    if (r_eof != EOF_BYTE || r_cmd != CMD_DECISION || r_len != payload_len) return 0;
    
    /* CRC calculated on Header (excluding SOF) + Payload. 
       Bytes 1 to 7 (Seq, Cmd, Len, Dir, SpL, SpH, Led) -> Length 7 */
    if (calculate_crc16_local(&full_packet[1], 7) != r_crc_val) return 0;

    *out_seq_num = (int16_t)r_seq;
    out_decision->dir = full_packet[4];
    
    uint16_t sp = (uint16_t)full_packet[5] | ((uint16_t)full_packet[6] << 8);
    out_decision->setpoint = (int16_t)sp;
    out_decision->led = full_packet[7];

    return 1;
}

void serializeDecisionB2(Decision_t* decision, uint32_t sequence, uint8_t* buffer) {
    const uint8_t TOTAL_PACKET_SIZE = DECISION_BUFFER_SIZE;
    const uint8_t SOF = 0xAAU;
    const uint8_t EOF_BYTE = 0x55U;
    const uint8_t CMD_DECISION = 0x30U;
    const uint8_t PAYLOAD_LEN = 4U;

    /* Initialize buffer with zeros (Padding) */
    memset(buffer, 0, TOTAL_PACKET_SIZE);

    buffer[0] = SOF;
    buffer[1] = (uint8_t)(sequence & 0xFF);
    buffer[2] = CMD_DECISION;
    buffer[3] = PAYLOAD_LEN;
    
    buffer[4] = decision->dir;
    buffer[5] = (uint8_t)((uint16_t)decision->setpoint & 0xFFU);
    buffer[6] = (uint8_t)(((uint16_t)decision->setpoint >> 8) & 0xFFU);
    buffer[7] = decision->led;
    
    uint16_t crc = calculate_crc16_local(&buffer[1], 7);
    buffer[8] = (uint8_t)(crc & 0xFFU);
    buffer[9] = (uint8_t)((crc >> 8) & 0xFFU);
    
    buffer[10] = EOF_BYTE;
}

uint8_t CheckDiscordance(Decision_t d1, Decision_t d2) {
    if (d1.dir != d2.dir) return 1;
    if (d1.setpoint != d2.setpoint) return 1;
    if (d1.led != d2.led) return 1;
    return 0;
}
