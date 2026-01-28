#include "helper_communication_b2.h"
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

uint8_t ReadUart_packet_B1_Local(uint8_t *rx_buffer, uint16_t rx_len, Partial_State_B1_Bus *out_bus, int16_t *out_seq_num) {
	const uint8_t packet_len = 32;
	const uint8_t payload_len = 27;
	const uint8_t SOF = 0xAA;
	const uint8_t EOF_BYTE = 0x55;
	const uint8_t CMD_ID = 0x20;

	if (rx_len < packet_len)
		return 0;

	int32_t found_index = -1;
	int32_t limit = (int32_t) rx_len - (int32_t) packet_len + 1;

	for (int32_t i = 0; i < limit; i++) {
		if (rx_buffer[i] == SOF) {
			found_index = i;
			break;
		}
	}

	if (found_index == -1)
		return 0;

	uint8_t *full_packet = &rx_buffer[found_index];

	if ((full_packet[31] != EOF_BYTE) || (full_packet[2] != CMD_ID)
			|| (full_packet[3] != payload_len)) {
		return 0;
	}

	*out_seq_num = (int16_t) full_packet[1];

	float v_fl, v_fr, v_rl, v_rr;
	memcpy(&v_fl, &full_packet[4], 4);
	memcpy(&v_fr, &full_packet[8], 4);
	memcpy(&v_rl, &full_packet[12], 4);
	memcpy(&v_rr, &full_packet[16], 4);

	out_bus->b1_vel_fl = v_fl;
	out_bus->b1_vel_fr = v_fr;
	out_bus->b1_vel_rl = v_rl;
	out_bus->b1_vel_rr = v_rr;

	out_bus->battery_level = full_packet[20];
	out_bus->temperatura = full_packet[21];

	// uint32_t photo; memcpy(&photo, &full_packet[22], 4);
	// out_bus->photoresistor = photo;

	out_bus->emergency = full_packet[29];
	out_bus->degraded = full_packet[30];

	return 1;
}

void serializePartialStateB2_Local(Partial_State_B2_Bus *state, uint8_t seq_num, uint8_t *buffer) {
	const uint8_t SOF = 0xAA;
	const uint8_t EOF_BYTE = 0x55;
	const uint8_t CMD_B2_STATE = 0x21;
	const uint8_t PAYLOAD_LEN = 25;

	uint8_t idx = 0;

	// Header
	buffer[idx++] = SOF;
	buffer[idx++] = seq_num;
	buffer[idx++] = CMD_B2_STATE;
	buffer[idx++] = PAYLOAD_LEN;

	// Controller Analog
	memcpy(&buffer[idx], &state->controller.ry, 2); idx += 2;
	memcpy(&buffer[idx], &state->controller.l2, 2); idx += 2;
	memcpy(&buffer[idx], &state->controller.r2, 2); idx += 2;

	// Controller Buttons & Battery
	buffer[idx++] = state->controller.cross;
	buffer[idx++] = state->controller.circle;
	buffer[idx++] = state->controller.square;
	buffer[idx++] = state->controller.triangle;
	buffer[idx++] = state->controller.l1;
	buffer[idx++] = state->controller.r1;
	buffer[idx++] = state->controller.battery;

	// Sensori
	memcpy(&buffer[idx], &state->dL, 2); idx += 2;
	memcpy(&buffer[idx], &state->dC, 2); idx += 2;
	memcpy(&buffer[idx], &state->dR, 2); idx += 2;

	buffer[idx++] = state->angolo;

	// Status Flags
	buffer[idx++] = state->controller_status;
	buffer[idx++] = state->sonar_status;
	buffer[idx++] = state->mpu_status;
	buffer[idx++] = state->degradato;
	buffer[idx++] = state->emergenza;

	// EOF
	buffer[idx++] = EOF_BYTE;
}

Global_State_t reconstructGlobalState(Partial_State_B1_Bus state_b1,
		Partial_State_B2_Bus state_b2) {
	Global_State_t state = { 0 };

	// --- 1. CALCOLO LOGICA DI SISTEMA ---
	// Emergenza Sistema: Attiva se B1 O B2 sono in emergenza
	uint8_t is_sys_emergency = (state_b1.emergency > 0)
			|| (state_b2.emergenza > 0);

	// --- 2. COSTRUZIONE STRUTTURA GLOBALE ---
	// A. Assegnazione Bus Annidati (Copia diretta)
	state.b1 = state_b1;
	state.b2 = state_b2;

	// B. Assegnazione Flag di Sistema
	state.system_emergenza = is_sys_emergency;
	state.comunication_degradato = 0;

	return state;
}

Decision_t calculateDecision(Global_State_t *glob_state) {
	// --- STATO PERSISTENTE ---
	static Controller_Bus prev_ctrl = { 0 };
	static uint8_t led_state = LED_OFF;
	static float timer_memory_L = 0.0f;
	static float timer_memory_R = 0.0f;

	Controller_Bus current_ctrl = glob_state->b2.controller;

	// --- 2. INIZIALIZZAZIONE OUTPUT ---
	Decision_t decision = { 0 };
	decision.setpoint = 0;
	decision.dir = DIR_INIT;
	decision.led = led_state;

	uint8_t avoiding_obstacle = 0;

	// --- 3. CONTROLLO SICUREZZA ---
	if (glob_state->system_emergenza == 1) {
		decision.dir = DIR_STOP_EMERGENCY;
		decision.led = LED_EMERGENCY;
		decision.setpoint = 0;
		prev_ctrl = current_ctrl;
		return decision;
	}

	// --- Lettura Sensori e Joystick ---
	float dist_C = (float) glob_state->b2.dC;
	float dist_L = (float) glob_state->b2.dL;
	float dist_R = (float) glob_state->b2.dR;

	float input_y = (float) current_ctrl.ry / 512.0f;
	if (current_ctrl.ry < 0)
		input_y = 0.0f;

	float val_r2 = (float) current_ctrl.r2 / 1020.0f;
	float val_l2 = (float) current_ctrl.l2 / 1020.0f;
	float input_x = val_r2 - val_l2;

	if (fabsf(input_y) < DEADZONE)
		input_y = 0.0f;
	if (fabsf(input_x) < DEADZONE)
		input_x = 0.0f;

	float vel = 0.0f;

	// --- AGGIORNAMENTO TIMER ---
	if (timer_memory_L > 0.0f)
		timer_memory_L -= DT;
	if (timer_memory_R > 0.0f)
		timer_memory_R -= DT;

	// --- LOGICA LED ---
	if (current_ctrl.r1 && !prev_ctrl.r1) {
		if (led_state == LED_OFF) led_state = LED_ON;
		else if (led_state == LED_ON) led_state = LED_AUTO;
		else if (led_state == LED_AUTO) led_state = LED_OFF;
	} else if (current_ctrl.l1 && !prev_ctrl.l1) {
		if (led_state == LED_OFF) led_state = LED_AUTO;
		else if (led_state == LED_AUTO) led_state = LED_ON;
		else if (led_state == LED_ON) led_state = LED_OFF;
	}
	decision.led = led_state;

	// --- LOGICA OSTACOLI PRIORITARIA (DEGRADATO) ---
	uint8_t is_degraded = (glob_state->b1.degraded == 1) || (glob_state->b2.degradato == 1);

	if (is_degraded) {
		if (dist_C < 400.0f || dist_L < 400.0f || dist_R < 400.0f) {
			decision.dir = DIR_STOP_EMERGENCY;
			decision.setpoint = 0;
			prev_ctrl = current_ctrl;
			return decision;
		}
	}

	// --- LOGICA OSTACOLI NORMALE ---
	if (input_y >= 0.0f) {
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

	// --- GUIDA MANUALE ---
	if (!avoiding_obstacle) {
		float steering_vel_spot = 0.0f;
		float steering_vel_pivot = 0.0f;
		float linear_vel = 0.0f;
		float vel_rev = 0.0f;

		if (current_ctrl.square && !prev_ctrl.square && !current_ctrl.circle && !current_ctrl.triangle && !current_ctrl.cross) {
			steering_vel_spot = input_x * 0.5f;
		} else if (current_ctrl.circle && !prev_ctrl.circle && !current_ctrl.square && !current_ctrl.triangle && !current_ctrl.cross) {
			steering_vel_pivot = input_x * 0.5f;
		} else if (current_ctrl.circle && !prev_ctrl.circle && current_ctrl.cross && !prev_ctrl.cross && !current_ctrl.square && !current_ctrl.triangle) {
			vel_rev = -0.5f;
		} else if (current_ctrl.triangle && !prev_ctrl.triangle && !current_ctrl.circle && !current_ctrl.cross && !current_ctrl.square) {
			linear_vel = 0.0f;
		} else {
			linear_vel = input_y * MAX_SPEED;
		}

		if (fabsf(steering_vel_spot) > 0.1f) {
			decision.dir = (steering_vel_spot > 0) ? DIR_RIGHT_SPOT : DIR_LEFT_SPOT;
			vel = fabsf(steering_vel_spot);
		} else if (fabsf(steering_vel_pivot) > 0.1f) {
			decision.dir = (steering_vel_pivot > 0) ? DIR_RIGHT_PIVOT : DIR_LEFT_PIVOT;
			vel = fabsf(steering_vel_pivot);
		} else if (fabsf(linear_vel) > 0.1f) {
			decision.dir = DIR_FORWARD;
			vel = fabsf(linear_vel);
		} else if (vel_rev == -0.5f) {
			decision.dir = DIR_REV;
			vel = fabsf(vel_rev);
		} else if (linear_vel == 0.0f && input_y > 0.0f) {
			decision.dir = DIR_STOP_EMERGENCY;
			vel = 0.0f;
		} else {
			decision.dir = DIR_INIT;
			vel = 0.0f;
		}
	}

	// --- APPLICAZIONE DEGRADATA (Finale) ---
	if (is_degraded) {
		decision.setpoint = (int16_t) (vel * 0.5f * 1000.0f);
	} else {
		decision.setpoint = (int16_t) (vel * 1000.0f);
	}

	prev_ctrl = current_ctrl;
	return decision;
}

Decision_t calculateDecision_B2(Partial_State_B2_Bus *partial_state) {
	static Controller_Bus prev_ctrl = { 0 };

	Decision_t decision = { 0 };
	decision.setpoint = 0;
	decision.dir = DIR_INIT;
	decision.led = LED_OFF;

	// --- 3. CONTROLLO SICUREZZA LOCALE ---
	if (partial_state->emergenza > 0) {
		decision.dir = DIR_STOP_EMERGENCY;
		decision.setpoint = 0;
		decision.led = LED_EMERGENCY;
		prev_ctrl = partial_state->controller;
		return decision;
	}

	// --- 4. PARAMETRI E SENSORI ---
	float OBSTACLE_THRESH_STOP_LOCAL = 400.0f;

	float dist_C = (float) partial_state->dC;
	float dist_L = (float) partial_state->dL;
	float dist_R = (float) partial_state->dR;

	Controller_Bus ctrl = partial_state->controller;

	double input_y = (double) ctrl.ry / 512.0;
	if (ctrl.ry < 0)
		input_y = 0.0;

	double val_r2 = (double) ctrl.r2 / 1020.0;
	double val_l2 = (double) ctrl.l2 / 1020.0;
	double input_x = val_r2 - val_l2;

	if (fabs(input_y) < DEADZONE)
		input_y = 0.0;
	if (fabs(input_x) < DEADZONE)
		input_x = 0.0;

	uint8_t avoiding_obstacle = 0;
	float vel = 0.0f;

	// LOGICA OSTACOLI
	if ((dist_C < OBSTACLE_THRESH_STOP_LOCAL)
			|| (dist_L < OBSTACLE_THRESH_STOP_LOCAL)
			|| (dist_R < OBSTACLE_THRESH_STOP_LOCAL)) {

		avoiding_obstacle = 1;
		decision.dir = DIR_STOP_EMERGENCY;
		vel = 0.0f;
	}

	// GUIDA NORMALE
	if (!avoiding_obstacle) {
		float steering_vel_spot = 0.0f;
		float steering_vel_pivot = 0.0f;
		float linear_vel = 0.0f;
		float vel_rev = 0.0f;

		if (ctrl.square && !prev_ctrl.square && !ctrl.circle && !ctrl.triangle && !ctrl.cross) {
			steering_vel_spot = (float) input_x * 0.5f;
		} else if (ctrl.circle && !prev_ctrl.circle && !ctrl.square && !ctrl.triangle && !ctrl.cross) {
			steering_vel_pivot = (float) input_x * 0.5f;
		} else if (ctrl.circle && !prev_ctrl.circle && ctrl.cross && !prev_ctrl.cross && !ctrl.square && !ctrl.triangle) {
			vel_rev = -0.5f;
		} else if (ctrl.triangle && !prev_ctrl.triangle && !ctrl.circle && !ctrl.cross && !ctrl.square) {
			linear_vel = 0.0f;
		} else {
			linear_vel = (float) input_y * MAX_SPEED;
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

	prev_ctrl = ctrl;
	decision.setpoint = (int16_t) (vel * 0.5f * 1000.0f);

	return decision;
}

uint8_t ReadUart_Decision_Local(uint8_t *rx_buffer, uint16_t rx_len,
		Decision_t *out_decision, int16_t *out_seq_num) {
	const uint8_t packet_len = 11;
	const uint8_t payload_len = 6;
	const uint8_t SOF = 0xAA;
	const uint8_t EOF_BYTE = 0x55;
	const uint8_t CMD_DECISION = 0x30;

	if (rx_len < packet_len)
		return 0;

	int32_t found_index = -1;
	int32_t limit = (int32_t) rx_len - (int32_t) packet_len + 1;

	for (int32_t i = 0; i < limit; i++) {
		if (rx_buffer[i] == SOF) {
			found_index = i;
			break;
		}
	}

	if (found_index == -1)
		return 0;

	uint8_t *full_packet = &rx_buffer[found_index];

	if ((full_packet[10] != EOF_BYTE) || (full_packet[2] != CMD_DECISION)
			|| (full_packet[3] != payload_len)) {
		return 0;
	}

	*out_seq_num = (int16_t) full_packet[1];

	uint16_t lo = (uint16_t) full_packet[4];
	uint16_t hi = (uint16_t) full_packet[5];
	int16_t val_i16 = (int16_t) (lo | (hi << 8));

	out_decision->setpoint = val_i16;
	out_decision->dir = full_packet[6];
	out_decision->led = full_packet[7];
	out_decision->decision_board = full_packet[8];
	out_decision->system_mode = full_packet[9];

	return 1;
}

void serializeDecisionB2_Local(Decision_t *decision, uint8_t sequence_num, uint8_t *buffer) {
	const uint8_t TOTAL_PACKET_SIZE = 15;
	const uint8_t SOF = 0xAA;
	const uint8_t EOF_BYTE = 0x55;
	const uint8_t CMD_DECISION = 0x30;
	const uint8_t PAYLOAD_LEN = 6;

	// Initialize buffer with zeros (Padding)
	memset(buffer, 0, TOTAL_PACKET_SIZE);

	uint8_t idx = 0;

	// Header
	buffer[idx++] = SOF;
	buffer[idx++] = sequence_num;
	buffer[idx++] = CMD_DECISION;
	buffer[idx++] = PAYLOAD_LEN;

	// Payload
	memcpy(&buffer[idx], &decision->setpoint, 2);
	idx += 2;

	buffer[idx++] = decision->dir;
	buffer[idx++] = decision->led;
	buffer[idx++] = decision->decision_board;
	buffer[idx++] = decision->system_mode;

	// Footer
	buffer[idx++] = EOF_BYTE;
}

 uint8_t CheckDiscordance_Local(Decision_t decision_A, Decision_t decision_B) {
	if (decision_A.dir != decision_B.dir || decision_A.setpoint != decision_B.setpoint
			|| decision_A.led != decision_B.led
			|| decision_A.decision_board != decision_B.decision_board
			|| decision_A.system_mode != decision_B.system_mode) {
		return 1;
	}
	return 0;
}
