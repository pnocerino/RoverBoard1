//#include "FreeRTOS.h"
//#include "task.h"
#include "main.h"
//#include "cmsis_os.h"
#include <math.h>
#include <string.h>
#include "communication.h"
#include "usart.h"
//#include "reading.h"
#include "helper_communication_b2.h"
#include "gpio.h"
#include "string.h"
#include "stdio.h"

/* Mutex Handle */
//extern osMutexId commDataMutexHandle;
//extern osMutexId partialStateMutexHandle;

char tx_msg[512];

/* Current State */
static CommState_t currentState = COMM_STATE_START;
static Global_State_t global_state = { 0 };

/* Packet Logic Variables */
static uint8_t success_send = 0;
static uint8_t success_read = 0;
static uint8_t loss_packet_1_counter = 0;
static uint8_t loss_send_counter = 0;
static uint8_t is_discordant = 0;
static int16_t seq_num = 0;
static uint8_t rx_partial_buffer[PACKET_B1_SIZE];
static uint8_t buffer_b2[PACKET_B2_SIZE];
static uint8_t rx_decision_buffer[DECISION_PACKET_SIZE];
static uint8_t buffer_decision[DECISION_PACKET_SIZE];
static Partial_State_B1_Bus partial_state_b1 = { 0 };
static Partial_State_B2_Bus partial_state_b2 = { 0 };

static Decision_t decision_b1;
static Decision_t decision_b2;  

void Communication_GetActuationData(Global_State_t *g, Decision_t *d,uint8_t *disc) {
		*g = global_state;
		*d = decision_b2;
		*disc = is_discordant;
}

void Communication_Init(void) {
	global_state.comunication_degradato = 0;
	global_state.system_emergenza = 0;

	Controller_Bus c = {.battery = 155, .circle = 0, .square = 0, .cross = 0, .triangle = 0, .ry = 345, .l1 = 0, .l2 = 0, .r1 = 0, .r2 = 0 };

	partial_state_b2.angolo = 82;
	partial_state_b2.controller = c;
	partial_state_b2.controller_status = 1;
	partial_state_b2.dC = 400; partial_state_b2.dL = 400; partial_state_b2.dR = 400;
	partial_state_b2.degradato = 0; partial_state_b2.emergenza = 0;
	partial_state_b2.mpu_status = 1;
	partial_state_b2.sonar_status = 1;

	HAL_GPIO_WritePin(BOARD_ALIVE_B2_GPIO_Port, BOARD_ALIVE_B2_Pin, GPIO_PIN_SET);
	currentState = COMM_STATE_START;
}

void Communication_Step(void) {
	uint8_t processing_complete = 0;

	do {
		switch (currentState) {
		case COMM_STATE_START:
			success_send = 0;
			success_read = 0;

			if (HAL_GPIO_ReadPin(BOARD_ALIVE_B1_GPIO_Port, BOARD_ALIVE_B1_Pin) == GPIO_PIN_RESET) {
                currentState = COMM_STATE_SAFE_MODE;
            } else {
				uint32_t tick_start = HAL_GetTick();
				while ((HAL_GetTick() - tick_start) < TIMEOUT_MS) {
					if(HAL_GPIO_ReadPin(RTS_B1_GPIO_Port, RTS_B1_Pin) == GPIO_PIN_SET) {
						currentState = COMM_STATE_RECEIVE_PACKET_B2;
						break;
					}
				}
			} 
			
			int len = snprintf(tx_msg, sizeof(tx_msg),"SONO VAIRO\n\r");
			HAL_UART_Transmit(&hlpuart1, (uint8_t*)tx_msg, len, 1);
			currentState = COMM_STATE_SAFE_MODE;
			break;

		case COMM_STATE_RECEIVE_PACKET_B2:
			// Entry Actions
			HAL_GPIO_WritePin(CTS_B2_GPIO_Port, CTS_B2_Pin, GPIO_PIN_SET);

			// Transition
			currentState = COMM_STATE_READ_PACKET;
			break;

		case COMM_STATE_READ_PACKET:
			do {
				if (HAL_UART_Receive(&huart1, rx_partial_buffer, PACKET_B1_SIZE, UART_TIMEOUT_PARTIAL_MS) == HAL_OK) {
					int16_t temp_seq;
					if (ReadUart_packet_B1_Local(rx_partial_buffer, PACKET_B1_SIZE, &partial_state_b1, &temp_seq)) {
						success_read = 1;
						seq_num = temp_seq;
					} else {
						success_read = 0;
						loss_packet_1_counter++;
					}
				} else {
					success_read = 0;
					loss_packet_1_counter++;
				}
			} while ((success_read == 0) && (loss_packet_1_counter <= 1));
			
			if (success_read == 0) {
				currentState = COMM_STATE_IMPOSSIBLE_RECEIVE_FROM_B1;
			} else {
				seq_num++;
				currentState = COMM_STATE_PACKET_RECEIVED;
			}
			break;

		case COMM_STATE_IMPOSSIBLE_RECEIVE_FROM_B1:
			HAL_GPIO_WritePin(CTS_B2_GPIO_Port, CTS_B2_Pin, GPIO_PIN_RESET);
			loss_packet_1_counter = 0;

			currentState = COMM_STATE_SAFE_MODE;
			break;

		case COMM_STATE_PACKET_RECEIVED:
			loss_packet_1_counter = 0;
			success_read = 0;
			HAL_GPIO_WritePin(CTS_B2_GPIO_Port, CTS_B2_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(RTS_B2_GPIO_Port, RTS_B2_Pin, GPIO_PIN_SET);
			currentState = COMM_STATE_SERIALIZE_PACKET_B2;
			break;

		case COMM_STATE_SERIALIZE_PACKET_B2:
			//if(osMutexAcquire(partialStateMutexHandle, MUTEX_TIMEOUT_MS) == osOK){
				serializePartialStateB2_Local(&partial_state_b2, (uint8_t) seq_num,
					buffer_b2);
				//osMutexRelease(partialStateMutexHandle);
			//}

			uint32_t tick_start = HAL_GetTick();
			while ((HAL_GetTick() - tick_start) < TIMEOUT_MS) {
				if (HAL_GPIO_ReadPin(CTS_B1_GPIO_Port, CTS_B1_Pin) == GPIO_PIN_SET) {
					currentState = COMM_STATE_SEND_PACKET_B2;
					break;
				}
			}
				currentState = COMM_STATE_SAFE_MODE;
			
			
			break;

		case COMM_STATE_SEND_PACKET_B2:

			do{
				if (HAL_UART_Transmit(&huart1, buffer_b2, PACKET_B2_SIZE, UART_TIMEOUT_PARTIAL_MS) == HAL_OK) {
					success_send = 1;
				} else {
					success_send = 0;
					loss_send_counter++;
				}
			}while(success_send == 0 && loss_send_counter <=1);
			

			if (success_send == 0) {
				currentState = COMM_STATE_IMPOSSIBLE_SEND_FROM_B2;
			} else {
				seq_num++;
				currentState = COMM_STATE_PACKET_SENT;
			}
			break;

		case COMM_STATE_PACKET_SENT:
			loss_send_counter = 0;
			success_send = 0;
			HAL_GPIO_WritePin(RTS_B2_GPIO_Port, RTS_B2_Pin, GPIO_PIN_RESET);
			tick_start = HAL_GetTick();
			while ((HAL_GetTick() - tick_start) < TIMEOUT_MS) {
				if (HAL_GPIO_ReadPin(CTS_B1_GPIO_Port, CTS_B1_Pin) == GPIO_PIN_RESET){
					currentState = COMM_STATE_GLOBAL_STATE_RECONSTRUCTION;
					break;
				}
			} 
			
			currentState = COMM_STATE_SAFE_MODE;
			
			break;

		case COMM_STATE_IMPOSSIBLE_SEND_FROM_B2:
			loss_send_counter = 0;
			HAL_GPIO_WritePin(RTS_B2_GPIO_Port, RTS_B2_Pin, GPIO_PIN_RESET);
			currentState = COMM_STATE_SAFE_MODE;
			break;

		case COMM_STATE_GLOBAL_STATE_RECONSTRUCTION:
		//if (osMutexAcquire(commDataMutexHandle, MUTEX_TIMEOUT_MS) == osOK){
			global_state = reconstructGlobalState(partial_state_b1, partial_state_b2);
			//osMutexRelease(commDataMutexHandle);
		//}
			currentState = COMM_STATE_TAKE_DECISION_B2;
			break;

		case COMM_STATE_TAKE_DECISION_B2:
			//if(osMutexAcquire(commDataMutexHandle, MUTEX_TIMEOUT_MS) == osOK){
				decision_b2 = calculateDecision(&global_state);
				//osMutexRelease(commDataMutexHandle);
			//}
			tick_start = HAL_GetTick();
			while ((HAL_GetTick() - tick_start) < TIMEOUT_MS) {
				if (HAL_GPIO_ReadPin(RTS_B1_GPIO_Port, RTS_B1_Pin) == GPIO_PIN_SET){
					currentState = COMM_STATE_READ_DECISION_B1;
				}
			}

			currentState = COMM_STATE_SAFE_MODE;
			break;

		case COMM_STATE_READ_DECISION_B1:
			HAL_GPIO_WritePin(CTS_B2_GPIO_Port, CTS_B2_Pin, GPIO_PIN_SET);
			do{
				if(HAL_UART_Receive(&huart1, rx_decision_buffer, DECISION_PACKET_SIZE, UART_TIMEOUT_DECISION_MS) == HAL_OK) {
					int16_t temp_seq;
					if (ReadUart_Decision_Local(rx_decision_buffer, DECISION_PACKET_SIZE, &decision_b1, &temp_seq)) {
						success_read = 1;
						seq_num = temp_seq;
					} else {
						success_read = 0;
						loss_packet_1_counter++;
					}
				} else {
					success_read = 0;
					loss_packet_1_counter++;
				}

			}while((success_read == 0) && (loss_packet_1_counter <= 1));

			if (success_read == 0) {
				currentState = COMM_STATE_IMPOSSIBLE_RECEIVE_FROM_B1;
			} else {
				seq_num++;
				currentState = COMM_STATE_DECISION_RECEIVED;
			}
			break;

		case COMM_STATE_DECISION_RECEIVED:
			HAL_GPIO_WritePin(CTS_B2_GPIO_Port, CTS_B2_Pin, GPIO_PIN_RESET);
			success_read = 0;
			loss_packet_1_counter = 0;
			currentState = COMM_STATE_SERIALIZE_DECISION_B2;
			break;

		case COMM_STATE_SERIALIZE_DECISION_B2:
			HAL_GPIO_WritePin(RTS_B2_GPIO_Port, RTS_B2_Pin, GPIO_PIN_SET);
			serializeDecisionB2_Local(&decision_b2, (uint8_t) seq_num,
					buffer_decision);
			tick_start = HAL_GetTick();
			while ((HAL_GetTick() - tick_start) < TIMEOUT_MS) {
				if (HAL_GPIO_ReadPin(CTS_B1_GPIO_Port, CTS_B1_Pin) == GPIO_PIN_SET) {
					currentState = COMM_STATE_SEND_DECISION_B2;
					break;
				}
			}
			currentState = COMM_STATE_SAFE_MODE;
			break;

		case COMM_STATE_SEND_DECISION_B2:
			
			do{
				if (HAL_UART_Transmit(&huart1, buffer_decision, DECISION_PACKET_SIZE, UART_TIMEOUT_DECISION_MS) == HAL_OK) {
					success_send = 1;
				} else {
					success_send = 0;
					loss_send_counter++;
				}
			}while((success_send == 0) && (loss_send_counter <= 1));

			if (success_send == 0) {
				currentState = COMM_STATE_IMPOSSIBLE_SEND_FROM_B2;
			} else {
				seq_num++;
				currentState = COMM_STATE_DECISION_SENT;
			}
			break;

		case COMM_STATE_DECISION_SENT:
			HAL_GPIO_WritePin(RTS_B2_GPIO_Port, RTS_B2_Pin, GPIO_PIN_RESET);
			loss_send_counter = 0;
			success_send = 0;
			tick_start = HAL_GetTick();
			while ((HAL_GetTick() - tick_start) < TIMEOUT_MS) {
				if (HAL_GPIO_ReadPin(CTS_B1_GPIO_Port, CTS_B1_Pin) == GPIO_PIN_RESET) {
					currentState = COMM_STATE_ACTUATE_DECISION;
					break;
				}
			}
			currentState = COMM_STATE_SAFE_MODE;
			break;

		case COMM_STATE_ACTUATE_DECISION:
			//if(osMutexAcquire(commDataMutexHandle, MUTEX_TIMEOUT_MS) == osOK) {
				is_discordant = CheckDiscordance_Local(decision_b1, decision_b2);
				//osMutexRelease(commDataMutexHandle);
			//}
			currentState = COMM_STATE_START;

			len = snprintf(tx_msg, sizeof(tx_msg),
			    "B1: V[%.2f, %.2f, %.2f, %.2f] Bat:%u Temp:%u Photo:%lu Enc:%u B_Stat:%u T_Stat:%u Em:%u Deg:%u | "
			    "B2: dL:%u dC:%u dR:%u Ang:%u C_Stat:%u S_Stat:%u M_Stat:%u Deg:%u Em:%u | "
			    "G: Em:%u Com:%u\r\n",

			    global_state.b1.b1_vel_fl,
			    global_state.b1.b1_vel_fr,
			    global_state.b1.b1_vel_rl,
			    global_state.b1.b1_vel_rr,
			    global_state.b1.battery_level,
			    global_state.b1.temperatura,
			    global_state.b1.photoresistor,
			    global_state.b1.encoder_status,
			    global_state.b1.battery_status,
			    global_state.b1.temp_status,
			    global_state.b1.emergency,
			    global_state.b1.degraded,

			    global_state.b2.dL,
			    global_state.b2.dC,
			    global_state.b2.dR,
			    global_state.b2.angolo,
			    global_state.b2.controller_status,
			    global_state.b2.sonar_status,
			    global_state.b2.mpu_status,
			    global_state.b2.degradato,
			    global_state.b2.emergenza,

			    global_state.system_emergenza,
			    global_state.comunication_degradato
			);

			HAL_UART_Transmit(&hlpuart1, (uint8_t*) tx_msg, len, 100);
			processing_complete = 1;
			break;

		case COMM_STATE_SAFE_MODE:
			//if (osMutexAcquire(commDataMutexHandle, MUTEX_TIMEOUT_MS) == osOK) {
				global_state = reconstructGlobalState(partial_state_b1, partial_state_b2);
				global_state.comunication_degradato = 1;
				decision_b2 = calculateDecision_B2(&partial_state_b2);

				//osMutexRelease(commDataMutexHandle);
			//}
				 len = snprintf(tx_msg, sizeof(tx_msg),"SONO BASILE\n\r");
							HAL_UART_Transmit(&hlpuart1, (uint8_t*)tx_msg, len, 1);
			currentState = COMM_STATE_START;
			processing_complete = 1;
			break;

		default:
			currentState = COMM_STATE_START;
			processing_complete = 1;
			break;
		}
	} while (!processing_complete);
}
