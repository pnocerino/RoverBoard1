#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#include <math.h>
#include <string.h>
#include <communication.h>
#include "usart.h"
#include "reading.h"
#include "helper_communication_b1.h"

/* -------------------------------------------------------------------------- */
/* TIPI E DEFINIZIONI                          */
/* -------------------------------------------------------------------------- */

#define UART_TIMEOUT_MS         (10U)
#define HANDSHAKE_TIMEOUT_MS    (100U)
#define HANDSHAKE_RX_VAL        (0xAAU)
#define HANDSHAKE_TX_VAL        (0xBBU)
#define MAX_LOSS_COUNT          (5U)
#define BUFFER_SIZE             (64U)
#define ALIVE_CHECK_MS          (300U)
#define PULSE_WIDTH_MS          (5U)

TaskHandle_t xCommTaskHandle = NULL;

/* Mutex Handle */
extern osMutexId commDataMutexHandle;
extern osMutexId partialStateMutexHandle;

/* Current State */

static CommState_t currentState = COMM_STATE_RECEIVE_RTS_B2;

/* Logic Support Variables */
static uint8_t connected_B2 = 0;
static uint8_t connected_B1 = 0; /* External logic/heartbeat update required */
static uint8_t board_1_alive = 1; /* External logic update required */
static uint8_t handshake_rx_success_B2 = 0;
static uint8_t handshake_tx_success_B2 = 0;
static Global_State_t global_state = { 0 };
static uint32_t timer_start = 0;

/* Packet Logic Variables */
static uint8_t success_send = 0;
static uint8_t success_read = 0;
static uint8_t loss_packet_1_counter = 0;
static uint8_t is_discordant = 0;
static uint32_t seq_num = 0;
static uint8_t packet_rx_buffer[BUFFER_SIZE];
static uint8_t buffer_b2[BUFFER_SIZE];
static uint8_t rx_buffer_decision[BUFFER_SIZE];
static uint8_t buffer_decision[BUFFER_SIZE];
//Partial_State_B1_Bus partial_state_b1;
static Partial_State_B2_Bus partial_state_b2;
static Decision_t decision_b1;
static Decision_t decision_b2;
static int16_t received_seq_num = 0;
static uint8_t loss_send_counter = 0;

void Communication_GetActuationData(Global_State_t *g, Decision_t *d,uint8_t *disc) {
	if (osMutexWait(commDataMutexHandle, 1) == osOK) {
		*g = global_state;
		*d = decision_b2;
		*disc = is_discordant;
		osMutexRelease(commDataMutexHandle);
	}
}

void Communication_Init(void) {
	global_state.system_degradato = 0;
	timer_start = xTaskGetTickCount();
}

void Communication_Step(void) {
	uint8_t rx_buffer = 0;
	uint8_t state_changed;

	do {
		state_changed = 0;

		switch (currentState) {
		case COMM_STATE_RECEIVE_RTS_B2:
			connected_B2 = 0;

			if (HAL_GPIO_ReadPin(RTS_B1_GPIO_Port, RTS_B1_Pin)
					== GPIO_PIN_SET) {
				currentState = COMM_STATE_SEND_CTS;
				state_changed = 1;
			}

			break;

		case COMM_STATE_SEND_CTS:
			HAL_GPIO_WritePin(CTS_B2_GPIO_Port, CTS_B2_Pin, GPIO_PIN_SET);

			vTaskDelay(pdMS_TO_TICKS(PULSE_WIDTH_MS));
			HAL_GPIO_WritePin(CTS_B2_GPIO_Port, CTS_B2_Pin, GPIO_PIN_RESET);

			currentState = COMM_STATE_READ_HANDSHAKE;
			state_changed = 1;
			break;

		case COMM_STATE_READ_HANDSHAKE:
			if (HAL_UART_Receive(&hlpuart1, &rx_buffer, 1, 0) == HAL_OK) {
				if (rx_buffer == HANDSHAKE_RX_VAL) {
					handshake_rx_success_B2 = 1;
				}
			}

			if (handshake_rx_success_B2 != 0U) {
				currentState = COMM_STATE_SEND_RTS_B1;
				state_changed = 1;
			}
			break;

		case COMM_STATE_SEND_RTS_B1:
			HAL_GPIO_WritePin(RTS_B2_GPIO_Port, RTS_B2_Pin, GPIO_PIN_SET);

			if (HAL_GPIO_ReadPin(CTS_B1_GPIO_Port, CTS_B1_Pin)
					== GPIO_PIN_SET) {
				HAL_GPIO_WritePin(RTS_B2_GPIO_Port, RTS_B2_Pin, GPIO_PIN_RESET);
				currentState = COMM_STATE_SEND_HANDSHAKE1;
				state_changed = 1;
			}
			break;

		case COMM_STATE_SEND_HANDSHAKE1:
			uint8_t tx_byte = HANDSHAKE_TX_VAL;
			if (HAL_UART_Transmit(&hlpuart1, &tx_byte, 1, HANDSHAKE_TIMEOUT_MS)
					== HAL_OK) {
				handshake_tx_success_B2 = 1;
			}

			if (handshake_tx_success_B2 != 0U) {
				currentState = COMM_STATE_REACH_CONNECTION;
				handshake_tx_success_B2 = 0;
				state_changed = 1;
			}
			break;

		case COMM_STATE_REACH_CONNECTION:
			connected_B2 = 1;

			if ((connected_B2 != 0U) && (connected_B1 != 0U)) {
				currentState = COMM_STATE_START;
				state_changed = 1;
			}
			break;

		case COMM_STATE_START:
			success_send = 0;
			success_read = 0;

			/* Hardware heartbeat check */
			board_1_alive = HAL_GPIO_ReadPin(BOARD_ALIVE_B1_GPIO_Port,
			BOARD_ALIVE_B1_Pin);

			/* Case: Board 1 dead */
			if (board_1_alive == 0U) {
				currentState = COMM_STATE_DEATH_B1;
				state_changed = 1;
			}
			/* Case: Data reception (B1 RTS high) */
			else if ((HAL_GPIO_ReadPin(RTS_B1_GPIO_Port, RTS_B1_Pin)
					== GPIO_PIN_SET) && (board_1_alive != 0U)) {
				currentState = COMM_STATE_RECEIVE_PACKET_B2;
				state_changed = 1;
			}
			break;

		case COMM_STATE_RECEIVE_PACKET_B2:
			HAL_GPIO_WritePin(CTS_B2_GPIO_Port, CTS_B2_Pin, GPIO_PIN_SET);

			currentState = COMM_STATE_READ_PACKET;
			state_changed = 1;
			break;

		case COMM_STATE_READ_PACKET:
			/* Use HAL timeout to simulate delay */
			if (HAL_UART_Receive(&hlpuart1, packet_rx_buffer, PACKET_B2_SIZE,
			UART_TIMEOUT_MS) == HAL_OK) {
				if (ReadUart_packet_B2(packet_rx_buffer, PACKET_B2_SIZE,
						&partial_state_b2, &received_seq_num) != 0U) {
					success_read = 1;
					seq_num = (uint32_t) received_seq_num;
				} else {
					success_read = 0;
				}
			} else {
				success_read = 0;
			}

			if (success_read != 0U) {
				seq_num++;
				if (osMutexWait(commDataMutexHandle, osWaitForever) == osOK) {
					global_state.system_degradato = 0;
					osMutexRelease(commDataMutexHandle);
				}
				currentState = COMM_STATE_PACKET_RECEIVED;
				state_changed = 1;
			} else {
				HAL_GPIO_WritePin(CTS_B2_GPIO_Port, CTS_B2_Pin, GPIO_PIN_RESET);

				if (loss_packet_1_counter <= MAX_LOSS_COUNT) {
					currentState = COMM_STATE_READ_PACKET;
					loss_packet_1_counter++;
					/* state_changed remains 0 to yield and wait */
				} else {
					currentState = COMM_STATE_IMPOSSIBLE_RECEIVE_FROM_B2;
					state_changed = 1;
				}
			}
			break;

		case COMM_STATE_IMPOSSIBLE_RECEIVE_FROM_B2:
			if (osMutexWait(commDataMutexHandle, osWaitForever) == osOK) {
				global_state.system_degradato = 1;
				osMutexRelease(commDataMutexHandle);
			}
			loss_packet_1_counter = 0;
			currentState = COMM_STATE_RECEIVE_RTS_B2;
			state_changed = 1;
			break;

		case COMM_STATE_PACKET_RECEIVED:
			success_read = 0;
			HAL_GPIO_WritePin(CTS_B2_GPIO_Port, CTS_B2_Pin, GPIO_PIN_RESET);

			currentState = COMM_STATE_SERIALIZE_PACKET_B1;
			state_changed = 1;
			break;

		case COMM_STATE_SERIALIZE_PACKET_B1:
			if (osMutexWait(partialStateMutexHandle, osWaitForever) == osOK) {
				HAL_GPIO_WritePin(RTS_B1_GPIO_Port, RTS_B1_Pin, GPIO_PIN_SET);
				loss_packet_1_counter = 0; /* Reset previous reception error counter */

				serializePartialStateB1(&partial_state_b1, seq_num, buffer_b2);

				osMutexRelease(partialStateMutexHandle);

				if (HAL_GPIO_ReadPin(CTS_B1_GPIO_Port, CTS_B1_Pin)
						== GPIO_PIN_SET) {
					currentState = COMM_STATE_SEND_PACKET_B1;
					state_changed = 1;
				}
			}
			break;

		case COMM_STATE_SEND_PACKET_B1:
			if (HAL_UART_Transmit(&hlpuart1, buffer_b2, PACKET_B2_SIZE,
			UART_TIMEOUT_MS) == HAL_OK) {
				success_send = 1;
			} else {
				success_send = 0;
			}

			if (success_send != 0U) {
				seq_num++;
				if (osMutexWait(commDataMutexHandle, osWaitForever) == osOK) {
					global_state.system_degradato = 0;
					osMutexRelease(commDataMutexHandle);
				}
				currentState = COMM_STATE_PACKET_SENT;
				state_changed = 1;
			} else {
				/* Send Error (Retry Logic) */
				if (loss_send_counter <= MAX_LOSS_COUNT) {
					HAL_GPIO_WritePin(RTS_B2_GPIO_Port, RTS_B2_Pin,
							GPIO_PIN_RESET);
					currentState = COMM_STATE_SEND_PACKET_B1;
					loss_send_counter++;
					/* state_changed remains 0 to yield */
				} else {
					HAL_GPIO_WritePin(RTS_B2_GPIO_Port, RTS_B2_Pin,
							GPIO_PIN_RESET);
					currentState = COMM_STATE_IMPOSSIBLE_SEND_FROM_B1;
					state_changed = 1;
				}
			}
			break;

		case COMM_STATE_PACKET_SENT:
			HAL_GPIO_WritePin(RTS_B2_GPIO_Port, RTS_B2_Pin, GPIO_PIN_RESET);
			loss_send_counter = 0;
			success_send = 0;
			currentState = COMM_STATE_GLOBAL_STATE_RECONSTRUCTION;
			state_changed = 1;
			break;

		case COMM_STATE_GLOBAL_STATE_RECONSTRUCTION:
			if (osMutexWait(commDataMutexHandle, osWaitForever) == osOK) {
				global_state = reconstructGlobalState(partial_state_b1,
						partial_state_b2);
				osMutexRelease(commDataMutexHandle);
			}

			currentState = COMM_STATE_TAKE_DECISION_B1;
			state_changed = 1;
			break;

		case COMM_STATE_TAKE_DECISION_B1:
			if (osMutexWait(commDataMutexHandle, osWaitForever) == osOK) {
				decision_b2 = calculateDecision(&global_state);
				osMutexRelease(commDataMutexHandle);
			}

			/* Accept if B1 RTS is high */
			if (HAL_GPIO_ReadPin(RTS_B1_GPIO_Port, RTS_B1_Pin)
					== GPIO_PIN_SET) {
				currentState = COMM_STATE_RECEIVE_DECISION_B2;
				state_changed = 1;
			}
			break;

		case COMM_STATE_RECEIVE_DECISION_B2:
			HAL_GPIO_WritePin(CTS_B2_GPIO_Port, CTS_B2_Pin, GPIO_PIN_SET);

			currentState = COMM_STATE_READ_DECISION_B2;
			state_changed = 1;
			break;

		case COMM_STATE_READ_DECISION_B2:
			if (HAL_UART_Receive(&hlpuart1, rx_buffer_decision,
			DECISION_BUFFER_SIZE, UART_TIMEOUT_MS) == HAL_OK) {
				if (ReadUart_Decision(rx_buffer_decision, DECISION_BUFFER_SIZE,
						&decision_b1, &received_seq_num) != 0U) {
					success_read = 1;
					seq_num = (uint32_t) received_seq_num;
				} else {
					success_read = 0;
				}
			} else {
				success_read = 0;
			}

			if (success_read != 0U) {
				seq_num++;
				currentState = COMM_STATE_DECISION_RECEIVED;
				state_changed = 1;
			} else {
				/* Critical Error */
				if (loss_packet_1_counter > MAX_LOSS_COUNT) {
					HAL_GPIO_WritePin(CTS_B2_GPIO_Port, CTS_B2_Pin,
							GPIO_PIN_RESET);
					currentState = COMM_STATE_IMPOSSIBLE_RECEIVE_FROM_B2;
					state_changed = 1;
				} else {
					loss_packet_1_counter++;
				}
			}
			break;

		case COMM_STATE_DECISION_RECEIVED:
			HAL_GPIO_WritePin(CTS_B2_GPIO_Port, CTS_B2_Pin, GPIO_PIN_RESET);
			success_read = 0;
			loss_packet_1_counter = 0;

			currentState = COMM_STATE_SERIALIZE_DECISION_B1;
			state_changed = 1;
			break;

		case COMM_STATE_SERIALIZE_DECISION_B1:
			HAL_GPIO_WritePin(RTS_B2_GPIO_Port, RTS_B2_Pin, GPIO_PIN_SET);
			serializeDecisionB1(&decision_b1, seq_num, buffer_decision);

			if (HAL_GPIO_ReadPin(CTS_B1_GPIO_Port, CTS_B1_Pin)
					== GPIO_PIN_SET) {
				currentState = COMM_STATE_DECISION_SEND;
				state_changed = 1;
			}
			break;

		case COMM_STATE_DECISION_SEND:
			if (HAL_UART_Transmit(&hlpuart1, buffer_decision,
			DECISION_BUFFER_SIZE, UART_TIMEOUT_MS) == HAL_OK) {
				currentState = COMM_STATE_DECISION_SENT;
				state_changed = 1;
			}
			break;

		case COMM_STATE_DECISION_SENT:
			HAL_GPIO_WritePin(RTS_B2_GPIO_Port, RTS_B2_Pin, GPIO_PIN_RESET);
			success_send = 0;
			loss_send_counter = 0;

			/* Proceed to actuate if B1 CTS goes low */
			if (HAL_GPIO_ReadPin(CTS_B1_GPIO_Port, CTS_B1_Pin)
					== GPIO_PIN_RESET) {
				currentState = COMM_STATE_ACTUATE_DECISION;
				state_changed = 1;
			}
			break;

		case COMM_STATE_ACTUATE_DECISION:
			if (osMutexWait(commDataMutexHandle, osWaitForever) == osOK) {
				is_discordant = CheckDiscordance(decision_b1, decision_b2);

				if (is_discordant == 0) {
					currentState = COMM_STATE_START;
				} else {
					/* Discordance management */
					global_state.system_degradato = 1;
					currentState = COMM_STATE_START;
				}
				osMutexRelease(commDataMutexHandle);

			}
			state_changed = 0; /* Terminal State: Yield */
			break;

		case COMM_STATE_IMPOSSIBLE_SEND_FROM_B1:
			if (osMutexWait(commDataMutexHandle, osWaitForever) == osOK) {
				global_state.system_degradato = 1;
				osMutexRelease(commDataMutexHandle);
			}
			loss_send_counter = 0;
			currentState = COMM_STATE_RECEIVE_RTS_B2;
			state_changed = 1;
			break;

		case COMM_STATE_DEATH_B1:
			/* Board 1 death management */
			if ((board_1_alive != 0U)
					&& ((xTaskGetTickCount() - timer_start)
							>= pdMS_TO_TICKS(ALIVE_CHECK_MS))) {
				if (osMutexWait(commDataMutexHandle, osWaitForever) == osOK) {
					global_state.system_degradato = 0;
					osMutexRelease(commDataMutexHandle);
				}
				timer_start = xTaskGetTickCount();
				currentState = COMM_STATE_RECEIVE_RTS_B2;

			}
			//decision_b2 = calculateDecision_B2(&partial_state_b2);

			state_changed = 0; /* Terminal State: Yield */
			break;

		default:
			currentState = COMM_STATE_RECEIVE_RTS_B2;
			break;
		}
	} while (state_changed == 1);
}

/*uint8_t CheckDiscordance(Decision_t decision_b1, Decision_t decision_b2) {
	return decision_b1.dir != decision_b2.dir || decision_b1.led != decision_b2.led || decision_b1.setpoint != decision_b2.setpoint;
}*/

