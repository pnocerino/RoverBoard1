//#include "FreeRTOS.h"
//#include "task.h"
#include "main.h"
//#include "cmsis_os.h"
#include <math.h>
#include <string.h>
#include <stdio.h>
#include "communication.h"
#include "usart.h"
//#include "reading.h"
#include "helper_communication_b1.h"
#include "gpio.h"

/* Mutex Handle */
//extern osMutexId commDataMutexHandle;
//extern osMutexId partialStateMutexHandle;

/* Current State */
static CommState_t currentState = COMM_STATE_START;
static Global_State_t global_state = { 0 };


/* Logic Support Variables */
static uint8_t success_send = 0;
static uint8_t success_read = 0;
static uint8_t receive_error_counter = 0;
static uint8_t counter_failed_send = 0;
static uint8_t is_discordant = 0;
static int16_t seq_num = 0;
static uint8_t packet_rx_buffer[PACKET_B2_SIZE];
static uint8_t tx_buffer[PACKET_B1_SIZE];
static uint8_t tx_buffer_decision[DECISION_PACKET_SIZE];
static uint8_t buffer_decision[DECISION_PACKET_SIZE];
static Partial_State_B1_Bus partial_state_b1 = { 0 };
static Partial_State_B2_Bus partial_state_b2 = { 0 };
static Decision_t decision_b1 = { 0 };
static Decision_t decision_b2 = { 0 };
static  uint32_t tick_start;
char tx_msg[512];

/* -------------------------------------------------------------------------- */
/* PUBLIC FUNCTIONS                                                           */
/* -------------------------------------------------------------------------- */

void Communication_GetActuationData(Global_State_t *g, Decision_t *d, uint8_t *disc);


void Communication_Init(void) {
	global_state.comunication_degradato = 0;
    global_state.system_emergenza = 0;
    partial_state_b1.b1_vel_fl = 50;
    partial_state_b1.b1_vel_fr = 50;
    partial_state_b1.b1_vel_rl = 50;
    partial_state_b1.b1_vel_rr = 50;
    partial_state_b1.battery_level = 100;
    partial_state_b1.battery_status = 1;
    partial_state_b1.photoresistor = 2005;
    partial_state_b1.temp_status = 1;
    partial_state_b1.encoder_status = 1;
    partial_state_b1.degraded = 0;
    partial_state_b1.emergency = 0;
    partial_state_b1.temperatura = 20;
    HAL_GPIO_WritePin(BOARD_ALIVE_B1_GPIO_Port, BOARD_ALIVE_B1_Pin, GPIO_PIN_SET);
    currentState = COMM_STATE_START;
}

void Communication_Step(void) {
    uint8_t processing_complete = 0;

    do {
        switch (currentState) {
            case COMM_STATE_START:
                success_send = 0;
                success_read = 0;

                if (HAL_GPIO_ReadPin(BOARD_ALIVE_B2_GPIO_Port, BOARD_ALIVE_B2_Pin) == GPIO_PIN_RESET) {
                    currentState = COMM_STATE_SAFE_MODE;
                } else {
                    HAL_GPIO_WritePin(RTS_B2_GPIO_Port, RTS_B2_Pin, GPIO_PIN_SET);
                    currentState = COMM_STATE_SERIALIZE_SENSOR_DATA;
                }
                break;

            case COMM_STATE_SAFE_MODE:
                //if (osMutexAcquire(commDataMutexHandle, 0) == osOK) {
                    global_state = reconstructGlobalState(partial_state_b1, partial_state_b2);
                    global_state.system_emergenza = 1;
                    decision_b1 = decisionStop(&global_state);
                    //osMutexRelease(commDataMutexHandle);
                //}
                processing_complete = 1;
                currentState = COMM_STATE_START;
                break;

            case COMM_STATE_SERIALIZE_SENSOR_DATA:
                //if (osMutexAcquire(partialStateMutexHandle, 0) == osOK) {
                    serializePartialStateB1(&partial_state_b1, (uint8_t) seq_num, tx_buffer);
                    //osMutexRelease(partialStateMutexHandle);
                //}
                tick_start = HAL_GetTick();
                while(HAL_GetTick() - tick_start < 2){
                    if (HAL_GPIO_ReadPin(CTS_B2_GPIO_Port, CTS_B2_Pin) == GPIO_PIN_SET) {
					currentState = COMM_STATE_SERIALIZE_SEND_B1;
					break;

                }
            }
                currentState = COMM_STATE_SAFE_MODE;
                break;

            case COMM_STATE_SERIALIZE_SEND_B1:

                do{
                    if(HAL_UART_Transmit(&hlpuart1, tx_buffer, PACKET_B1_SIZE, UART_TIMEOUT_PARTIAL_MS) == HAL_OK) {
                        success_send = 1;
                    } else {
                        success_send = 0;
                        counter_failed_send++;
                    }
                }while(success_send == 0 && counter_failed_send <= 1);

                if (success_send == 0) {
                    currentState = COMM_STATE_IMPOSSIBLE_SEND_TO_B2;
                } else {
                    seq_num++;
                    currentState = COMM_STATE_PACKET_SENT;
                }
                break;

            case COMM_STATE_PACKET_SENT:
                counter_failed_send = 0;
                success_send = 0;
                HAL_GPIO_WritePin(RTS_B1_GPIO_Port, RTS_B1_Pin, GPIO_PIN_RESET);
                tick_start = HAL_GetTick();
                while(HAL_GetTick() - tick_start < 2){
                    if (HAL_GPIO_ReadPin(RTS_B2_GPIO_Port, RTS_B2_Pin) == GPIO_PIN_SET) {
                        currentState = COMM_STATE_RECEIVE_PACKET;
                        break;
                    }
                }
                currentState = COMM_STATE_SAFE_MODE;
                break;

            case COMM_STATE_RECEIVE_PACKET:
                HAL_GPIO_WritePin(CTS_B1_GPIO_Port, CTS_B1_Pin, GPIO_PIN_SET);
                currentState = COMM_STATE_READ_PACKET;
                break;

            case COMM_STATE_READ_PACKET:
                do{
                    if(HAL_UART_Receive(&hlpuart1, packet_rx_buffer, PACKET_B2_SIZE, UART_TIMEOUT_PARTIAL_MS) == HAL_OK) {
                        int16_t temp_seq;
                        if (ReadUart_packet_B2(packet_rx_buffer, PACKET_B2_SIZE, &partial_state_b2, &temp_seq)) {
                            success_read = 1;
                            seq_num = temp_seq;
                        } else {
                            success_read = 0;
                            receive_error_counter++;

                        }
                    } else {
                        success_read = 0;
                        receive_error_counter++;
                    }
                }while(success_read == 0 && receive_error_counter <= 1);
        
                if (success_read == 0) {
                    currentState = COMM_STATE_IMPOSSIBLE_RECEIVE_FROM_B2;
                } else {
                    seq_num++;
                    currentState = COMM_STATE_PACKET_RECEIVED;
                }
                break;

            case COMM_STATE_PACKET_RECEIVED:
                receive_error_counter = 0;
                HAL_GPIO_WritePin(CTS_B1_GPIO_Port, CTS_B1_Pin, GPIO_PIN_RESET);
                success_read = 0;
                tick_start = HAL_GetTick();
                while(HAL_GetTick() - tick_start < 2){
                    if (HAL_GPIO_ReadPin(RTS_B2_GPIO_Port, RTS_B2_Pin) == GPIO_PIN_RESET) {
                        currentState = COMM_STATE_GLOBAL_STATE_RECONSTRUCTION;
                        break;
                    }
                }
                currentState = COMM_STATE_SAFE_MODE;
                break;

            case COMM_STATE_GLOBAL_STATE_RECONSTRUCTION:
                //if(osMutexAcquire(commDataMutexHandle, 0) == osOK) {
                    global_state = reconstructGlobalState(partial_state_b1, partial_state_b2);
                    //osMutexRelease(commDataMutexHandle);
                //}
                currentState = COMM_STATE_TAKE_DECISION_B1;
                break;

            case COMM_STATE_TAKE_DECISION_B1:
                decision_b1 = calculateDecision(&global_state);
                currentState = COMM_STATE_SERIALIZE_DECISION_B1;
                break;

            case COMM_STATE_SERIALIZE_DECISION_B1:
                HAL_GPIO_WritePin(RTS_B1_GPIO_Port, RTS_B1_Pin, GPIO_PIN_SET);
                serializeDecisionB1(&decision_b1, (uint8_t) seq_num, tx_buffer_decision);
                
                tick_start = HAL_GetTick();
                while(HAL_GetTick() - tick_start < 2){
                    if(HAL_GPIO_ReadPin(CTS_B2_GPIO_Port, CTS_B2_Pin) == GPIO_PIN_SET) {
                        currentState = COMM_STATE_DECISION_SEND;
                        break;
                    }
                }
                currentState = COMM_STATE_SAFE_MODE;
                break;

            case COMM_STATE_DECISION_SEND:
                do{
                    if(HAL_UART_Transmit(&hlpuart1, tx_buffer_decision, DECISION_PACKET_SIZE, UART_TIMEOUT_DECISION_MS) == HAL_OK) {
                        success_send = 1;
                    } else {
                        success_send = 0;
                        counter_failed_send++;
                    }
                }while(success_send == 0 && counter_failed_send <= 1);

                if (success_send == 0) {
                    currentState = COMM_STATE_IMPOSSIBLE_SEND_TO_B2;
                } else {
                    seq_num++;
                    currentState = COMM_STATE_DECISION_SENT;
                }
                break;

            case COMM_STATE_DECISION_SENT:
                HAL_GPIO_WritePin(RTS_B1_GPIO_Port, RTS_B1_Pin, GPIO_PIN_RESET);
                success_send = 0;
                counter_failed_send = 0;

                tick_start = HAL_GetTick();
                while(HAL_GetTick() - tick_start < 2){  
                    if (HAL_GPIO_ReadPin(RTS_B2_GPIO_Port, RTS_B2_Pin) == GPIO_PIN_SET) {
                        currentState = COMM_STATE_READ_DECISION_B2;
                        break;
                    }
                }
                currentState = COMM_STATE_SAFE_MODE;
                break;

            case COMM_STATE_READ_DECISION_B2:
                HAL_GPIO_WritePin(CTS_B1_GPIO_Port, CTS_B1_Pin, GPIO_PIN_SET);
                do{
                    if(HAL_UART_Receive(&hlpuart1, buffer_decision, DECISION_PACKET_SIZE, UART_TIMEOUT_DECISION_MS) == HAL_OK) {
                        int16_t temp_seq;
                        if (ReadUart_decision_B2(buffer_decision, DECISION_PACKET_SIZE, &decision_b2, &temp_seq)) {
                            success_read = 1;
                            seq_num = temp_seq;
                        } else {
                            success_read = 0;
                            receive_error_counter++;

                        }
                    } else {
                        success_read = 0;
                        receive_error_counter++;
                    }
                }while(success_read == 0 && receive_error_counter <= 1);

                if( success_read == 0) {
                    currentState = COMM_STATE_IMPOSSIBLE_RECEIVE_FROM_B2;
                } else {
                    seq_num++;
                    currentState = COMM_STATE_DECISION_RECEIVED;
                }
                break;

            case COMM_STATE_DECISION_RECEIVED:
                receive_error_counter = 0;
                HAL_GPIO_WritePin(CTS_B1_GPIO_Port, CTS_B1_Pin, GPIO_PIN_RESET);
                success_read = 0;
                tick_start = HAL_GetTick();
                while(HAL_GetTick() - tick_start < 2){
                    if (HAL_GPIO_ReadPin(RTS_B2_GPIO_Port, RTS_B2_Pin) == GPIO_PIN_RESET) {
                        currentState = COMM_STATE_ACTUATE_DECISION;
                        break;
                    }
                }
                currentState = COMM_STATE_SAFE_MODE;
                break;

            case COMM_STATE_ACTUATE_DECISION:
                //if(osMutexAcquire(commDataMutexHandle, MUTEX_TIMEOUT_MS) == osOK) {
				is_discordant = CheckDiscordance(decision_b1, decision_b2);
				//osMutexRelease(commDataMutexHandle);
			//}
				int len = snprintf(tx_msg, sizeof(tx_msg),
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
			currentState = COMM_STATE_START;
			processing_complete = 1;
			break;

            case COMM_STATE_IMPOSSIBLE_SEND_TO_B2:
                counter_failed_send = 0;
                HAL_GPIO_WritePin(RTS_B1_GPIO_Port, RTS_B1_Pin, GPIO_PIN_RESET);
                currentState = COMM_STATE_SAFE_MODE;
                break;
            
            case COMM_STATE_IMPOSSIBLE_RECEIVE_FROM_B2:
                receive_error_counter = 0;
                HAL_GPIO_WritePin(CTS_B1_GPIO_Port, CTS_B1_Pin, GPIO_PIN_RESET);
                currentState = COMM_STATE_SAFE_MODE;
                break;

            default:
                currentState = COMM_STATE_START;
                processing_complete = 1;
                break;
        }
    } while (!processing_complete);
}


void Communication_GetActuationData(Global_State_t *g, Decision_t *d, uint8_t *disc) {
		*g = global_state;
		*d = decision_b2;
		*disc = is_discordant;

}


