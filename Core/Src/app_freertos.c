/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : app_freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "cmsis_os.h"
#include "communication.h"
#include "reading.h"
#include "temperature.h"
#include "usart.h"
#include <stdbool.h> // Per bool
#include <string.h>  // Per memcpy, strlen
#include <stdio.h>   // Per sprintf
#include <stdlib.h>  // Per abs
#include "actuation.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
typedef StaticTask_t osStaticThreadDef_t;
typedef StaticSemaphore_t osStaticMutexDef_t;
/* USER CODE BEGIN PTD */
#if (configAPPLICATION_ALLOCATED_HEAP == 1)
uint8_t ucHeap[configTOTAL_HEAP_SIZE] __attribute__((section(".ccmram")));
#endif
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
Partial_State_B2_Bus partial_state_b2;
Global_State_t global_state_b1;
Partial_State_B1_Bus partial_state_b1;
Decision_t decision_b1;
Decision_t decision_b2;
bool discordant;
/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 4
};
/* Definitions for readingTask */
osThreadId_t readingTaskHandle __attribute__((section(".ccmram")));
uint32_t readingTaskBuffer[ 512 ] __attribute__((section(".ccmram")));
osStaticThreadDef_t readingTaskControlBlock;
const osThreadAttr_t readingTask_attributes = {
  .name = "readingTask",
  .stack_mem = &readingTaskBuffer[0],
  .stack_size = sizeof(readingTaskBuffer),
  .cb_mem = &readingTaskControlBlock,
  .cb_size = sizeof(readingTaskControlBlock),
  .priority = (osPriority_t) osPriorityNormal2,
};
/* Definitions for comTask */
osThreadId_t comTaskHandle __attribute__((section(".ccmram")));
uint32_t comTaskBuffer[ 512 ] __attribute__((section(".ccmram")));
osStaticThreadDef_t comTaskControlBlock;
const osThreadAttr_t comTask_attributes = {
  .name = "comTask",
  .stack_mem = &comTaskBuffer[0],
  .stack_size = sizeof(comTaskBuffer),
  .cb_mem = &comTaskControlBlock,
  .cb_size = sizeof(comTaskControlBlock),
  .priority = (osPriority_t) osPriorityNormal3,
};
/* Definitions for actuationTask */
osThreadId_t actuationTaskHandle __attribute__((section(".ccmram")));
uint32_t actuationTaskBuffer[ 512 ] __attribute__((section(".ccmram")));
osStaticThreadDef_t actuationTaskControlBlock;
const osThreadAttr_t actuationTask_attributes = {
  .name = "actuationTask",
  .stack_mem = &actuationTaskBuffer[0],
  .stack_size = sizeof(actuationTaskBuffer),
  .cb_mem = &actuationTaskControlBlock,
  .cb_size = sizeof(actuationTaskControlBlock),
  .priority = (osPriority_t) osPriorityNormal4,
};
/* Definitions for temperatureTask */
osThreadId_t temperatureTaskHandle __attribute__((section(".ccmram")));
uint32_t temperatureTaskBuffer[ 512 ] __attribute__((section(".ccmram")));
osStaticThreadDef_t temperatureTaskControlBlock;
const osThreadAttr_t temperatureTask_attributes = {
  .name = "temperatureTask",
  .stack_mem = &temperatureTaskBuffer[0],
  .stack_size = sizeof(temperatureTaskBuffer),
  .cb_mem = &temperatureTaskControlBlock,
  .cb_size = sizeof(temperatureTaskControlBlock),
  .priority = (osPriority_t) osPriorityNormal1,
};
/* Definitions for commDataMutex */
osStaticMutexDef_t commDataMutexControlBlock __attribute__((section(".ccmram")));
osMutexId_t commDataMutexHandle;
const osMutexAttr_t commDataMutex_attributes = {
.name = "partialStateMutex",
.cb_mem = &commDataMutexControlBlock,
.cb_size = sizeof(commDataMutexControlBlock),
};
/* Definitions for partialStateMutex */
osStaticMutexDef_t partialStateMutexControlBlock __attribute__((section(".ccmram")));
osMutexId_t partialStateMutexHandle;
const osMutexAttr_t partialStateMutex_attributes = {
.name = "partialStateMutex",
.cb_mem = &commDataMutexControlBlock,
.cb_size = sizeof(commDataMutexControlBlock),
};


/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
HAL_StatusTypeDef TransmitDecision(UART_HandleTypeDef *huart,Decision_t *decision);
HAL_StatusTypeDef ReceiveDecision(UART_HandleTypeDef *huart,Decision_t *decision) ;
Decision_t calculateDecision(Global_State_t *global_state);
void reconstructGlobalState(Partial_State_B1_Bus *b1, Partial_State_B2_Bus *b2,Global_State_t *global);
HAL_StatusTypeDef ReceivePartialStateBoard2(UART_HandleTypeDef *huart,	Partial_State_B2_Bus *state);
HAL_StatusTypeDef TransmitPartialStateBoard1(UART_HandleTypeDef *huart,Partial_State_B1_Bus *state);
void printExit(const char *taskName);
void printEnter(const char *taskName);
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void readingFunction(void *argument);
void comFunction(void *argument);
void actuationFunc(void *argument);
void temperatureFunc(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */
  /* Create the mutex(es) */
  /* creation of commDataMutex */
  commDataMutexHandle = osMutexNew(&commDataMutex_attributes);

  /* creation of partialStateMutex */
  partialStateMutexHandle = osMutexNew(&partialStateMutex_attributes);

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of readingTask */
  readingTaskHandle = osThreadNew(readingFunction, NULL, &readingTask_attributes);

  /* creation of comTask */
  comTaskHandle = osThreadNew(comFunction, NULL, &comTask_attributes);

  /* creation of actuationTask */
  actuationTaskHandle = osThreadNew(actuationFunc, NULL, &actuationTask_attributes);

  /* creation of temperatureTask */
  temperatureTaskHandle = osThreadNew(temperatureFunc, NULL, &temperatureTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_readingFunction */
/**
* @brief Function implementing the readingTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_readingFunction */
void readingFunction(void *argument)
{
  /* USER CODE BEGIN readingFunction */
  uint32_t tick = osKernelGetTickCount();
  Reading_Init();

  for (;;) {
      printEnter("Reading");
      tick += pdMS_TO_TICKS(TASK_READING_PERIOD_MS);
      Reading_Step(); // Esegue la FSM di lettura encoder + ultrasuoni
      printExit("Reading");
      // Il task si mette in stato Blocked fino allo scoccare del tick preciso
      osDelayUntil(tick);
  }
  /* USER CODE END readingFunction */
}

/* USER CODE BEGIN Header_comFunction */
/**
* @brief Function implementing the comTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_comFunction */
void comFunction(void *argument)
{
  /* USER CODE BEGIN comFunction */
  /* Infinite loop */
  uint32_t tick = osKernelGetTickCount();
  Partial_State_B2_Bus partial_state_b2={0};
  Partial_State_B1_Bus tx_b1_local = {0};
  Global_State_t global_local = {0};
  Decision_t local_decision = {0};

  //Communication_Init();

  for (;;) {
      printEnter("Communication");
      tick += pdMS_TO_TICKS(TASK_COMUNICATION_PERIOD_MS);

      // 1. Prelievo atomico dei dati letti dai sensori
      if (osMutexAcquire(partialStateMutexHandle, 2) == osOK) {
          memcpy(&tx_b1_local, &partial_state_b1, sizeof(Partial_State_B1_Bus));
          osMutexRelease(partialStateMutexHandle);
      }

      // 2. Scambio UART (Sequenza: TX -> RX)
      TransmitPartialStateBoard1(&hlpuart1, &tx_b1_local);
      ReceivePartialStateBoard2(&hlpuart1, &partial_state_b2);

      // 3. Elaborazione Decisionale
      reconstructGlobalState(&tx_b1_local,&partial_state_b2, &global_local);
      local_decision = calculateDecision(&global_local);

      // 4. Scambio Decisioni
      TransmitDecision(&hlpuart1, &local_decision);
      ReceiveDecision(&hlpuart1, &decision_b2);

      // 5. Aggiornamento dati condivisi per l'attuazione
      if (osMutexAcquire(commDataMutexHandle, 2) == osOK) {
          global_state_b1 = global_local;
          decision_b1 = local_decision;
          osMutexRelease(commDataMutexHandle);
      }

      printExit("Communication");

      osDelayUntil(tick); // Ritorna READY ogni 40ms esatti
  }
  /* USER CODE END comFunction */
}

/* USER CODE BEGIN Header_actuationFunc */
/**
* @brief Function implementing the actuationTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_actuationFunc */
void actuationFunc(void *argument)
{
  /* USER CODE BEGIN actuationFunc */
  /* Infinite loop */

  Global_State_t g_state;
  Decision_t dec;
  uint8_t disc;

  uint32_t tick = osKernelGetTickCount();
  Actuation_Init();

  for (;;) {
      printEnter("Actuation");
      tick += pdMS_TO_TICKS(TASK_ACTUATION_PERIOD_MS);
      // Recuperiamo i dati calcolati dal task di comunicazione
      Communication_GetActuationData(&g_state, &dec, &disc);

      // Eseguiamo il controllo motori
      actuation(g_state, dec, disc);

      printExit("Actuation");

      osDelayUntil(tick); // Ritorna READY ogni 10ms esatti
  }
  /* USER CODE END actuationFunc */
}

/* USER CODE BEGIN Header_temperatureFunc */
/**
* @brief Function implementing the temperatureTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_temperatureFunc */
void temperatureFunc(void *argument)
{
  /* USER CODE BEGIN temperatureFunc */
	uint32_t tick= osKernelGetTickCount();
  Temperature_Init();
  /* Infinite loop */
  for(;;)
  {
	printEnter("Temperature");
	tick += pdMS_TO_TICKS(TASK_TEMPERATURE_PERIOD_MS);
    Temperature_Step();
    printExit("temperature");
    osDelayUntil(tick); /* Periodo di campionamento (es. 100ms) */
  }
  /* USER CODE END temperatureFunc */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void printEnter(const char *taskName) {

	char msg[30];
	sprintf(msg, "Enter Task: %s\r\n", taskName);
	HAL_UART_Transmit(&hlpuart1, (uint8_t*) msg, strlen(msg), 1000);

}

void printExit(const char *taskName) {

	char msg[30];
	sprintf(msg, "Exit Task: %s\r\n", taskName);
	HAL_UART_Transmit(&hlpuart1, (uint8_t*) msg, strlen(msg), 1000);

}

void print(const char *string) {
	char msg[30];
	sprintf(msg, "%s\r\n", string);
	HAL_UART_Transmit(&hlpuart1, (uint8_t*) msg, strlen(msg), 1000);
}

/*static void transmitPartialStateBoard2(void) {
 HAL_StatusTypeDef check;

 do {
 check = HAL_UART_Transmit(&huart1, (uint8_t*) &partial_state_b2,
 sizeof(partial_state_b2), 2); //////////////////////////////////////////////////////////////////////////////////
 } while (check != HAL_OK);

 }*/

HAL_StatusTypeDef TransmitPartialStateBoard1(UART_HandleTypeDef *huart,Partial_State_B1_Bus *state) {

	// 1. Calcola la dimensione totale della struct
	uint16_t dataSize = sizeof(Partial_State_B1_Bus);

	// 2. Cast della struct a puntatore di uint8_t
	// Questo permette alla HAL di leggere la memoria della struct come un array di byte
	uint8_t *pData = (uint8_t*) state;

	// 3. Trasmissione in Polling
	// Il timeout deve essere calcolato per coprire il "Worst Case"
	// Per ~30 byte a 115200 baud, 10ms sono più che sufficienti.
	return HAL_UART_Transmit(huart, pData, dataSize, 4);
}

HAL_StatusTypeDef ReceivePartialStateBoard2(UART_HandleTypeDef *huart,	Partial_State_B2_Bus *state) {

	uint16_t dataSize = sizeof(Partial_State_B2_Bus); // Ora sono 20 byte

	// Timeout ridotto a 10ms per la simulazione del WCET.
	// Se il cavo è scollegato, la CPU resterà occupata per esattamente 10ms.
	return HAL_UART_Receive(huart, (uint8_t*) state, dataSize,5 );
}

/*void reconstructGlobalState(Partial_State_B1_Bus *b1, Partial_State_B2_Bus *b2,Global_State_t *global) {

	// --- DATI DA BOARD 1 (Cast da float a double) ---
	global->b1_vel_fl = b1->b1_vel_fl;
	global->b1_vel_fr = b1->b1_vel_fr;
	global->b1_vel_rl = b1->b1_vel_rl;
	global->b1_vel_rr = b1->b1_vel_rr;

	// Battery e Temp erano uint8_t nella nuova struct B1
	global->b1_battery = b1->battery_level;
	global->b1_temp = b1->temperatura;

	global->b1_emergenza = b1->emergenza;
	global->b1_degradato = b1->degradato;

	// --- DATI DA BOARD 2 ---
	global->controller = b2->controller;
	global->sonar_left = b2->dL;
	global->sonar_center = b2->dC;
	global->sonar_right = b2->dR;
	global->angolo = b2->angolo;
	global->b2_emergenza = b2->emergenza;
	global->b2_degradato = b2->degradato;

	// --- LOGICA DI SISTEMA (JOIN) ---
	// Il sistema è in emergenza se almeno una delle due schede lo è
	global->system_emergenza = (b1->emergenza || b2->emergenza) ? 1 : 0;

	// Il sistema è degradato se una scheda è degradata o se ci sono errori di comunicazione
	// (Puoi integrare qui la variabile 'system_degradato' gestita dal task)
	global->system_degradato = (b1->degradato || b2->degradato) ? 1 : 0;
}

Decision_t calculateDecision(Global_State_t *global_state) {
	// --- VARIABILI PERSISTENTI (STATIC) ---
	static Controller_Bus prev_ctrl = { 0 };
	static uint8_t led_state = LED_OFF;
	static float timer_memory_L = 0.0f;
	static float timer_memory_R = 0.0f;

	// --- INIZIALIZZAZIONE DECISIONE ---
	Decision_t decision;
	decision.setpoint = 0;
	decision.dir = DIR_INIT;
	decision.led = led_state;

	float vel = 0.0f;
	uint8_t avoiding_obstacle = 0;

	// --- 1. CONTROLLO SICUREZZA ---
	if (global_state->system_emergenza) {
		decision.dir = DIR_STOP_EMERGENCY;
		decision.setpoint = 0;
		return decision;
	}

	// --- 2. AGGIORNAMENTO TIMER (NON-BLOCKING) ---
	if (timer_memory_L > 0)
		timer_memory_L -= DT;
	if (timer_memory_R > 0)
		timer_memory_R -= DT;

	// --- 3. ELABORAZIONE INPUT JOYSTICK ---
	float input_y = (float) global_state->controller.ry / 512.0f;
	if (global_state->controller.ry < 0)
		input_y = 0;

	float val_r2 = (float) global_state->controller.r2 / 1020.0f;
	float val_l2 = (float) global_state->controller.l2 / 1020.0f;
	float input_x = val_r2 - val_l2;

	if (abs(input_y) < DEADZONE)
		input_y = 0.0f;
	if (abs(input_x) < DEADZONE)
		input_x = 0.0f;

	// --- 4. LOGICA OSTACOLI ---
	float dist_C = global_state->sonar_center;
	float dist_L = global_state->sonar_left;
	float dist_R = global_state->sonar_right;

	// Caso sistema degradato
	if (global_state->system_degradato) {
		if (dist_C < 400.0f || dist_L < 400.0f || dist_R < 400.0f) {
			avoiding_obstacle = 1;
			decision.dir = DIR_STOP_EMERGENCY;
			vel = 0.0f;
		}
	}

	if (input_y >= 0.0f && !avoiding_obstacle) {
		// A. STOP EMERGENZA (Sensori vicini)
		if (dist_C < OBSTACLE_THRESH_STOP || dist_L < OBSTACLE_THRESH_STOP
				|| dist_R < OBSTACLE_THRESH_STOP) {
			avoiding_obstacle = 1;
			decision.dir = DIR_STOP_EMERGENCY;
			vel = 0.0f;
		} else {
			// B. RILEVAZIONE LATERALE (Armamento Timer)
			if (dist_L >= DIST_FAR_MIN && dist_L <= DIST_FAR_MAX)
				timer_memory_L = MEMORY_WINDOW;
			if (dist_R >= DIST_FAR_MIN && dist_R <= DIST_FAR_MAX)
				timer_memory_R = MEMORY_WINDOW;

			// C. SEQUENZA CENTRALE
			if (dist_C >= DIST_FAR_MIN && dist_C <= DIST_FAR_MAX) {
				if (timer_memory_L > 0 && timer_memory_L >= timer_memory_R) {
					avoiding_obstacle = 1;
					decision.dir = DIR_LEFT_SPOT;
					vel = MAX_SPEED * 0.5f;
				} else if (timer_memory_R > 0
						&& timer_memory_R > timer_memory_L) {
					avoiding_obstacle = 1;
					decision.dir = DIR_RIGHT_SPOT;
					vel = MAX_SPEED * 0.5f;
				}
			}
		}
	}

	// --- 5. GUIDA NORMALE ---
	if (!avoiding_obstacle) {
		// Logica Toggle LED (R1)
		if (global_state->controller.r1 && !prev_ctrl.r1) {
			if (led_state == LED_OFF)
				led_state = LED_ON;
			else if (led_state == LED_ON)
				led_state = LED_AUTO;
			else
				led_state = LED_OFF;
		}
		decision.led = led_state;

		// Gestione pulsanti e movimenti
		if (global_state->controller.square && !prev_ctrl.square) {
			decision.dir = (input_x > 0) ? DIR_RIGHT_SPOT : DIR_LEFT_SPOT;
			vel = abs(input_x) * 0.5f;
		} else if (global_state->controller.circle && !prev_ctrl.circle) {
			decision.dir = (input_x > 0) ? DIR_RIGHT_PIVOT : DIR_LEFT_PIVOT;
			vel = abs(input_x) * 0.5f;
		} else if (input_y > 0.1f) {
			decision.dir = DIR_FORWARD;
			vel = input_y * MAX_SPEED;
		} else {
			decision.dir = DIR_INIT;
			vel = 0.0f;
		}
	}

	// --- 6. APPLICAZIONE DEGRADAZIONE VELOCITÀ ---
	if (global_state->system_degradato && !avoiding_obstacle) {
		vel = input_y * 0.3f;
	}

	// --- 7. FINALIZE ---
	prev_ctrl = global_state->controller;
	decision.setpoint = (int16_t) vel; // Nota: converte m/s in intero (personalizza se serve mm/s)

	return decision;
}*/

HAL_StatusTypeDef TransmitDecision(UART_HandleTypeDef *huart,Decision_t *decision) {
	// Trasmettiamo i 4 byte della struct.
	// Timeout di 5ms è ampiamente sufficiente per il Worst Case.
	return HAL_UART_Transmit(huart, (uint8_t*) decision, sizeof(Decision_t), 2);
}

HAL_StatusTypeDef ReceiveDecision(UART_HandleTypeDef *huart,Decision_t *decision) {
	// Riceviamo i 4 byte.
	// In assenza di segnale, la CPU resterà occupata per 10ms.
	return HAL_UART_Receive(huart, (uint8_t*) decision, sizeof(Decision_t), 2);
}
/* USER CODE END Application */

