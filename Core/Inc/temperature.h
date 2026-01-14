#ifndef INC_TEMPERATURE_H_
#define INC_TEMPERATURE_H_

#include "main.h"

/* Definizioni Stati Flag */
#define TEMP_OK     0
#define TEMP_NOT_OK 1
#define TASK_TEMPERATURE_PERIOD_MS (200U)

void Temperature_Init(void);
void Temperature_Step(void);

float Temperature_GetValue(void);
uint8_t Temperature_GetFlag(void);

#endif /* INC_TEMPERATURE_H_ */
