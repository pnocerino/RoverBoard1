#ifndef INC_READING_H_
#define INC_READING_H_

#include "main.h"
#include "cmsis_os.h"
#include "communication.h"
// Status defines
#define ENC_OK          0
#define ENC_NOT_OK      1
#define BATTERY_OK      0
#define BATTERY_NOT_OK  1

#define TASK_READING_PERIOD_MS  (40U)



extern Partial_State_B1_Bus partial_state_b1;
extern osMutexId_t boardStateMutexHandle;

void Reading_Task_Init(void);
void Reading_Task_Loop(void);
void Reading_Step(void);
void Reading_Init(void);


#endif /* INC_READING_H_ */
