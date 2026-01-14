#ifndef ACTUATION_H
#define ACTUATION_H

#ifdef __cplusplus
extern "C" {
#endif

#include <communication.h>

#define TASK_ACTUATION_PERIOD_MS  (10U)


void Actuation_Init(void);
void actuation(Global_State_t global_state, Decision_t decision, uint8_t is_discordant);

#ifdef __cplusplus
}
#endif

#endif /* ACTUATION_H */
