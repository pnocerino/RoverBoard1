#ifndef INC_LED_H_
#define INC_LED_H_

#include "main.h"

/* Struttura di un LED */
typedef struct {
    GPIO_TypeDef* port;       /* Porta GPIO */
    uint16_t pin;             /* Pin GPIO */
    TIM_HandleTypeDef* tim;   /* Handle del Timer (NULL se GPIO puro) */
    uint32_t channel;         /* Canale del Timer (0 se GPIO puro) */
} Led_t;

/* Prototipi Funzioni */
void Led_Init(void);
void Led_Off(void);
void Led_On(void);
void Led_Auto(void);
void Led_Emergency(void);
void Led_Min(void);
void Led_Left(void);
void Led_Right(void);
void Led_Left_Min(void);
void Led_Right_Min(void);

#endif /* INC_LED_H_ */
