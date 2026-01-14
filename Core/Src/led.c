#include "led.h"
#include "tim.h"

/* --- DEFINIZIONE ISTANZE LED --- */
/* 2 LED Bianchi collegati a PWM (TIM15) */
static Led_t led_white_l = {LED_WHITE_LEFT_GPIO_Port,  LED_WHITE_LEFT_Pin,  &htim15, TIM_CHANNEL_1};
static Led_t led_white_r = {LED_WHITE_RIGHT_GPIO_Port, LED_WHITE_RIGHT_Pin, &htim15, TIM_CHANNEL_2};

/* 2 LED Rossi collegati a GPIO Output */
static Led_t led_red_l   = {LED_RED_LEFT_GPIO_Port,    LED_RED_LEFT_Pin,    NULL, 0};
static Led_t led_red_r   = {LED_RED_RIGHT_GPIO_Port,   LED_RED_RIGHT_Pin,   NULL, 0};

/* --- FUNZIONI PRIVATE --- */
static void Led_Set(Led_t* led, uint32_t val) {
    if (led->tim != NULL) {
        /* Gestione PWM: val è il duty cycle (0-1000 in base al Period del TIM15) */
        __HAL_TIM_SET_COMPARE(led->tim, led->channel, val);
    } else {
        /* Gestione GPIO: val > 0 accende, val == 0 spegne */
        HAL_GPIO_WritePin(led->port, led->pin, (val > 0) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    }
}

static void Led_Toggle(Led_t* led) {
    if (led->tim != NULL) {
        uint32_t curr = __HAL_TIM_GET_COMPARE(led->tim, led->channel);
        __HAL_TIM_SET_COMPARE(led->tim, led->channel, (curr > 0) ? 0 : 1000);
    } else {
        HAL_GPIO_TogglePin(led->port, led->pin);
    }
}

/* --- FUNZIONI PUBBLICHE --- */

void Led_Init(void) {
    /* Avvio PWM per i LED bianchi */
    HAL_TIM_PWM_Start(&htim15, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim15, TIM_CHANNEL_2);
    
    /* Stato iniziale: Tutto spento */
    Led_Off();
}

void Led_Off(void) {
    Led_Set(&led_white_l, 0);
    Led_Set(&led_white_r, 0);
    Led_Set(&led_red_l, 0);
    Led_Set(&led_red_r, 0);
}

void Led_On(void) {
    /* Bianchi al massimo, Rossi spenti */
    Led_Set(&led_white_l, 1000);
    Led_Set(&led_white_r, 1000);
    Led_Set(&led_red_l, 0);
    Led_Set(&led_red_r, 0);
}

void Led_Auto(void) {
    /* Modalità automatica: Bianchi al 50% */
    Led_Set(&led_white_l, 500);
    Led_Set(&led_white_r, 500);
    Led_Set(&led_red_l, 0);
    Led_Set(&led_red_r, 0);
}

void Led_Min(void) {
    /* Modalità risparmio/minimo: Bianchi al 10% */
    Led_Set(&led_white_l, 100);
    Led_Set(&led_white_r, 100);
    Led_Set(&led_red_l, 0);
    Led_Set(&led_red_r, 0);
}

void Led_Emergency(void) {
    /* Lampeggio LED Rossi (entrambi) ogni 250ms */
    static uint32_t last_tick = 0;
    if (HAL_GetTick() - last_tick > 250) {
        Led_Toggle(&led_red_l);
        Led_Toggle(&led_red_r);
        last_tick = HAL_GetTick();
    }
    /* Bianchi spenti in emergenza */
    Led_Set(&led_white_l, 0);
    Led_Set(&led_white_r, 0);
}

void Led_Left(void) {
    /* Lampeggio LED Rosso Sinistro (indicatore di direzione) */
    static uint32_t last_tick = 0;
    if (HAL_GetTick() - last_tick > 500) {
        Led_Toggle(&led_red_l);
        Led_Toggle(&led_white_l);
        Led_Toggle(&led_white_r);
        last_tick = HAL_GetTick();
    }
    /* Assicura che gli altri siano spenti o in stato sicuro */
    Led_Set(&led_red_r, 0);
}

void Led_Right(void) {
    /* Lampeggio LED Rosso Destro (indicatore di direzione) */
    static uint32_t last_tick = 0;
    if (HAL_GetTick() - last_tick > 500) {
        Led_Toggle(&led_red_r);
        Led_Toggle(&led_white_l);
        Led_Toggle(&led_white_r);
        last_tick = HAL_GetTick();
    }
    /* Assicura che gli altri siano spenti o in stato sicuro */
    Led_Set(&led_red_l, 0);
}

void Led_Left_Min(void) {
    /* Lampeggio LED Rosso Sinistro, Bianchi fissi al minimo */
    static uint32_t last_tick = 0;
    if (HAL_GetTick() - last_tick > 500) {
        Led_Toggle(&led_red_l);
        last_tick = HAL_GetTick();
    }
    Led_Set(&led_red_r, 0);
    Led_Set(&led_white_l, 100);
    Led_Set(&led_white_r, 100);
}

void Led_Right_Min(void) {
    /* Lampeggio LED Rosso Destro, Bianchi fissi al minimo */
    static uint32_t last_tick = 0;
    if (HAL_GetTick() - last_tick > 500) {
        Led_Toggle(&led_red_r);
        last_tick = HAL_GetTick();
    }
    Led_Set(&led_red_l, 0);
    Led_Set(&led_white_l, 100);
    Led_Set(&led_white_r, 100);
}
