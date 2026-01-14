#include "actuation.h"
#include "sys_rover.h"
#include "led.h"


void Actuation_Init(void) {
    Led_Init();
}

void actuation(Global_State_t global_state, Decision_t decision, uint8_t is_discordant) {
    


    if (is_discordant) {
       if (decision.dir == 4 || decision.dir == 6) {
           Led_Left_Min();
       } else if (decision.dir == 5 || decision.dir == 7) {
           Led_Right_Min();
       } else {
           Led_Min();
       }
    } else {
        /* --- CASO NORMALE (CONCORDANTE) --- */
        switch (decision.led) {
            case LED_OFF:
                if (decision.dir == 4 || decision.dir == 6) {
                    Led_Left();
                } else if (decision.dir == 5 || decision.dir == 7) {
                    Led_Right();
                } else {
                    Led_Off();
                }
                break;
            case LED_ON:
                if (decision.dir == 4 || decision.dir == 6) {
                    Led_Left();
                } else if (decision.dir == 5 || decision.dir == 7) {
                    Led_Right();
                } else {
                    Led_On();
                }
                break;

            case LED_AUTO:
                if (decision.dir == 4 || decision.dir == 6) {
                    Led_Left();
                } else if (decision.dir == 5 || decision.dir == 7) {
                    Led_Right();
                } else {
                    Led_Auto();
                }
                break;

            case LED_EMERGENCY:
                Led_Emergency();
                break;
        }
    }
}
