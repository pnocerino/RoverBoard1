/*
 * joycon_buttons.h
 *
 *  Created on: Dec 5, 2025
 *      Author: Rover Team
 */

#ifndef DUALSENSE_H
#define DUALSENSE_H

#include <stdint.h> // Necessario per usare int16_t e uint16_t
//#include "communication.h"

// ==========================================================
// STRUTTURA DATI CONDIVISA (Deve essere identica su ESP32 e STM32)
// ==========================================================
typedef struct __attribute__((packed)) {
    // --- ASSI ANALOGICI ---
    int16_t ry;       // Stick Destro Y (Usato per avanti/indietro)

    // --- GRILLETTI ANALOGICI ---
    uint16_t l2; // Valore 0-1023 (o 0-255 a seconda sender)
    uint16_t r2; // Valore 0-1023

    // --- PULSANTI (Booleans: 1 = Premuto, 0 = Rilasciato) ---
    uint8_t cross;
    uint8_t circle;
    uint8_t square;
    uint8_t triangle;

    uint8_t l1;
    uint8_t r1;

    uint8_t battery;
} Controller_Bus;


void read_controller_data(Controller_Bus* ds);

#endif // JOYCON_DEFINES_H
