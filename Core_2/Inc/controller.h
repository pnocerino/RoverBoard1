/*
 * controller.h
 *
 *  Created on: Dec 5, 2025
 *      Author: Rover Team
 */

#ifndef DUALSENSE_H
#define DUALSENSE_H

#include <stdint.h>

typedef struct __attribute__((packed)) {
    int16_t ry;

    uint16_t l2;
    uint16_t r2;

    uint8_t cross;
    uint8_t circle;
    uint8_t square;
    uint8_t triangle;

    uint8_t l1;
    uint8_t r1;

    uint8_t battery;
} Controller_Bus;


void read_controller_data(Controller_Bus* ds);

#endif // CONTROLLER_H
