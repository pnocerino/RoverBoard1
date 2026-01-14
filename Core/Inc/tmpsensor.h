/*
 * tmpsensor.h
 *
 *  Created on: Nov 24, 2025
 *      Author: gabri
 */

#ifndef INC_TMPSENSOR_H_
#define INC_TMPSENSOR_H_

#include "main.h"

/* Configurations BEGIN */
#define TMPSENSOR_USE_INTREF 1 /* 1 - Use Internal Reference Voltage; 0 - Not use; */
/* Configurations END */

/* Constant values BEGIN */
#define TMPSENSOR_AVGSLOPE 2.5 /* ogni aumento di un grado della temperatura comporta un aumento di 2,5 mV di tensione nel sensore */
#define TMPSENSOR_V25  0.76 /* voltaggio del sensore a 25°C */

#define TMPSENSOR_ADCMAX 4095.0 /* 12-bit ADC maximum value (12^2)-1)  */
#define TMPSENSOR_ADCREFVOL  3.3 /* Typical reference voltage, V  */
#define TMPSENSOR_ADCVREFINT  1.21 /* Internal reference voltage, V questo voltaggio è molto preciso */
/* Constant values END */


/* PFP BEGIN */
uint8_t TMPSENSOR_getTemperature(uint16_t adc_sensor, uint16_t adc_intref);
/* PFP END */


#endif /* INC_TMPSENSOR_H_ */