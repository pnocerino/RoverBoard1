/*
 * tmpsensor.c
 *
 *  Created on: Nov 24, 2025
 *      Author: gabri
 */


#include "tmpsensor.h"


/**
  * @brief Calculate temperature (tested on STM32F401, other MCU may have different constants!)
  * @note If IntRef not use, set it [ex.: #define TMPSENSOR_USE_INTREF 0]
  * @param Temperature sensor's ADC 16-bit value, Internal Reference ADC 16-bit value (if use)
  * @retval Internal sensor temperature
  */
uint8_t TMPSENSOR_getTemperature(uint16_t adc_sensor, uint16_t adc_intref){
/*prima calcolo la vera tensione a cui sta lavorando il microcontrollore*/

#if(TMPSENSOR_USE_INTREF)
//qui attuo la correzzione per conoscere il valore corretto della tensione a cui lavora la scheda
	double intref_vol = (TMPSENSOR_ADCMAX*TMPSENSOR_ADCVREFINT)/adc_intref;

#else
	double intref_vol = TMPSENSOR_ADCREFVOL;
#endif

	double sensor_vol = adc_sensor * intref_vol/TMPSENSOR_ADCMAX;// calcolo il voltaggio letto dall'ADC

	double sensor_tmp = (sensor_vol - TMPSENSOR_V25) *1000.0/TMPSENSOR_AVGSLOPE + 25.0;/*Calcola la differenza di tensione rispetto al punto standard (25°C), converte in millivolt, tiene conto della pendenza della rette che fornisce la temperatura,
	aggiunge 25 perché è la temperatura base da cui siamo partiti*/

	return sensor_tmp;
}