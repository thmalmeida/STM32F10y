/*
 * motor.h
 *
 *  Created on: 13 de jul de 2020
 *      Author: thmalmeida
 */

#ifndef MOTOR_H_
#define MOTOR_H_

#include "Hardware/gpio.h"
#include <stm32f10x.h>

class motorAC : GPIO {
public:
	uint8_t motorStatus = 0;				// motor state: 1- on, 0- off;
	uint16_t motorTimerE = 0;				// time elapsed to turn load off;

	uint8_t flag_Th = 0;					// thermal relay
};




#endif /* MOTOR_H_ */
