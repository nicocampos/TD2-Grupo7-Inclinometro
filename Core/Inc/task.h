/*
 * task.h
 *
 *  Created on: 2 nov. 2020
 *      Author: nicoc
 */


#ifndef INC_TASK_H_
#define INC_TASK_H_

#include "main.h"

#define MODE_KEY			GPIOB, inputMode_Pin
#define HOLD_KEY			GPIOB, inputHold_Pin
#define ZERO_KEY			GPIOB, inputZero_Pin

#define MODE				1
#define ZERO				2
#define HOLD				3

#define CANT_MODOS			3
#define	NO_KEY				0

#define MPU6050_ADDR 		0xD0
#define MAX_LEN_TASK_LIST	(8)
#define TICK_SISTEMA		(10)
#define TIMEOUT_I2C			10
#define PROMEDIO			50
#define POS_GRADOS			10,30	//30,30
#define POS_GRADOSX			20,15
#define POS_GRADOSY			20,40
#define POS_GRADOSA			20,40
#define POS_MODO			5,0

// ESTADOS - MODOS
#define MODO_MEDIR			0
#define MODO_NIVEL			1
#define MODO_ALARMAS		2

#define ON					1
#define OFF					0

#define THRESHOLD			0.13


/*
 * Prototipos de las funciones que hacen al sistema.
 */


void tarea_led_blinking(void *p);
void tarea_display(void *p);
void tarea_orienta(void *p);
void tarea_refresh(void *p);
void tarea_pulsadores(void *p);
void tarea_modos(void *p);
void tarea_alarmas(void *p);


/*
 * Funciones necesarias para algunas tareas
 */

uint8_t antirebote (uint8_t lectura_actual);
void lastAlarm(char *);

#endif /* INC_TASK_H_ */
