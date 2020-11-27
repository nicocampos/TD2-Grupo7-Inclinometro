/*
 * task.c
 *
 *  Created on: 2 nov. 2020
 *      Author: nicoc
 */

#include "task.h"
#include "mpu_6050.h"
#include "ssd1306.h"
#include "fonts.h"
#include "stdio.h"
#include <math.h>
#include "eeprom.h"

// Variables globales
extern I2C_HandleTypeDef hi2c1;
MPU6050_t MPU6050;
float offset_x = 0;
float offset_y = 0;
float valuetoSave = 0;
uint8_t tecla = NO_KEY;
uint8_t f_hold = OFF;
uint8_t f_zero = OFF;
uint8_t fEjeY = OFF;
uint8_t fAlarms = OFF;
float customAlarms[CANT_ALARMAS] = {30,45,90,EEPROM_EMPTY,EEPROM_EMPTY,EEPROM_EMPTY,EEPROM_EMPTY};

////////////////////////////
//	 FUNCION ANTIREBOTE	  //
////////////////////////////
uint8_t antirebote (uint8_t teclaActual)
{
	static uint8_t teclaAnt = NO_KEY;
	static uint8_t cont = 0;

	if(cont == 0){
		teclaAnt = teclaActual;
		cont++;
	}
	else{
		cont = 0;
		if(teclaActual == teclaAnt){
			return teclaAnt;
		}
		else{
			return NO_KEY;
		}
	}
	return NO_KEY;
}

void lastAlarm(char *string)
{
	float aux = 0;

	for(int i = 0; i < CANT_ALARMAS; i++){
		if(customAlarms[i] != EEPROM_EMPTY)
			aux = customAlarms[i];
		else
			break;
	}
	if(aux!=90){
		if(aux < 10){
			sprintf(string,"0%.2f", aux);
		}
		else{
			sprintf(string,"%.2f", aux);
		}
	}
	else{	//aux==90
		sprintf(string,"%d   ", (int)aux);
	}
}


// TAREAS

void tarea_pulsadores (void *p)
{

	if(HAL_GPIO_ReadPin(MODE_KEY)){
		tecla = MODE;
	}

	if(HAL_GPIO_ReadPin(ZERO_KEY)){
		tecla = ZERO;
	}

	if(HAL_GPIO_ReadPin(HOLD_KEY)){
		tecla = HOLD;
	}

	if (tecla != NO_KEY)
		tecla = antirebote(tecla);

}

void tarea_modos(void *p)
{
	static uint8_t clean = OFF;
	static uint8_t mode = MODO_MEDIR;

	if(tecla == MODE){
		mode++;
		mode%=CANT_MODOS;
		clean = OFF;
	}

	switch(mode){
	case MODO_MEDIR:
		if(clean == OFF){
			SSD1306_Fill(SSD1306_COLOR_BLACK); /* Clear screen */
			offset_x = 0;
			offset_y = 0;
			clean = ON;
			f_hold = OFF;
		}
		SSD1306_GotoXY (POS_MODO);
		SSD1306_Puts("Modo: 1", &Font_7x10, SSD1306_COLOR_WHITE);
		fEjeY = OFF;
		fAlarms = OFF;
		if(tecla == HOLD){
			f_hold = ~ f_hold; // toggle flag
		}
		if(tecla == ZERO){
			f_zero = ON;
		}
		break;

	case MODO_NIVEL:
		if(clean == OFF){
			SSD1306_Fill(SSD1306_COLOR_BLACK); /* Clear screen */
			offset_x = 0;
			offset_y = 0;
			clean = ON;
			f_hold = OFF;
		}
		SSD1306_GotoXY (POS_MODO);
		SSD1306_Puts("Modo: 2", &Font_7x10, SSD1306_COLOR_WHITE);
		fEjeY = ON;
		fAlarms = OFF;
		if(tecla == HOLD){
			f_hold = ~ f_hold; // toggle flag
		}
		if(tecla == ZERO){
			f_zero = ON;
		}
		break;
	case MODO_ALARMAS:
		if(clean == OFF){
			SSD1306_Fill(SSD1306_COLOR_BLACK); /* Clear screen */
			offset_x = 0;
			offset_y = 0;
			clean = ON;
			f_hold = OFF;
		}
		SSD1306_GotoXY (POS_MODO);
		SSD1306_Puts("Modo: 3", &Font_7x10, SSD1306_COLOR_WHITE);
		fEjeY = OFF;
		fAlarms = ON;
		if(tecla == HOLD){	// Guardar
			EEPROM_refresh(EEPROM_SAVE);
		}
		if(tecla == ZERO){	// Borrar
			EEPROM_refresh(EEPROM_DELETE);
		}

		break;
	default:
		mode = MODO_MEDIR;
	}
}

void tarea_display(void *p)
{
	static int cont = 0;				//!< Variable contador para el promedio
	static float prom_x = 0;			//!<
	static float prom_y = 0;			//!<
	static float valorAntX = 0;			//!<
	static float valorAntY = 0;			//!<
	float prom_offset_x = 0;			//!<
	float prom_offset_y = 0;			//!<
	char str_x[5];						//!<
	char str_y[5];						//!<
	char str1[6] = {0};				//!<
	char str2[6] = {0};				//!<

	if(cont >= PROMEDIO){
		cont = 0;
		prom_x /= PROMEDIO;
		prom_y /= PROMEDIO;

		//------------------------------------------------------------------------------------------------
		//--------------------------------------- Chequeo signo -----------------------------------------
		//------------------------------------------------------------------------------------------------
		if(offset_x != 0){
			prom_x = fabs(prom_x);

			if((prom_offset_x) < 0)
				str1[0] = 118; // v
			else
				str1[0] = 94; // ^
		}
		else{
			if(prom_x < 0)
				str1[0] = 118; // v
			else
				str1[0] = 94; // ^
			prom_x = fabs(prom_x);
		}

		if(offset_y != 0){
			prom_y = fabs(prom_y);

			if((prom_offset_y) < 0)
				str2[0] = 118; // v
			else
				str2[0] = 94; // ^
		}
		else{
			if(prom_y < 0)
				str2[0] = 118; // v
			else
				str2[0] = 94; // ^
			prom_x = fabs(prom_x);
		}

		prom_offset_x = prom_x - offset_x;
		prom_offset_y = prom_y - offset_y;

		//------------------------------------------------------------------------------------------------
		//----------------------------------------------- Zero -------------------------------------------
		//------------------------------------------------------------------------------------------------
		if(f_zero == ON){
			offset_x = prom_x;
			offset_y = prom_y;
			f_zero = OFF;
		}

		//------------------------------------------------------------------------------------------------
		//------------------------------------------- THRESHOLD ------------------------------------------
		//------------------------------------------------------------------------------------------------
		if(fabs(prom_offset_x - valorAntX) < THRESHOLD )
			prom_offset_x = valorAntX;
		if(fabs(prom_offset_x - valorAntY) < THRESHOLD )
			prom_offset_y = valorAntY;

		//------------------------------------------------------------------------------------------------
		//-------------------------------------------- Valor Anterior ------------------------------------
		//------------------------------------------------------------------------------------------------
		valorAntX = prom_offset_x;
		valorAntY = prom_offset_y;

		//------------------------------------------------------------------------------------------------
		//---------------------------------------- Redondeo ----------------------------------------------
		//------------------------------------------------------------------------------------------------
		if(fabs(prom_offset_x) >= 10 ){
			valuetoSave = prom_offset_x;
			sprintf(str_x,"%.2f", roundf(fabs(prom_offset_x) * 100)/100);
			strcat(str1, str_x);
		}
		else{
			valuetoSave = prom_offset_x;
			sprintf(str_x,"0%.2f", roundf(fabs(prom_offset_x) * 100)/100);
			strcat(str1, str_x);
		}
		if(prom_offset_y >= 10 ){
			sprintf(str_y,"%.2f", roundf(fabs(prom_offset_y) * 100)/100);
			strcat(str2, str_y);
		}
		else{
			sprintf(str_y,"0%.2f", roundf(fabs(prom_offset_y) * 100)/100);
			strcat(str2, str_y);
		}


		//------------------------------------------------------------------------------------------------
		//-------------------------------------- MODO 1 o MODO 2 -----------------------------------------
		//------------------------------------------------------------------------------------------------
		if(fEjeY == OFF){	// Solo eje X
			if(fAlarms == OFF){
				SSD1306_GotoXY (POS_GRADOS);
				SSD1306_Puts(str1, &Font_16x26, SSD1306_COLOR_WHITE);
			}
			else{
				SSD1306_GotoXY(POS_GRADOSX);
				SSD1306_Puts(str1, &Font_11x18, SSD1306_COLOR_WHITE);
				SSD1306_GotoXY(POS_GRADOSA);
				lastAlarm(str2);
				SSD1306_Puts(str2, &Font_11x18, SSD1306_COLOR_WHITE);
			}
		}
		else{	// Solo eje Y
			SSD1306_GotoXY(POS_GRADOSX);
			SSD1306_Puts(str1, &Font_11x18, SSD1306_COLOR_WHITE);
			SSD1306_GotoXY(POS_GRADOSY);
			SSD1306_Puts(str2, &Font_11x18, SSD1306_COLOR_WHITE);
		}

		prom_x = 0;
		prom_y = 0;
	}
	else{
		prom_x += MPU6050.KalmanAngleX;
		prom_y += MPU6050.KalmanAngleY;
		cont++;
	}


}

void tarea_orienta(void *p)
{
	MPU6050_Read_All(&hi2c1, &MPU6050);
}


void tarea_led_blinking(void *p)
{
	HAL_GPIO_TogglePin(GPIOC, Led_Blink_Pin);
}


void tarea_refresh(void *p)
{
	if(f_hold == OFF)
		SSD1306_UpdateScreen();
}
