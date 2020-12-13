/*
 * functions.c
 *
 *  Created on: Dec 12, 2020
 *      Author: root
 */
#include "functions.h"

void strtoINT(uint32_t number, int lenght){
	uch string[lenght];
	sprintf(string,"%d",number);

	LCD_string(string);
}
