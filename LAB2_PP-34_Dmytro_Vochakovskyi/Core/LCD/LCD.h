/*
 * LCD.h
 *
 *  Created on: Sep 25, 2024
 *      Author: volvo
 */

#ifndef LCD_LCD_H_
#define LCD_LCD_H_

#include "stdint.h"

void LCD_Init(void);
void LCD_SendChar(char ch);
void LCD_SendString(char *str, uint8_t size);
void LCD_Clear(void);

#endif /* LCD_LCD_H_ */
