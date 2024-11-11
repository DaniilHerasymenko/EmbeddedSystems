/*
 * LCD.h
 *
 *  Created on: Nov 11, 2024
 *      Author: Daniil
 */

#ifndef LCD_LCD_H_
#define LCD_LCD_H_

#include "stdint.h"

void LCD_Init(void);
void LCD_SendChar(char ch);
void LCD_SendString(char *str, uint8_t size);
void LCD_Clear(void);
void lcd_move_to(uint8_t x, uint8_t y);

#endif /* LCD_LCD_H_ */
