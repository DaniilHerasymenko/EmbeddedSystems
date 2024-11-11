/*
 * LCD.c
 *
 *  Created on: Nov 11, 2024
 *      Author: Daniil
 */

#include "wh1602.h"
#include "LCD.h"

void LCD_Init(void)
{
	lcd_init();
}

void LCD_SendChar(char ch)
{
	lcd_print_char(ch);
}

void LCD_SendString(char *str, uint8_t size)
{
	lcd_print_string(str);
	uint8_t x = 0;
	uint8_t y = 0;

    for (uint8_t i = 0; i < size; i++)
    {
    	lcd_move_to(x, y);
    	lcd_print_char(str[i]);

    	x++;

    	if (x > 15) {
    		x = 0;
    	    y++;

    	    if (y > 1) break;
    	}

    }
}

void LCD_Clear(void)
{
	lcd_clear();
}
