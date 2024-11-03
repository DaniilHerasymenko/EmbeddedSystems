/*
 * LCD.c
 *
 *  Created on: Sep 25, 2024
 *      Author: volvo
 */


#include "wh1602.h"
#include "LCD.h"

/**
 * @brief LCD initialization
 *
 */
void LCD_Init(void)
{
	lcd_init();
}

/**s
 * @brief Show character on the display
 *
 * @param ch symbol to display
 */
void LCD_SendChar(char ch)
{
	lcd_print_char(ch);
}

/**
 * @brief Show an array of characters on the display
 *
 * @param str strinh/array to display
 * @param size number of characters
 */
void LCD_SendString(char *str, uint8_t size)
{

//    for (uint8_t i = 0; i < size; i++)
//    {
//    	lcd_print_char(str[i]);
//    }
	lcd_print_string(str);
//	uint8_t x = 0;
//	  uint8_t y = 0;
//
//	    for (uint8_t i = 0; i < size; i++)
//	    {
//	      lcd_move_to(x, y);
//	      lcd_print_char_at(str[i], x, y);
//
//	      x++;
//
//	      if (x > 15) {
//	        x = 0;
//	          y++;
//
//
//	          if (y > 1) {
//	            break;
//	          }
//	      }
//
//	    }
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


    	    if (y > 1) {
    	    	break;
    	    }
    	}

    }


	//lcd_print_string(str);
}

/**
 * @brief Clear the LCD
 *
 */
void LCD_Clear(void)
{
	lcd_clear();
}

