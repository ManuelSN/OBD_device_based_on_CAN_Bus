/*
 * Buttons.h
 *
 *  Created on: 13 dic. 2020
 *      Author: Lolo
 *
 *      This work is licensed under the Creative Commons Attribution-NonCommercial 4.0 International License.
 *      To view a copy of this license, visit http://creativecommons.org/licenses/by-nc/4.0/ or send a letter to
 *      Creative Commons, PO Box 1866, Mountain View, CA 94042, USA.
 */

#ifndef BUTTONS_H_
#define BUTTONS_H_

// Libraries
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "ST7735.h"

#define MENU_BUTTON GPIO_PIN_0
#define RIGHT_BUTTON GPIO_PIN_1
#define DOWN_BUTTON GPIO_PIN_2
#define UP_BUTTON GPIO_PIN_3
#define OK_BUTTON GPIO_PIN_4
#define LEFT_BUTTON GPIO_PIN_5

#define BUTTONS_PORT_BASE GPIO_PORTE_BASE
#define BUTTONS_PERIPH SYSCTL_PERIPH_GPIOE
#define BUTTONS_PIN  (MENU_BUTTON | RIGHT_BUTTON | DOWN_BUTTON | UP_BUTTON | OK_BUTTON | LEFT_BUTTON)
#define INT_BUTTONS_PERIPH INT_GPIOE

#define DELAY_BOUNCE 20e-3

// Functions
void init_Buttons(void);
void init_AntiBounce(void);
void init_buttonTasks(void);
void ButtonsIntHandler(void);
void AntiBounceIntHandler(void);


#endif /* BUTTONS_H_ */
