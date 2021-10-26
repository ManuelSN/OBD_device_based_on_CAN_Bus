/*
 * Graphic_interface.h
 *
 *  Created on: 23 oct. 2020
 *      Author: Manuel Sánchez Natera
 *
 *      This work is licensed under the Creative Commons Attribution-NonCommercial 4.0 International License.
 *      To view a copy of this license, visit http://creativecommons.org/licenses/by-nc/4.0/ or send a letter to
 *      Creative Commons, PO Box 1866, Mountain View, CA 94042, USA.
 */

#ifndef GRAPHIC_INTERFACE_H_
#define GRAPHIC_INTERFACE_H_

// Libraries
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "ST7735.h"

// Menu defines
#define MENU_BG_COLOUR Colour565(0,0,10)
#define MENU_ITEM_UNSELECTED_BG_COLOUR Colour565(0,0,10)
#define MENU_ITEM_SELECTED_TEXT_COLOUR Colour565(0,0,10)
#define MENU_ITEM_SELECTED_BG_COLOUR Colour565(210,210,255)
#define MENU_ITEM_UNSELECTED_TEXT_COLOUR Colour565(210,210,255)
#define MENU_DATA_TEXT_COLOUR Colour565(255,0,0)
#define MENU_ITEM_TEXT_SIZE 1
#define MENU_ITEM_POS_X0 2
#define MENU_ITEM_POS_Y0 2
#define MENU_ITEM_POS_OFFSET 10
#define MENU_ITEMS 6
#define MENU_ECU_ITEMS 3

// Menu showed values
#define MENU_ECU 0
#define MENU_MODE 1

void init_graphicInterface(void);
char convert2Hex(uint16_t decimal);
void decimal2Hex(const char cadena[], char *cadenaHex);
uint16_t hex2Decimal(const char cadenaHex[]);
void hex2Binary(char *cadena_hex, char **cadena_bin_output);
void drawCANframe(int16_t x, int16_t y, char *CAN_frame, uint16_t colour,  uint16_t bg, uint8_t size);
void drawMenu(void);
void drawECUMenu(void);
void cleanScreen(void);
void cleanData(uint8_t posData);
void tabular(char cadena[]);

static const char *menu_ECU_items[] = {

          "ECM - ID: 0x7E0",
          "TCM - ID: 0x7E1",
          "ABS - ID: 0x7E2"
};

static const char *menu_items[] = {

          "Vehicle information",
          "Read codes (DTC)",
          "Erase codes",
          "View freeze frame",
          "Live all data",
          "DTCs during driving cycle"
};



#endif /* GRAPHIC_INTERFACE_H_ */
