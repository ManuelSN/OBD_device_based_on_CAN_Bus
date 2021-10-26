/*
 * Graphic_interface.c
 *
 *  Created on: 23 oct. 2020
 *      Author: Manuel Sánchez Natera
 *
 *      This work is licensed under the Creative Commons Attribution-NonCommercial 4.0 International License.
 *      To view a copy of this license, visit http://creativecommons.org/licenses/by-nc/4.0/ or send a letter to
 *      Creative Commons, PO Box 1866, Mountain View, CA 94042, USA.
 */



// C libraries
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

// TIVA libraries
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/can.h"
#include "driverlib/interrupt.h"
#include "driverlib/timer.h"
#include "driverlib/uart.h"
#include "utils/uartstdio.h"


// FreeRTOS libraries
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "utils/cpu_usage.h"
#include "event_groups.h"


// Programmer libraries
#include "Graphic_interface.h"
#include "CAN_device.h"
#include "ST7735.h"

// Global variables
uint16_t menu_cursor, menu_ECU_cursor, menu_showed;
bool OnMenu;

void cleanScreen(void){

    fillScreen(MENU_BG_COLOUR);

}

void cleanData(uint8_t posData){

    fillRect(120, (posData*10)+5, 40, 10, MENU_BG_COLOUR);
}

void init_graphicInterface(void) {

    LcdInit();
    menu_cursor = 0;
    menu_ECU_cursor = 0;
    menu_showed = MENU_ECU;
    OnMenu = false;
    setRotation(1);
    cleanScreen();

}

void tabular(char cadena[]){

    uint8_t size = strlen(cadena);
    char *cadena_tabulada = (char*)pvPortMalloc(sizeof(char)*size);
    uint8_t j = 1;

    cadena_tabulada[0] = ' ';
    for (int i = 0; i < size; i++){

        if (cadena[i] != '.'){
            cadena_tabulada[j] = cadena[i];
        } else{
            cadena_tabulada[j] = '\0';
        }
        j++;
    }

    strcpy(cadena, cadena_tabulada);
    vPortFree(cadena_tabulada);
}

char convert2Hex(const uint16_t decimal) {

    char hexadecimal;

    if (decimal >= 10) {
        switch(decimal) {
        case 10:
            hexadecimal = 'A';
           break;
        case 11:
            hexadecimal = 'B';
            break;
        case 12:
            hexadecimal = 'C';
           break;
        case 13:
            hexadecimal = 'D';
           break;
        case 14:
            hexadecimal = 'E';
           break;
        case 15:
            hexadecimal = 'F';
           break;

       }
    } else {

           hexadecimal = (char)decimal;
           // convert to ASCII format to show on display (offset of 48)
           hexadecimal = hexadecimal + 48;
    }

    return hexadecimal;
}

void decimal2Hex(const char cadena[], char *cadenaHex) {


    uint16_t cociente, resto, cont;
    cont = 0;

    for (int i = 0; i < MAX_BYTES; i++){

        cociente = cadena[i]/16;
        resto = cadena[i]%16;

        cadenaHex[cont] = convert2Hex(cociente);
        cadenaHex[cont+1] = convert2Hex(resto);

        cont += 2;
    }

    cadenaHex[HEX_ARRAY] = '\0';

}

void hex2Binary(char *cadena_hex, char **cadena_bin_output){

    int numBytes = sizeOfFrame(cadena_hex);
    char cadena_bin[32];
    *cadena_bin_output = (char*)pvPortMalloc(4*numBytes*sizeof(char));

    int j = 0;

    for (int i = 0; i < numBytes; i++){

        switch(cadena_hex[i]){
            case '0':
                  cadena_bin[j] = '0';
                  j++;
                  cadena_bin[j] = '0';
                  j++;
                  cadena_bin[j] = '0';
                  j++;
                  cadena_bin[j] = '0';
                  break;
            case '1':
                  cadena_bin[j] = '0';
                  j++;
                  cadena_bin[j] = '0';
                  j++;
                  cadena_bin[j] = '0';
                  j++;
                  cadena_bin[j] = '1';
                  break;
            case '2':
                  cadena_bin[j] = '0';
                  j++;
                  cadena_bin[j] = '0';
                  j++;
                  cadena_bin[j] = '1';
                  j++;
                  cadena_bin[j] = '0';
                  break;
            case '3':
                  cadena_bin[j] = '0';
                  j++;
                  cadena_bin[j] = '0';
                  j++;
                  cadena_bin[j] = '1';
                  j++;
                  cadena_bin[j] = '1';
                  break;
            case '4':
                  cadena_bin[j] = '0';
                  j++;
                  cadena_bin[j] = '1';
                  j++;
                  cadena_bin[j] = '0';
                  j++;
                  cadena_bin[j] = '0';
                  break;
            case '5':
                  cadena_bin[j] = '0';
                  j++;
                  cadena_bin[j] = '1';
                  j++;
                  cadena_bin[j] = '0';
                  j++;
                  cadena_bin[j] = '1';
                  break;
            case '6':
                  cadena_bin[j] = '0';
                  j++;
                  cadena_bin[j] = '1';
                  j++;
                  cadena_bin[j] = '1';
                  j++;
                  cadena_bin[j] = '0';
                  break;
            case '7':
                  cadena_bin[j] = '0';
                  j++;
                  cadena_bin[j] = '1';
                  j++;
                  cadena_bin[j] = '1';
                  j++;
                  cadena_bin[j] = '1';
                  break;
            case '8':
                  cadena_bin[j] = '1';
                  j++;
                  cadena_bin[j] = '0';
                  j++;
                  cadena_bin[j] = '0';
                  j++;
                  cadena_bin[j] = '0';
                  break;
            case '9':
                  cadena_bin[j] = '1';
                  j++;
                  cadena_bin[j] = '0';
                  j++;
                  cadena_bin[j] = '0';
                  j++;
                  cadena_bin[j] = '1';
                  break;
            case 'A':
                  cadena_bin[j] = '1';
                  j++;
                  cadena_bin[j] = '0';
                  j++;
                  cadena_bin[j] = '1';
                  j++;
                  cadena_bin[j] = '0';
                  break;
            case 'B':
                  cadena_bin[j] = '1';
                  j++;
                  cadena_bin[j] = '0';
                  j++;
                  cadena_bin[j] = '1';
                  j++;
                  cadena_bin[j] = '1';
                  break;
            case 'C':
                  cadena_bin[j] = '1';
                  j++;
                  cadena_bin[j] = '1';
                  j++;
                  cadena_bin[j] = '0';
                  j++;
                  cadena_bin[j] = '0';
                  break;
            case 'D':
                  cadena_bin[j] = '1';
                  j++;
                  cadena_bin[j] = '1';
                  j++;
                  cadena_bin[j] = '0';
                  j++;
                  cadena_bin[j] = '1';
                  break;
            case 'E':
                  cadena_bin[j] = '1';
                  j++;
                  cadena_bin[j] = '1';
                  j++;
                  cadena_bin[j] = '1';
                  j++;
                  cadena_bin[j] = '0';
                  break;
            case 'F':
                  cadena_bin[j] = '1';
                  j++;
                  cadena_bin[j] = '1';
                  j++;
                  cadena_bin[j] = '1';
                  j++;
                  cadena_bin[j] = '1';
                  break;

        }
        j++;
    }

    memcpy(*cadena_bin_output, cadena_bin, (numBytes*4)+1);
    *(*cadena_bin_output+(numBytes*4)) = '\0';
}

uint16_t hex2Decimal(const char cadenaHex[]) {

    uint16_t size = sizeOfFrame(cadenaHex);
    uint16_t elemento;
    uint16_t decimal = 0;

    for (int i = 0; i < size; i++){
        switch(cadenaHex[i]) {
        case 'A':
            elemento = 10;
           break;
        case 'B':
            elemento = 11;
        case 'C':
            elemento = 12;
           break;
        case 'D':
            elemento = 13;
           break;
        case 'E':
            elemento = 14;
           break;
        case 'F':
            elemento = 15;
           break;
        default:
            // subtract offset of ASCII (48)
            elemento = (uint16_t)cadenaHex[i] - 48;
            break;

       }

        decimal = decimal + elemento*pow(16, (size-1)-i);
    }

    return decimal;
}

void drawCANframe(int16_t x, int16_t y, char *CAN_frame, uint16_t colour,  uint16_t bg, uint8_t size) {

    if((x >= SCREEN_WIDTH)              ||
       (y >= SCREEN_HEIGHT)             ||
       ((x + 6 * size - 1) < 0)  ||
       ((y + 8 * size - 1) < 0))
        return;

    uint16_t frame_size = sizeOfFrame(CAN_frame);

   for (int i = 0; i < frame_size; i++){

        if (!(i%2) && (i != 0)){

            x = x + size*12;
        }else {

            x = x + size*6;
        }

        drawChar(x, y, CAN_frame[i], colour,  bg, size);
    }

}

void drawMenu(void) {

    uint16_t menu_item_bg_colour, menu_item_text_colour;

    for (int i = 0; i < MENU_ITEMS; i++){

        if (i == menu_cursor) {

            menu_item_bg_colour = MENU_ITEM_SELECTED_BG_COLOUR;
            menu_item_text_colour = MENU_ITEM_SELECTED_TEXT_COLOUR;

        } else {

            menu_item_bg_colour = MENU_ITEM_UNSELECTED_BG_COLOUR;
            menu_item_text_colour = MENU_ITEM_UNSELECTED_TEXT_COLOUR;
        }

        drawString(MENU_ITEM_POS_X0, MENU_ITEM_POS_Y0 + (MENU_ITEM_POS_OFFSET*i), menu_items[i], menu_item_text_colour, menu_item_bg_colour, MENU_ITEM_TEXT_SIZE, 0);

    }
}

void drawECUMenu(void){

    uint16_t menu_ECU_item_bg_colour, menu_ECU_item_text_colour;

     for (int i = 0; i < MENU_ECU_ITEMS; i++){

         if (i == menu_ECU_cursor) {

             menu_ECU_item_bg_colour = MENU_ITEM_SELECTED_BG_COLOUR;
             menu_ECU_item_text_colour = MENU_ITEM_SELECTED_TEXT_COLOUR;

         } else {

             menu_ECU_item_bg_colour = MENU_ITEM_UNSELECTED_BG_COLOUR;
             menu_ECU_item_text_colour = MENU_ITEM_UNSELECTED_TEXT_COLOUR;
         }

         drawString(MENU_ITEM_POS_X0, MENU_ITEM_POS_Y0 + (MENU_ITEM_POS_OFFSET*i), menu_ECU_items[i], menu_ECU_item_text_colour, menu_ECU_item_bg_colour, MENU_ITEM_TEXT_SIZE, 0);

     }
}

