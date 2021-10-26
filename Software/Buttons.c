/*
 * Buttons.c
 *
 *  Created on: 13 dic. 2020
 *      Author: Lolo
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
//#include "inc/tm4c123gh6pm.h"
//#include "inc/hw_memmap.h"
//#include "inc/hw_gpio.h"
//#include "inc/hw_types.h"
//#include "inc/hw_ints.h"
//#include "inc/hw_can.h"
//#include "inc/hw_uart.h"
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
#include "Buttons.h"

// Global variables
extern uint16_t menu_cursor, menu_ECU_cursor, menu_showed;
extern EventGroupHandle_t flagEvents;
static TaskHandle_t Button_taskHandler = NULL;
QueueHandle_t buttons_queue;
bool left_button_state, menu_button_state;
extern bool live_all_data_mode, OnMenu;

static portTASK_FUNCTION(Button_pressed, pvParameters){

    uint32_t buttons_status;

    while(1){

        if (xQueueReceive(buttons_queue, &buttons_status, portMAX_DELAY) == pdTRUE){
            //xEventGroupWaitBits(flagEvents, BUTTON_PRESSED, pdTRUE, pdFALSE, portMAX_DELAY);
            if(!(buttons_status & MENU_BUTTON)) { // Menu button pressed

                //drawString(MENU_ITEM_POS_X0, MENU_ITEM_POS_Y0 + (MENU_ITEM_POS_OFFSET), "Menu pulsado", MENU_ITEM_UNSELECTED_TEXT_COLOUR, MENU_ITEM_UNSELECTED_BG_COLOUR, MENU_ITEM_TEXT_SIZE, 0);
                if (menu_showed == MENU_MODE){
                    if (!menu_button_state){

                       menu_button_state = true;
                    }else {

                       menu_cursor = 0;
                       cleanScreen();
                       drawMenu();
                    }
                } else{
                    drawECUMenu();
                }


            }else if(!(buttons_status & RIGHT_BUTTON)) { // Right button pressed

                //drawString(MENU_ITEM_POS_X0, MENU_ITEM_POS_Y0 + (MENU_ITEM_POS_OFFSET),"Derecho pulsado", MENU_ITEM_UNSELECTED_TEXT_COLOUR, MENU_ITEM_UNSELECTED_BG_COLOUR, MENU_ITEM_TEXT_SIZE, 0);
                if (menu_showed == MENU_MODE){
                    drawMenu();
                }else {
                    drawECUMenu();
                }

            }else if(!(buttons_status & DOWN_BUTTON)) { // Down button pressed

                if (menu_showed == MENU_MODE){
                    if(menu_cursor < MENU_ITEMS-1){
                        menu_cursor++;
                        drawMenu();
                    }
                }else {
                    if (menu_ECU_cursor < MENU_ECU_ITEMS-1){
                        menu_ECU_cursor++;
                        drawECUMenu();
                    }
                }

            }else if(!(buttons_status & UP_BUTTON)) { // Up button pressed

                if (menu_showed == MENU_MODE){
                    if(menu_cursor > 0){
                        menu_cursor--;
                        drawMenu();
                    }
                }else {
                    if (menu_ECU_cursor > 0){
                        menu_ECU_cursor--;
                        drawECUMenu();
                    }
                }

            }else if(!(buttons_status & OK_BUTTON)) { // Ok button pressed

                if (menu_showed == MENU_MODE){

                    xEventGroupSetBits(flagEvents, SELECT_CAN_COMMAND);
                } else{
                    xEventGroupSetBits(flagEvents, SELECT_ECU_ADDRESS);
                }

            }else if(!(buttons_status & LEFT_BUTTON)) { // Left button pressed

                left_button_state = true;
                if (OnMenu){
                    menu_showed = MENU_ECU;
                    menu_ECU_cursor = 0;
                    cleanScreen();
                    drawECUMenu();
                }

            }

        }else {

            drawString(MENU_ITEM_POS_X0, MENU_ITEM_POS_Y0 + (MENU_ITEM_POS_OFFSET), "Buttons queue error", MENU_ITEM_UNSELECTED_TEXT_COLOUR, MENU_ITEM_UNSELECTED_BG_COLOUR, MENU_ITEM_TEXT_SIZE, 0);
        }
        // To Enable the Buttons Peripheric after dealing the interruption to don´t block the queue (full).
        IntEnable(INT_BUTTONS_PERIPH);

    }

}

void init_Buttons(void) {

    // Enable the GPIO port to which the pushbuttons are connected.
    ROM_SysCtlPeripheralEnable(BUTTONS_PERIPH);

    // Set each of the button GPIO pins as an input with a pull-up.
    ROM_GPIODirModeSet(BUTTONS_PORT_BASE, BUTTONS_PIN, GPIO_DIR_MODE_IN);

    //GPIOPadConfigSet(MENU_BUTTON_PERIPH, MENU_BUTTON, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);
    ROM_GPIOPadConfigSet(BUTTONS_PORT_BASE, BUTTONS_PIN, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);

    ROM_GPIOIntTypeSet(BUTTONS_PORT_BASE, BUTTONS_PIN, GPIO_FALLING_EDGE);

    init_AntiBounce();

    ROM_IntEnable(INT_BUTTONS_PERIPH);

    ROM_IntPrioritySet(INT_GPIOE, configMAX_SYSCALL_INTERRUPT_PRIORITY);

    GPIOIntEnable(BUTTONS_PORT_BASE, BUTTONS_PIN);

    // Create queue of Button ISR
    buttons_queue = xQueueCreate(1, sizeof(uint32_t));

    left_button_state = false;
    menu_button_state = false;

}

void init_AntiBounce(void){

    uint32_t delay_bounce;

    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER3);

    ROM_TimerConfigure(TIMER3_BASE, TIMER_CFG_PERIODIC);

    delay_bounce = (SysCtlClockGet()*DELAY_BOUNCE);
    ROM_TimerLoadSet(TIMER3_BASE, TIMER_A, delay_bounce);

    ROM_TimerIntEnable(TIMER3_BASE, TIMER_TIMA_TIMEOUT);

    ROM_IntEnable(INT_TIMER3A);

    // RTOS masked interrupts higher than configMAX_SYSCALL_INTERRUPT_PRIORITY,
    // to use FreeRTOS API calls from ISR the priority has to be equal or lower than the macro.
    ROM_IntPrioritySet(INT_TIMER3A, configMAX_SYSCALL_INTERRUPT_PRIORITY);
}

void init_buttonTasks(void){

    if ((xTaskCreate(Button_pressed, (portCHAR *)"Buttons", 512, NULL,tskIDLE_PRIORITY + 1, &Button_taskHandler) != pdTRUE)){

        while(1);
    }
}

void AntiBounceIntHandler(void){

    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    GPIOIntClear(BUTTONS_PORT_BASE, BUTTONS_PIN);

    ROM_TimerEnable(TIMER3_BASE, TIMER_A);

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}



void ButtonsIntHandler(void){

    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    uint32_t pin_status;

    ROM_TimerIntClear(TIMER3_BASE, TIMER_TIMA_TIMEOUT);
    ROM_TimerDisable(TIMER3_BASE, TIMER_A);

    pin_status = GPIOPinRead(BUTTONS_PORT_BASE, BUTTONS_PIN);
    ROM_IntDisable(INT_GPIOE);

    xQueueSendFromISR(buttons_queue, &pin_status, &xHigherPriorityTaskWoken);

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}
