/*
 * main.c
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
//#include "inc/hw_memmap.h"
//#include "inc/hw_gpio.h"
//#include "inc/hw_types.h"
//#include "inc/hw_ints.h"
#include "inc/hw_can.h"
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
#include "CAN_device.h"
#include "ST7735.h"
#include "Graphic_interface.h"
#include "Buttons.h"
//#include "sdcard.h"


//Global variables
uint32_t g_ui32CPUUsage;
uint32_t g_ulSystemClock;

tCANBitClkParms psClkParms;

extern volatile bool g_bRXFlag;

//*****************************************************************************
//
// The error routine that is called if the driver library encounters an error.
//
// This function is called if the driverlib or FreeRTOS library checks for the existence of an error (using
// the macros ASSERT(...) and configASSERT(...)
// The parameters filename and line contain information about where the error is...
//
//*****************************************************************************
#ifdef DEBUG
void __error__(char *nombrefich, uint32_t linea)
{
    while(1) // If the execution is in here, it means that RTOS or one of the peripheral libraries has checked for an error.
    { // Look at the call tree in the debugger and the file name and line values for possible clues.
    }
}
#endif


// This is what is executed when the system detects a stack overflow
void vApplicationStackOverflowHook(xTaskHandle *pxTask, signed char *pcTaskName)
{
    //
    // This function can not return, so loop forever.  Interrupts are disabled
    // on entry to this function, so no processor interrupts will interrupt
    // this loop.
    //
    while(1)
    {
    }
}

// This is executed every Tick of the system. It keeps the CPU usage statistics (time the CPU has been running).
void vApplicationTickHook( void )
{
    static uint8_t count = 0;

    if (++count == 10)
    {
        g_ui32CPUUsage = CPUUsageTick();
        count = 0;
    }
    //return;
}

// This is executed each time the Idle task is started.
void vApplicationIdleHook (void)
{
    SysCtlSleep();
}


// This is executed each time the Idle task is started.
void vApplicationMallocFailedHook (void)
{
    while(1);
}


int main(void){

    // Set the main clock to 50 MHz (200 MHz of the Pll divided by 4)
    ROM_SysCtlClockSet(SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ | SYSCTL_OSC_MAIN);

    // Get the system clock speed.
    g_ulSystemClock = ROM_SysCtlClockGet();

    // Initializes the subsystem of measurement of the CPU usage (it measures the time that the CPU is not asleep).
    // For that it uses a timer, that here we have put that it is the TIMER0 (last parameter that is passed to the function)
    // (and therefore this one should not be used for another thing).
    CPUUsageInit(g_ulSystemClock, configTICK_RATE_HZ/10, 0);

    init_CanDevice(GPIO_PORTB_BASE, CAN0_BASE, GPIO_PIN_5, GPIO_PIN_4, BIT_RATE, true);
    init_graphicInterface();
    init_Buttons();

    drawECUMenu();

    // Flag events creation
    init_flagEvents();

    // Task creation
    init_deviceTasks();
    init_buttonTasks();

    ROM_IntMasterEnable();
    // Start the scheduler. The tasks that have been activated are executed.
    vTaskStartScheduler();

    while(1){
        // Something was wrong.
    }

}
