/*
    FreeRTOS V8.2.0 - Copyright (C) 2015 Real Time Engineers Ltd.


    ***************************************************************************
     *                                                                       *
     *    FreeRTOS tutorial books are available in pdf and paperback.        *
     *    Complete, revised, and edited pdf reference manuals are also       *
     *    available.                                                         *
     *                                                                       *
     *    Purchasing FreeRTOS documentation will not only help you, by       *
     *    ensuring you get running as quickly as possible and with an        *
     *    in-depth knowledge of how to use FreeRTOS, it will also help       *
     *    the FreeRTOS project to continue with its mission of providing     *
     *    professional grade, cross platform, de facto standard solutions    *
     *    for microcontrollers - completely free of charge!                  *
     *                                                                       *
     *    >>> See http://www.FreeRTOS.org/Documentation for details. <<<     *
     *                                                                       *
     *    Thank you for using FreeRTOS, and thank you for your support!      *
     *                                                                       *
    ***************************************************************************


    This file is part of the FreeRTOS distribution.

    FreeRTOS is free software; you can redistribute it and/or modify it under
    the terms of the GNU General Public License (version 2) as published by the
    Free Software Foundation AND MODIFIED BY the FreeRTOS exception.
    >>>NOTE<<< The modification to the GPL is included to allow you to
    distribute a combined work that includes FreeRTOS without being obliged to
    provide the source code for proprietary components outside of the FreeRTOS
    kernel.  FreeRTOS is distributed in the hope that it will be useful, but
    WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
    or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
    more details. You should have received a copy of the GNU General Public
    License and the FreeRTOS license exception along with FreeRTOS; if not it
    can be viewed here: http://www.freertos.org/a00114.html and also obtained
    by writing to Richard Barry, contact details for whom are available on the
    FreeRTOS WEB site.

    1 tab == 4 spaces!

    http://www.FreeRTOS.org - Documentation, latest information, license and
    contact details.

    http://www.SafeRTOS.com - A version that is certified for use in safety
    critical systems.

    http://www.OpenRTOS.com - Commercial support, development, porting,
    licensing and training services.
*/

#ifndef FREERTOS_CONFIG_H
#define FREERTOS_CONFIG_H

#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_ints.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"

/*-----------------------------------------------------------
 * Application specific definitions.
 *
 * These definitions should be adjusted for your particular hardware and
 * application requirements.
 *
 * THESE PARAMETERS ARE DESCRIBED WITHIN THE 'CONFIGURATION' SECTION OF THE
 * FreeRTOS API DOCUMENTATION AVAILABLE ON THE FreeRTOS.org WEB SITE.
 *
 * See http://www.freertos.org/a00110.html.
 *----------------------------------------------------------*/

#define configUSE_PREEMPTION                1 // 1/0: Indica si FreeRTOS hará uso de una planificación expropiativa
#define configUSE_IDLE_HOOK                 0 // 1/0: Indica si la tarea IDLE llamará a una función de usuario al ejecutarse
#define configUSE_TICK_HOOK                 1 // 1/0: Indica si se ejecutará una función de usuario asociada al TICK del sistema
#define configMAX_PRIORITIES                ( 16 )  // Número de prioridades para las tareas
#define configCPU_CLOCK_HZ                  ( ( unsigned long ) MAP_SysCtlClockGet() ) // Frecuencia de reloj del sistema (debe coincidir con SyCtlClockSet)
#define configTICK_RATE_HZ                  ( ( portTickType ) 1000 ) // Número de TICKS por segundo --> precision del SO
#define configMINIMAL_STACK_SIZE            ( ( unsigned short ) 128 )   // Tamaño de la pila de la tarea IDLE en objetos (ARM:1objeto->32bits)
#define configTOTAL_HEAP_SIZE               ( ( size_t ) ( 20 * 1024 ) ) // Memoria dinámica reservada a FreeRTOS (en bytes)
#define configMAX_TASK_NAME_LEN             ( 12 ) // Longitud máxima de los nombres dados a las tareas
#define configUSE_TRACE_FACILITY            1 // 1/0: Activación del modo traza
#define configUSE_16_BIT_TICKS              0 // 1/0: Tamaño de la variable de cuenta de ticks (1:16bits, 0:32bits)
#define configIDLE_SHOULD_YIELD             1 // 1/0: Si 1, la tarea IDLE SIEMPRE cede el control a otra tarea, aunque tenga tambien prioridad 0
#define configUSE_MUTEXES                   1 // 1/0: Indica si se van a usar MUTEX en la aplicación
#define configUSE_RECURSIVE_MUTEXES         0 // 1/0: Indica si se van a usar MUTEX recursivos en la aplicación
#define configUSE_COUNTING_SEMAPHORES       0 // 1/0: Indica si se van a usar semaforos contadores en la aplicación
#define configUSE_MALLOC_FAILED_HOOK	1 // 1/0: Indica si se ejecutará una función de usuario en caso de fallo de memoria dinámica
#define configUSE_APPLICATION_TASK_TAG	0 // 1/0: Activa el modo TAG de las tareas (relacionado con la depuración)
#define configGENERATE_RUN_TIME_STATS	1 // 1/0: Activa la recogida de estadísticas (relacionado con la depuración)
#define configUSE_TICKLESS_IDLE		0  // 1/0: Desactiva la ejecución de la tarea IDLE si el sistema se suspende durante un tiempo
									   // hasta que el sistema vuelva a reactivarse
#define configSUPPORT_DYNAMIC_ALLOCATION 1

/* Constants provided for debugging and optimisation assistance. */
// 0/1/2: Activa alguno de los mecanismos de chequeo de desbordamiento en pila de tareas
#define configCHECK_FOR_STACK_OVERFLOW          (2)
#define configASSERT( x ) if( ( x ) == 0 ) { taskDISABLE_INTERRUPTS(); for( ;; ); }
#define configQUEUE_REGISTRY_SIZE               0 // Define el máximo número de colas y semáforos que se pueden registrar en el sistema de depuracion

#define configUSE_CO_ROUTINES               0 // 1/0: Activa el uso de mecanismos de corrutinas
#define configMAX_CO_ROUTINE_PRIORITIES     ( 2 ) // Indica el número de prioridades que podrán tener las tareas que funcionen como corrutinas

/* Software timer definitions. */
#define configUSE_TIMERS				1 // 1/0: Activa el uso de timers SW (basados en ticks)
#define configTIMER_TASK_PRIORITY		( 2 ) // Fija la prioridad de la tareas interna que actualiza los timer SW
#define configTIMER_QUEUE_LENGTH		32 // Tamaño de la cola de comandos de control de Timers SW
#define configTIMER_TASK_STACK_DEPTH	( configMINIMAL_STACK_SIZE * 2 ) // Tamaño de la pila de la tarea interna que gestiona los timer SW


#define configUSE_STATS_FORMATTING_FUNCTIONS 1 // 1/0: Formateo de parámetros estadísticos recogidos en depuración

//Esto es para las estadisticas
#if configGENERATE_RUN_TIME_STATS
#ifndef __ASM_HEADER__ /* elimina un monton de warnings */
#include"utils/RunTimeStatsConfig.h"
#define portCONFIGURE_TIMER_FOR_RUN_TIME_STATS()            vConfigureTimerForRunTimeStats()
#define portGET_RUN_TIME_COUNTER_VALUE()	(GetOverflowCounts())
#endif
#endif


/* Set the following definitions to 1 to include the API function, or zero
to exclude the API function. */

#define INCLUDE_pcTaskGetTaskName 			1
#define INCLUDE_vTaskPrioritySet            1
#define INCLUDE_uxTaskPriorityGet           1
#define INCLUDE_vTaskDelete                 1
#define INCLUDE_vTaskCleanUpResources       0
#define INCLUDE_vTaskSuspend                1
#define INCLUDE_vTaskDelayUntil             1
#define INCLUDE_vTaskDelay                  1
#define INCLUDE_uxTaskGetStackHighWaterMark 1
#define INCLUDE_xEventGroupSetBitFromISR    1

//La definicion de INCLUDE_xTimerPendFunctionCallFromISR es necesaria para que funcionen
//Los flags de eventos. Ademas la utilizamos para lanzar codigo que se ejecute a nivel de tarea desde una ISR
#define INCLUDE_xTimerPendFunctionCallFromISR          1
//Esta de abajo por lo visto es necesaria para que funcionen los eventos ??
#define INCLUDE_xTimerPendFunctionCall			1

//Chequeo interno de FreeRTOS...
#ifdef ccs
   void __error__(char *pcFilename, uint32_t ulLine);
    #undef configASSERT
    #define configASSERT(expr) if ( (expr) == 0)   {                                                    \
                                                        __error__( __FILE__ , __LINE__ ); \
                                                    }
#endif


/* Use the system definition, if there is one. */
#ifdef __NVIC_PRIO_BITS
    #define configPRIO_BITS       __NVIC_PRIO_BITS
#else
    #define configPRIO_BITS       3     /* 8 priority levels */
#endif

/* Be ENORMOUSLY careful if you want to modify these values and make sure
 * you read http://www.freertos.org/a00110.html#kernel_priority first!
 */

/* The lowest interrupt priority that can be used in a call to a "set priority"
function. */
#define configLIBRARY_LOWEST_INTERRUPT_PRIORITY         0x07

/* The highest interrupt priority that can be used by any interrupt service
routine that makes calls to interrupt safe FreeRTOS API functions.  DO NOT CALL
INTERRUPT SAFE FREERTOS API FUNCTIONS FROM ANY INTERRUPT THAT HAS A HIGHER
PRIORITY THAN THIS! (higher priorities are lower numeric values. */
#define configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY    4

/* Interrupt priorities used by the kernel port layer itself.  These are generic
to all Cortex-M ports, and do not rely on any particular library functions. */
#define configKERNEL_INTERRUPT_PRIORITY         ( configLIBRARY_LOWEST_INTERRUPT_PRIORITY << (8 - configPRIO_BITS) )
/* !!!! configMAX_SYSCALL_INTERRUPT_PRIORITY must not be set to zero !!!!
See http://www.FreeRTOS.org/RTOS-Cortex-M3-M4.html. */
#define configMAX_SYSCALL_INTERRUPT_PRIORITY    ( configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY << (8 - configPRIO_BITS) )


#endif /* FREERTOS_CONFIG_H */
