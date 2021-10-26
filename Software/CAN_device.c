/*
 * CAN_device.c
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
//#include "inc/tm4c123gh6pm.h"
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
#include "utils/cpu_usage.h"
#include "event_groups.h"

// Programmer libraries
#include "CAN_device.h"
#include "Graphic_interface.h"
#include "ST7735.h"
#include "Buttons.h"
//#include "sdcard.h"


// Globals variables
static TaskHandle_t DTC_taskHandler = NULL;
static TaskHandle_t Main_taskHandler = NULL;
static TaskHandle_t Live_data_taskHandler = NULL;
static TaskHandle_t Erase_DTCs_taskHandler = NULL;
static TaskHandle_t Get_VIN_taskHandler = NULL;
static TaskHandle_t Freeze_frame_taskHandler = NULL;
EventGroupHandle_t flagEvents;
static tCANMsgObject CANTxMessage, CANRxMessage, CANLiveData;


extern uint16_t menu_cursor, menu_ECU_cursor, menu_showed;
static bool time_expired;
bool live_all_data_mode = false;
extern bool left_button_state, menu_button_state, OnMenu;
uint32_t ECU_ID_Response, ECU_ID_Request;

//*****************************************************************************
//
// A counter that keeps track of the number of times the TX and RX interrupt has
// occurred, which should match the number of TX and RX messages that were sent.
//
//*****************************************************************************
volatile uint32_t g_ui32TXMsgCount = 0;

volatile uint32_t g_ui32RXMsgCount = 0;

// A flag for the interrupt handler to indicate that a message was received.
//
//*****************************************************************************
volatile bool g_bRXFlag = 0;

//*****************************************************************************
//
// A flag to indicate that some transmission error occurred.
//
//*****************************************************************************
volatile bool g_ui32ErrFlag = 0;

//*****************************************************************************

//*****************************************************************************
//
// This function is the interrupt handler for the CAN peripheral.  It checks
// for the cause of the interrupt, and maintains a count of all messages that
// have been transmitted.
//
//*****************************************************************************
void CANIntHandler(void){

    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    uint32_t ui32Status;

       // Read the CAN interrupt status to find the cause of the interrupt
       //
       // CAN_INT_STS_CAUSE register values
       // 0x0000        = No Interrupt Pending
       // 0x0001-0x0020 = Number of message object that caused the interrupt
       // 0x8000        = Status interrupt
       // all other numbers are reserved and have no meaning in this system
       ui32Status = CANIntStatus(CAN0_BASE, CAN_INT_STS_CAUSE);


       // If this was a status interrupt acknowledge it by reading the CAN
       // controller status register.
       if(ui32Status & CAN_INT_INTID_STATUS){
           // Read the controller status.  This will return a field of status
           // error bits that can indicate various errors. Refer to the
           // API documentation for details about the error status bits.
           // The act of reading this status will clear the interrupt.
           ui32Status = CANStatusGet(CAN0_BASE, CAN_STS_CONTROL);

           // Add ERROR flags to list of current errors. To be handled
          // later, because it would take too much time here in the interrupt.
           g_ui32ErrFlag |= ui32Status;

       } else if(ui32Status & RXOBJECT){
           // Getting to this point means that the RX interrupt occurred on
           // message object RXOBJECT, and the message reception is complete.
           // Clear the message object interrupt.
           CANIntClear(CAN0_BASE, RXOBJECT);

           // Increment a counter to keep track of how many messages have been
           // received.  In a real application this could be used to set flags to
           // indicate when a message is received.
           g_ui32RXMsgCount++;

           // Set flag to indicate received message is pending.
           //g_bRXFlag = true;
           // If CAN interruption has lower priority than configMAX_SYSCALL_INTERRUPT_PRIORITY, flag events doesn't work fromISR.
           xEventGroupSetBitsFromISR(flagEvents, CAN_RX_INTERRUPT, &xHigherPriorityTaskWoken);

           // Since a message was received, clear any error flags.
           // This is done because before the message is received it triggers
           // a Status Interrupt for RX complete. by clearing the flag here we
           // prevent unnecessary error handling from happening
           g_ui32ErrFlag = 0;
        } else if(ui32Status & TXOBJECT){
           // Getting to this point means that the TX interrupt occurred on
           // message object 1, and the message TX is complete.  Clear the
           // message object interrupt.
           CANIntClear(CAN0_BASE, TXOBJECT);

           xEventGroupSetBitsFromISR(flagEvents, CAN_TX_INTERRUPT, &xHigherPriorityTaskWoken);
           // Increment a counter to keep track of how many messages have been
           // sent.  In a real application this could be used to set flags to
           // indicate when a message is sent.
           g_ui32TXMsgCount++;


           // Since the message was sent, clear any error flags.
           g_ui32ErrFlag = 0;
           //drawString("Successful transmission...", 20, 70, ST7735_WHITE, ST7735_BLACK, 1);
       } else{
           //xEventGroupSetBitsFromISR(flagEvents, CAN_ERROR_INTERRUPT, &xHigherPriorityTaskWoken);
       }

       //xEventGroupSetBitsFromISR(flagEvents, CAN_STATUS, &xHigherPriorityTaskWoken);

       portYIELD_FROM_ISR(xHigherPriorityTaskWoken);

}


// Device Tasks
static portTASK_FUNCTION(Main_task, pvParameters){

    EventBits_t bitsReaded;

    while(1){

        bitsReaded = xEventGroupWaitBits(flagEvents, SELECT_CAN_COMMAND|SELECT_ECU_ADDRESS, pdTRUE, pdFALSE, portMAX_DELAY);
        if (bitsReaded & SELECT_CAN_COMMAND){
            OnMenu = false;
            switch(menu_cursor){

                case 0:
                    xEventGroupSetBits(flagEvents, VEHICLE_INFORMATION);
                    break;

                case 1:
                    xEventGroupSetBits(flagEvents, READ_DTC);
                    break;

                case 2:
                    xEventGroupSetBits(flagEvents, ERASE_DTC);
                    break;

                case 3:
                    xEventGroupSetBits(flagEvents, FREEZE_FRAME);
                    break;

                case 4:
                    xEventGroupSetBits(flagEvents, LIVE_ALL_DATA);
                    break;

                case 5:
                    xEventGroupSetBits(flagEvents, READ_DTC_DRIVING_CYCLE);
                    break;
            }
        }else if (bitsReaded & SELECT_ECU_ADDRESS){

                switch(menu_ECU_cursor){
                case 0:
                    ECU_ID_Response = ECM_RESPONSE;
                    ECU_ID_Request = ECM_REQUEST;
                    break;

                case 1:
                    ECU_ID_Response = TCM_RESPONSE;
                    ECU_ID_Request = TCM_REQUEST;
                    break;

                case 2:
                    ECU_ID_Response = ABS_RESPONSE;
                    ECU_ID_Request = ABS_REQUEST;
                    break;
                }
                menu_showed = MENU_MODE;
                OnMenu = true;
                menu_cursor = 0;
                cleanScreen();
                drawMenu();
        }

        xEventGroupClearBits(flagEvents, SELECT_CAN_COMMAND|SELECT_ECU_ADDRESS);
    }

}


static portTASK_FUNCTION(Read_DTC, pvParameters){

    uint8_t request_data_frame[MAX_BYTES];
    uint8_t response_data_frame[MAX_BYTES];
    EventBits_t bitsReaded;

    while(1){

        bitsReaded = xEventGroupWaitBits(flagEvents, READ_DTC_DRIVING_CYCLE | READ_DTC, pdTRUE, pdFALSE, portMAX_DELAY);


        request_data_frame[0] = 0x01;
        if (bitsReaded & READ_DTC_DRIVING_CYCLE){

            request_data_frame[1] = 0x07;
        }else {

            request_data_frame[1] = 0x03;
        }
        for (int i = 2; i < MAX_BYTES; i++){

            request_data_frame[i] = 0x55;
        }

        // Reception message object
        // Initialize a message object to be used for receiving CAN messages with
        // any CAN ID.  In order to receive any CAN ID, the ID and mask must both
        // be set to 0, and the ID filter enabled.
        CANRxMessage.pui8MsgData = (uint8_t *) &response_data_frame;
        CANRxMessage.ui32MsgID = ECU_ID_Response;
        CANRxMessage.ui32MsgIDMask = MASK_RESPONSE_ID;
        CANRxMessage.ui32Flags = MSG_OBJ_RX_INT_ENABLE | MSG_OBJ_USE_ID_FILTER;
        CANRxMessage.ui32MsgLen = 8; // 8 bytes

        // Now load the message object into the CAN peripheral.  Once loaded the
        // CAN will receive any message on the bus, and an interrupt will occur.
        // Use message object RXOBJECT for receiving messages (this is not the
        // same as the CAN ID which can be any value in this example).
        CANMessageSet(CAN0_BASE, RXOBJECT, &CANRxMessage, MSG_OBJ_TYPE_RX);


        // Transmission message object
        CANTxMessage.pui8MsgData = request_data_frame;
        CANTxMessage.ui32MsgLen = 8;
        CANTxMessage.ui32Flags = MSG_OBJ_TX_INT_ENABLE;
        CANTxMessage.ui32MsgIDMask = MASK_RESPONSE_ID;
        CANTxMessage.ui32MsgID = REMOTE_REQUEST_ID;

        CANMessageSet(CAN0_BASE, TXOBJECT, &CANTxMessage, MSG_OBJ_TYPE_TX);


        bitsReaded = xEventGroupWaitBits(flagEvents, CAN_RX_INTERRUPT, pdTRUE, pdFALSE, portMAX_DELAY);

        //if (bitsReaded & (CAN_RX_INTERRUPT|CAN_TX_INTERRUPT)){
        if (bitsReaded & CAN_RX_INTERRUPT){

            char *CAN_frame = NULL;
            char *CAN_FirstFrame = NULL;
            char *num_DTCs_Hex = NULL;
            char decoded_DTC_buffer[NUM_CHAR_DTC+1];
            char input_DTC_buffer[5];
            uint16_t numDTCs;

            menu_button_state = false;
            left_button_state = false;

            // Read the message from the CAN.  Message object RXOBJECT is used
            // (which is not the same thing as CAN ID).  The interrupt clearing
            // flag is not set because this interrupt was already cleared in
            // the interrupt handler.
            time_expired = false;
            config_systemPauseTimer(1/100);
            system_pause();
            while(!time_expired);
            CANMessageGet(CAN0_BASE, RXOBJECT, &CANRxMessage, 0);

            // Clear the pending message flag so that the interrupt handler can
            // set it again when the next message arrives.
            cleanScreen();
            get_CANframe(&CAN_frame, (char*)&response_data_frame, true, false);

            num_DTCs_Hex = (char*)pvPortMalloc(3*sizeof(char));
            memcpy(num_DTCs_Hex, CAN_frame, 3);
            num_DTCs_Hex[2] = '\0';
            numDTCs = hex2Decimal(num_DTCs_Hex);

            vPortFree(CAN_frame);
            CAN_frame = NULL;

            if (numDTCs > 0){

                cleanScreen();
                int offset_DTC = 2;
                get_CANframe(&CAN_FirstFrame, (char*)&response_data_frame, false, false);
                if (is_Multiframe(CAN_FirstFrame)){

                    offset_DTC = 2;
                    for (int i = 0; i < 2; i++){

                        memcpy(input_DTC_buffer, CAN_FirstFrame+offset_DTC, 4);
                        input_DTC_buffer[4] = '\0';
                        get_DTC_decoded(input_DTC_buffer, decoded_DTC_buffer);
                        showDTC(decoded_DTC_buffer, i*20);
                        offset_DTC = offset_DTC + 4;
                    }

                    uint16_t contFrames = 0;
                    int16_t numOfFrames = get_numberOfConsecutiveFrameToReceive(CAN_FirstFrame);

                    CANMessageSet(CAN0_BASE, RXOBJECT, &CANRxMessage, MSG_OBJ_TYPE_RX);

                    request_data_frame[0] = 0x30;
                    for (int i = 1; i < MAX_BYTES; i++){

                         request_data_frame[i] = 0x00;
                    }
                    CANTxMessage.pui8MsgData = request_data_frame;
                    CANTxMessage.ui32MsgLen = 8;
                    CANTxMessage.ui32MsgIDMask = MASK_RESPONSE_ID;
                    CANTxMessage.ui32MsgID = ECU_ID_Request;

                    // Wait 100ms to send the Flow Control Frame to ECU. TODO: Test it with least time
                    time_expired = false;
                    config_systemPauseTimer(1/100);
                    system_pause();
                    while(!time_expired);

                    CANMessageSet(CAN0_BASE, TXOBJECT, &CANTxMessage, MSG_OBJ_TYPE_TX);

                    int numDTCs_restantes = numDTCs;
                    while(contFrames < numOfFrames){

                        bitsReaded = xEventGroupWaitBits(flagEvents, CAN_RX_INTERRUPT, pdTRUE, pdFALSE, portMAX_DELAY);
                        if (bitsReaded & CAN_RX_INTERRUPT){
                            // Read the message from the CAN.  Message object RXOBJECT is used
                            // (which is not the same thing as CAN ID).  The interrupt clearing
                            // flag is not set because this interrupt was already cleared in
                            // the interrupt handler.
                            CANMessageGet(CAN0_BASE, RXOBJECT, &CANRxMessage, 0);

                            get_CANframe(&CAN_frame, (char*)&response_data_frame, true, false);
                            if (offset_DTC != 14){
                                offset_DTC = 2;
                                numDTCs_restantes = numDTCs - 2;
                                for (int i = 0; i < numDTCs_restantes; i++){

                                    if (offset_DTC != 14){
                                        memcpy(input_DTC_buffer, CAN_frame+offset_DTC, 4);
                                        input_DTC_buffer[4] = '\0';
                                        get_DTC_decoded(input_DTC_buffer, decoded_DTC_buffer);
                                        showDTC(decoded_DTC_buffer, 40+20*i);
                                        offset_DTC = offset_DTC + 4;
                                    }else {

                                        i = numDTCs_restantes;
                                        memcpy(input_DTC_buffer, CAN_frame+offset_DTC, 2);
                                    }
                                }
                            }else {
                                // save the last byte from the last DTC
                                offset_DTC = 2;
                                memcpy(input_DTC_buffer+offset_DTC, CAN_frame+offset_DTC, 4);
                                input_DTC_buffer[4] = '\0';
                                get_DTC_decoded(input_DTC_buffer, decoded_DTC_buffer);
                                showDTC(decoded_DTC_buffer, 105);
                            }
                            contFrames++;
                        }
                        vPortFree(CAN_frame);
                        CAN_frame = NULL;
                    }

                }else {


                    for (int i = 0; i < numDTCs; i++){

                         memcpy(input_DTC_buffer, CAN_FirstFrame+offset_DTC, 4);
                         input_DTC_buffer[4] = '\0';
                         get_DTC_decoded(input_DTC_buffer, decoded_DTC_buffer);
                         showDTC(decoded_DTC_buffer, i*20);
                         get_CANframe(&CAN_FirstFrame, (char*)&response_data_frame, true, false);
                         offset_DTC = offset_DTC + 4;
                     }

                }

            }else {

                drawString(20, 55, "0 DTCs stored", MENU_DATA_TEXT_COLOUR, ST7735_BLACK, 1, 20);
            }

            vPortFree(CAN_frame);
            vPortFree(num_DTCs_Hex);
            vPortFree(CAN_FirstFrame);
            while((!menu_button_state) && (!left_button_state));
            cleanScreen();
            menu_cursor = 0;
            OnMenu = true;
            menu_showed = MENU_MODE;
            drawMenu();
        }

    }

}

static portTASK_FUNCTION(Live_all_data, pvParameters){

    double realTime_values[NUM_LIVE_DATA_PIDS];
    char *CAN_data = NULL;
    uint8_t requestFrame_liveData[MAX_BYTES];
    uint8_t response_data_frame[MAX_BYTES];
    EventBits_t bitsReaded;
    uint8_t cont = 0;
    uint8_t posPID;
    char *pids_freezeData = NULL;
    uint16_t numPIDs_supported = 0;


    // The same for all tx frames
    requestFrame_liveData[0] = 0x02;
    requestFrame_liveData[1] = 0x01;
    for (int i = 3; i < MAX_BYTES; i++){

        requestFrame_liveData[i] = 0xA5;
    }

    // Transmission message object
    CANLiveData.pui8MsgData = (uint8_t *) &requestFrame_liveData;
    // Same for all frames
    CANLiveData.ui32MsgLen = 3;
    CANLiveData.ui32Flags = MSG_OBJ_TX_INT_ENABLE;
    CANLiveData.ui32MsgIDMask = 0;
    CANLiveData.ui32MsgID = REMOTE_REQUEST_ID;

    while(1){

        xEventGroupWaitBits(flagEvents, LIVE_ALL_DATA, pdTRUE, pdFALSE, portMAX_DELAY);

        live_all_data_mode = true;
        // Reception message object
        // Initialize a message object to be used for receiving CAN messages with
        // any CAN ID.  In order to receive any CAN ID, the ID and mask must both
        // be set to 0, and the ID filter enabled.
        /*CANRxMessage.pui8MsgData = (uint8_t *) &response_data_frame;
        CANRxMessage.ui32MsgID = ECU_ID_Response;
        CANRxMessage.ui32MsgIDMask = MASK_RESPONSE_ID;
        CANRxMessage.ui32Flags = MSG_OBJ_RX_INT_ENABLE | MSG_OBJ_USE_ID_FILTER;
        CANRxMessage.ui32MsgLen = 8; // 8 bytes

        // Now load the message object into the CAN peripheral.  Once loaded the
        // CAN will receive any message on the bus, and an interrupt will occur.
        // Use message object RXOBJECT for receiving messages (this is not the
        // same as the CAN ID which can be any value in this example).
        CANMessageSet(CAN0_BASE, RXOBJECT, &CANRxMessage, MSG_OBJ_TYPE_RX);*/

        cleanScreen();
        cont = 0;
        left_button_state = false;
        menu_button_state = false;
        GPIOIntDisable(BUTTONS_PORT_BASE, RIGHT_BUTTON | DOWN_BUTTON | UP_BUTTON | OK_BUTTON);

        if (ECU_ID_Response != ECM_RESPONSE){

            request_PIDs_supportedOnMode01(&pids_freezeData);
            // Substract 0x1C PID because its decoding does not implemented
            numPIDs_supported = sizeOfFrame(pids_freezeData)-1;

        }else {
            // Not all PIDs decoding are implemented for ECM
            numPIDs_supported = NUM_LIVE_DATA_PIDS;
        }

        CANRxMessage.pui8MsgData = (uint8_t *) &response_data_frame;
        CANRxMessage.ui32MsgID = ECU_ID_Response;
        CANRxMessage.ui32MsgIDMask = MASK_RESPONSE_ID;
        CANRxMessage.ui32Flags = MSG_OBJ_RX_INT_ENABLE | MSG_OBJ_USE_ID_FILTER;
        CANRxMessage.ui32MsgLen = 8; // 8 bytes

        CANMessageSet(CAN0_BASE, RXOBJECT, &CANRxMessage, MSG_OBJ_TYPE_RX);

        // Left button to skip
        while ((!left_button_state) && (!menu_button_state)){

            if  (ECU_ID_Response != ECM_RESPONSE){

                requestFrame_liveData[2] = pids_freezeData[cont];

            }else {

                requestFrame_liveData[2] = pids_liveData[cont];
            }
            CANMessageSet(CAN0_BASE, TXOBJECT, &CANLiveData, MSG_OBJ_TYPE_TX);

            bitsReaded = xEventGroupWaitBits(flagEvents, CAN_ERROR_INTERRUPT|CAN_TX_INTERRUPT, pdTRUE, pdFALSE, portMAX_DELAY);
            if(bitsReaded & CAN_ERROR_INTERRUPT){

                cleanScreen();
                check_CANerrors();
            }

            bitsReaded = xEventGroupWaitBits(flagEvents, CAN_RX_INTERRUPT, pdTRUE, pdFALSE, MAX_TIME_TO_WAIT_MS);
            if (bitsReaded & CAN_RX_INTERRUPT){

                uint16_t dataA = 0;
                uint16_t dataB = 0;

                // Read the message from the CAN.  Message object RXOBJECT is used
                // (which is not the same thing as CAN ID).  The interrupt clearing
                // flag is not set because this interrupt was already cleared in
                // the interrupt handler.
                CANMessageGet(CAN0_BASE, RXOBJECT, &CANRxMessage, 0);

                get_CANframe(&CAN_data, (char*)response_data_frame, true, false);
                // Return the size of the frame in hex format (1 byte, 2 positions)
                // if there are 2 data bytes(>= 4 positions)...
                if (sizeOfFrame(CAN_data) >= 4){
                    char CAN_frame_1[3];
                    CAN_frame_1[0] = CAN_data[0];
                    CAN_frame_1[1] = CAN_data[1];
                    CAN_frame_1[2] ='\0';

                    char CAN_frame_2[3];
                    CAN_frame_2[0] = CAN_data[2];
                    CAN_frame_2[1] = CAN_data[3];
                    CAN_frame_2[2] ='\0';

                    dataA = hex2Decimal(CAN_frame_1);
                    dataB = hex2Decimal(CAN_frame_2);


                } else{

                    dataA = hex2Decimal(CAN_data);
                }
                vPortFree(CAN_data);
                CAN_data = NULL;
                posPID = get_posPID((char*)response_data_frame);
                realTime_values[posPID] = decode_CANdata(posPID, (double)dataA, (double)dataB);
            }
            cleanData(cont);
            show_liveData(realTime_values[posPID], posPID);
            cont++;
            if (cont == numPIDs_supported){

                cont = 0;
            }

        }
        // Back to menu
        live_all_data_mode = false;
        cleanScreen();
        OnMenu = true;
        menu_showed = MENU_MODE;
        drawMenu();
        GPIOIntEnable(BUTTONS_PORT_BASE, RIGHT_BUTTON | DOWN_BUTTON | UP_BUTTON | OK_BUTTON);

    }

}

static portTASK_FUNCTION(Erase_DTCs, pvParameters){

    uint8_t request_data_frame[MAX_BYTES];
    uint8_t response_data_frame[MAX_BYTES];
    EventBits_t bitsReaded;


    while(1){

        xEventGroupWaitBits(flagEvents, ERASE_DTC, pdTRUE, pdFALSE, portMAX_DELAY);

        time_expired = false;

        // Reception message object
        // Initialize a message object to be used for receiving CAN messages with
        // any CAN ID.  In order to receive any CAN ID, the ID and mask must both
        // be set to 0, and the ID filter enabled.
        CANRxMessage.pui8MsgData = response_data_frame;
        CANRxMessage.ui32MsgID = ECU_ID_Response;
        CANRxMessage.ui32MsgIDMask = MASK_RESPONSE_ID;
        CANRxMessage.ui32Flags = MSG_OBJ_RX_INT_ENABLE | MSG_OBJ_USE_ID_FILTER;
        CANRxMessage.ui32MsgLen = 8; // 8 bytes

        // Now load the message object into the CAN peripheral.  Once loaded the
        // CAN will receive any message on the bus, and an interrupt will occur.
        // Use message object RXOBJECT for receiving messages (this is not the
        // same as the CAN ID which can be any value in this example).
        CANMessageSet(CAN0_BASE, RXOBJECT, &CANRxMessage, MSG_OBJ_TYPE_RX);

        request_data_frame[0] = 0x01;
        request_data_frame[1] = 0x04;
        for (int i = 2; i < MAX_BYTES; i++){

            request_data_frame[i] = 0x55;
        }

        // Transmission message object
        CANTxMessage.pui8MsgData = request_data_frame;
        CANTxMessage.ui32MsgLen = 2;
        CANTxMessage.ui32Flags = MSG_OBJ_TX_INT_ENABLE;
        CANTxMessage.ui32MsgIDMask = 0;
        CANTxMessage.ui32MsgID = REMOTE_REQUEST_ID;

        CANMessageSet(CAN0_BASE, TXOBJECT, &CANTxMessage, MSG_OBJ_TYPE_TX);

        bitsReaded = xEventGroupWaitBits(flagEvents, CAN_ERROR_INTERRUPT|CAN_TX_INTERRUPT, pdTRUE, pdFALSE, portMAX_DELAY);
        if(bitsReaded & CAN_TX_INTERRUPT){

           cleanScreen();
           //drawString(35, 50, "\nCAN message sent\n", ST7735_WHITE, ST7735_BLACK, 1, 0);
       } else if (bitsReaded & CAN_ERROR_INTERRUPT){

           cleanScreen();
           check_CANerrors();
       }

        bitsReaded = xEventGroupWaitBits(flagEvents, CAN_RX_INTERRUPT, pdTRUE, pdFALSE, MAX_TIME_TO_WAIT_MS);
        if (bitsReaded & CAN_RX_INTERRUPT){

            char *CAN_frame = NULL;

            // Read the message from the CAN.  Message object RXOBJECT is used
            // (which is not the same thing as CAN ID).  The interrupt clearing
            // flag is not set because this interrupt was already cleared in
            // the interrupt handler.
            CANMessageGet(CAN0_BASE, RXOBJECT, &CANRxMessage, 0);

            get_CANframe(&CAN_frame, (char*)response_data_frame, false, false);
            cleanScreen();
            if ((CAN_frame[2] == '4') && (CAN_frame[3] == '4')){

                drawString(10, 50, "DTCs correctly cleared\n\n     MIL Status: OFF\n", MENU_DATA_TEXT_COLOUR, ST7735_BLACK, 1, 0);
            } else{

                drawString(20, 50, "Error clearing DTCs\n\n        MIL Status: ON\n", MENU_DATA_TEXT_COLOUR, ST7735_BLACK, 1, 0);
            }

            vPortFree(CAN_frame);
            CAN_frame = NULL;

        } else{

            cleanScreen();
            drawString(20, 50, "There is not DTCs\n\n    MIL Status: OFF", MENU_DATA_TEXT_COLOUR, ST7735_BLACK, 1, 0);
        }
        config_systemPauseTimer(4);
        system_pause();
        while(!time_expired);
        cleanScreen();
        menu_cursor = 0;
        menu_showed = MENU_MODE;
        OnMenu = true;
        drawMenu();
    }

}

static portTASK_FUNCTION(Get_VIN, pvParameters){

    uint8_t request_data_frame[MAX_BYTES];
    uint8_t response_data_frame[MAX_BYTES];
    EventBits_t bitsReaded;


    while(1){

        xEventGroupWaitBits(flagEvents, VEHICLE_INFORMATION, pdTRUE, pdFALSE, portMAX_DELAY);

        if (ECU_ID_Response == ECM_RESPONSE){

            time_expired = false;

            // Reception message object
            // Initialize a message object to be used for receiving CAN messages with
            // any CAN ID.  In order to receive any CAN ID, the ID and mask must both
            // be set to 0, and the ID filter enabled.
            CANRxMessage.pui8MsgData = response_data_frame;
            CANRxMessage.ui32MsgID = ECU_ID_Response;
            CANRxMessage.ui32MsgIDMask = MASK_RESPONSE_ID;
            CANRxMessage.ui32Flags = MSG_OBJ_RX_INT_ENABLE | MSG_OBJ_USE_ID_FILTER;
            CANRxMessage.ui32MsgLen = 8; // 8 bytes

            // Now load the message object into the CAN peripheral.  Once loaded the
            // CAN will receive any message on the bus, and an interrupt will occur.
            // Use message object RXOBJECT for receiving messages (this is not the
            // same as the CAN ID which can be any value in this example).
            CANMessageSet(CAN0_BASE, RXOBJECT, &CANRxMessage, MSG_OBJ_TYPE_RX);

            request_data_frame[0] = 0x02;
            request_data_frame[1] = 0x09;
            request_data_frame[2] = 0x02;
            for (int i = 3; i < MAX_BYTES; i++){

                request_data_frame[i] = 0x55;
            }


            // Transmission message object
            CANTxMessage.pui8MsgData = request_data_frame;
            CANTxMessage.ui32MsgLen = 3;
            CANTxMessage.ui32Flags = MSG_OBJ_TX_INT_ENABLE;
            CANTxMessage.ui32MsgIDMask = MASK_RESPONSE_ID;
            CANTxMessage.ui32MsgID = REMOTE_REQUEST_ID;

            CANMessageSet(CAN0_BASE, TXOBJECT, &CANTxMessage, MSG_OBJ_TYPE_TX);

            bitsReaded = xEventGroupWaitBits(flagEvents, CAN_ERROR_INTERRUPT|CAN_TX_INTERRUPT, pdTRUE, pdFALSE, portMAX_DELAY);
            if(bitsReaded & CAN_TX_INTERRUPT){

               cleanScreen();
               //drawString(35, 50, "\nCAN message sent\n", ST7735_WHITE, ST7735_BLACK, 1, 0);
           } else if (bitsReaded & CAN_ERROR_INTERRUPT){

               cleanScreen();
               check_CANerrors();
           }

            bitsReaded = xEventGroupWaitBits(flagEvents, CAN_RX_INTERRUPT, pdTRUE, pdFALSE, MAX_TIME_TO_WAIT_MS);
            if (bitsReaded & CAN_RX_INTERRUPT){

                char *CAN_frame = NULL;
                char *CAN_FirstFrame = NULL;
                char VIN[MAX_VIN_BYTES];
                int numBytes_data;

                CANMessageGet(CAN0_BASE, RXOBJECT, &CANRxMessage, 0);

                cleanScreen();

                drawString(70, 50, "VIN", ST7735_WHITE, ST7735_BLACK, 1, 0);

                get_CANframe(&CAN_FirstFrame, (char*)&response_data_frame, false, false);
                if (is_Multiframe(CAN_FirstFrame)){

                    get_CANframe(&CAN_frame, (char*)&response_data_frame, true, true);
                    numBytes_data = sizeOfFrame(CAN_frame);

                    memcpy(VIN, CAN_frame, numBytes_data);
                    VIN[numBytes_data] = '\0';

                    uint16_t contFrames = 0;
                    int16_t numOfFrames = get_numberOfConsecutiveFrameToReceive(CAN_FirstFrame);

                    request_data_frame[0] = 0x30;
                    for (int i = 1; i < MAX_BYTES; i++){

                         request_data_frame[i] = 0x00;
                    }
                    CANTxMessage.pui8MsgData = request_data_frame;
                    CANTxMessage.ui32Flags = MSG_OBJ_NO_FLAGS;
                    CANTxMessage.ui32MsgLen = 8;
                    CANTxMessage.ui32MsgIDMask = MASK_RESPONSE_ID;
                    CANTxMessage.ui32MsgID = ECU_ID_Request;

                    CANMessageSet(CAN0_BASE, TXOBJECT, &CANTxMessage, MSG_OBJ_TYPE_TX);

                    int numBytes_readed = numBytes_data;

                    while(contFrames < numOfFrames){
                        vPortFree(CAN_frame);
                        CAN_frame = NULL;

                        bitsReaded = xEventGroupWaitBits(flagEvents, CAN_RX_INTERRUPT, pdTRUE, pdFALSE, MAX_TIME_TO_WAIT_MS);
                        if (bitsReaded & CAN_RX_INTERRUPT){
                            // Read the message from the CAN.  Message object RXOBJECT is used
                            // (which is not the same thing as CAN ID).  The interrupt clearing
                            // flag is not set because this interrupt was already cleared in
                            // the interrupt handler.
                            CANMessageGet(CAN0_BASE, RXOBJECT, &CANRxMessage, 0);

                            get_CANframe(&CAN_frame, (char*)&response_data_frame, true, true);
                            numBytes_data = sizeOfFrame(CAN_frame);

                            memcpy(VIN+numBytes_readed, CAN_frame, numBytes_data);
                            numBytes_readed += numBytes_data;
                            VIN[numBytes_readed] = '\0';
                        }
                        contFrames++;
                    }

                }else {

                    get_CANframe(&CAN_frame, (char*)&response_data_frame, true, true);
                    numBytes_data = sizeOfFrame(CAN_frame);

                    memcpy(VIN, CAN_frame, numBytes_data);
                    VIN[numBytes_data] = '\0';
                }

                drawString(25, 70, VIN,  ST7735_WHITE, ST7735_BLACK, 1, 0);
                vPortFree(CAN_frame);
                vPortFree(CAN_FirstFrame);
            }
            config_systemPauseTimer(4);
            system_pause();
            time_expired = false;
            while(!time_expired);
            cleanScreen();
            menu_cursor = 0;
            OnMenu = true;
            menu_showed = MENU_MODE;
            drawMenu();

        }else {

            cleanScreen();
            drawString(25, 60, "Mode not implemented           on this ECU",  MENU_DATA_TEXT_COLOUR, ST7735_BLACK, 1, 0);
            config_systemPauseTimer(3);
            system_pause();
            time_expired = false;
            while(!time_expired);
            cleanScreen();
            menu_cursor = 0;
            OnMenu = true;
            menu_showed = MENU_MODE;
            drawMenu();
        }
    }
}

static portTASK_FUNCTION(Freeze_frame, pvParameters){

    uint8_t request_data_frame[MAX_BYTES];
    uint8_t response_data_frame[MAX_BYTES];
    EventBits_t bitsReaded;
    uint8_t posPID;
    uint16_t numPIDs_supported = 0;
    bool decoded;

    while(1){

        xEventGroupWaitBits(flagEvents, FREEZE_FRAME, pdTRUE, pdFALSE, portMAX_DELAY);

        if (ECU_ID_Response == ECM_RESPONSE){

            char *CAN_data = NULL;
            char *pids_freezeData = NULL;

            uint8_t cont = 0;

            // Reception message object
            // Initialize a message object to be used for receiving CAN messages with
            // any CAN ID.  In order to receive any CAN ID, the ID and mask must both
            // be set to 0, and the ID filter enabled.
            CANRxMessage.pui8MsgData = response_data_frame;
            CANRxMessage.ui32MsgID = ECU_ID_Response;
            CANRxMessage.ui32MsgIDMask = MASK_RESPONSE_ID;
            CANRxMessage.ui32Flags = MSG_OBJ_RX_INT_ENABLE | MSG_OBJ_USE_ID_FILTER;
            CANRxMessage.ui32MsgLen = 8; // 8 bytes

            // Now load the message object into the CAN peripheral.  Once loaded the
            // CAN will receive any message on the bus, and an interrupt will occur.
            // Use message object RXOBJECT for receiving messages (this is not the
            // same as the CAN ID which can be any value in this example).
            CANMessageSet(CAN0_BASE, RXOBJECT, &CANRxMessage, MSG_OBJ_TYPE_RX);

            cleanScreen();

            // First request the DTC that caused required freeze frame data storage
            request_data_frame[0] = 0x03;
            request_data_frame[1] = 0x02;
            request_data_frame[2] = 0x02;
            request_data_frame[3] = 0x00;
            for (int i = 4; i < MAX_BYTES; i++){

                request_data_frame[i] = 0x55;
            }

            // Transmission message object
            CANTxMessage.pui8MsgData = request_data_frame;
            CANTxMessage.ui32MsgLen = 4;
            CANTxMessage.ui32Flags = MSG_OBJ_TX_INT_ENABLE;
            CANTxMessage.ui32MsgIDMask = MASK_RESPONSE_ID;
            CANTxMessage.ui32MsgID = REMOTE_REQUEST_ID;


            CANMessageSet(CAN0_BASE, TXOBJECT, &CANTxMessage, MSG_OBJ_TYPE_TX);

            bitsReaded = xEventGroupWaitBits(flagEvents, CAN_ERROR_INTERRUPT|CAN_TX_INTERRUPT, pdTRUE, pdFALSE, portMAX_DELAY);
            if(bitsReaded & CAN_ERROR_INTERRUPT){

                cleanScreen();
                check_CANerrors();
            }
            bitsReaded = xEventGroupWaitBits(flagEvents, CAN_RX_INTERRUPT, pdTRUE, pdFALSE, MAX_TIME_TO_WAIT_MS);

            if (bitsReaded & CAN_RX_INTERRUPT){

                CANMessageGet(CAN0_BASE, RXOBJECT, &CANRxMessage, 0);
                get_CANframe(&CAN_data, (char*)response_data_frame, false, false);

                int offset_DTC = 8;
                char input_DTC_buffer[5];
                char decoded_DTC_buffer[NUM_CHAR_DTC+1];

                memcpy(input_DTC_buffer, CAN_data+offset_DTC, 4);
                input_DTC_buffer[4] = '\0';
                decoded = get_DTC_decoded(input_DTC_buffer, decoded_DTC_buffer);

                vPortFree(CAN_data);
                CAN_data = NULL;
                xEventGroupClearBits(flagEvents, CAN_RX_INTERRUPT);

                if (decoded){
                    if (valid_DTC(decoded_DTC_buffer)){

                        double data_values[NUM_LIVE_DATA_PIDS];

                        request_PIDs_supportedOnMode02(&pids_freezeData);
                        numPIDs_supported = sizeOfFrame(pids_freezeData);

                        drawString(10, 50, decoded_DTC_buffer,  ST7735_WHITE, ST7735_BLACK, 1, 5);
                        drawString(45, 50, "is the DTC that \n caused required freeze \n  frame data storage",  ST7735_WHITE, ST7735_BLACK, 1, 10);

                        config_systemPauseTimer(4);
                        system_pause();
                        time_expired = false;
                        while(!time_expired);
                        cleanScreen();

                        while(cont < numPIDs_supported){

                            CANRxMessage.pui8MsgData = response_data_frame;
                            CANMessageSet(CAN0_BASE, RXOBJECT, &CANRxMessage, MSG_OBJ_TYPE_RX);

                            request_data_frame[0] = 0x03;
                            request_data_frame[1] = 0x02;
                            request_data_frame[2] = pids_freezeData[cont];
                            request_data_frame[3] = 0x00;
                            for (int i = 4; i < MAX_BYTES; i++){

                               request_data_frame[i] = 0x00;
                            }

                            // Transmission message object
                            CANTxMessage.pui8MsgData = request_data_frame;
                            CANTxMessage.ui32MsgLen = 8;
                            CANTxMessage.ui32Flags = MSG_OBJ_TX_INT_ENABLE;
                            CANTxMessage.ui32MsgIDMask = MASK_RESPONSE_ID;
                            CANTxMessage.ui32MsgID = REMOTE_REQUEST_ID;

                            CANMessageSet(CAN0_BASE, TXOBJECT, &CANTxMessage, MSG_OBJ_TYPE_TX);

                            bitsReaded = xEventGroupWaitBits(flagEvents, CAN_RX_INTERRUPT, pdTRUE, pdFALSE, portMAX_DELAY);
                            if (bitsReaded & CAN_RX_INTERRUPT){

                                uint16_t dataA = 0;
                                uint16_t dataB = 0;

                                // Read the message from the CAN.  Message object RXOBJECT is used
                                // (which is not the same thing as CAN ID).  The interrupt clearing
                                // flag is not set because this interrupt was already cleared in
                                // the interrupt handler.
                                CANMessageGet(CAN0_BASE, RXOBJECT, &CANRxMessage, 0);

                                get_CANframe(&CAN_data, (char*)response_data_frame, true, false);
                                // Return the size of the frame in hex format (1 byte, 2 positions)
                                // if there are 2 data bytes(>= 4 positions)...
                                if (sizeOfFrame(CAN_data) >= 4){
                                    char CAN_frame_1[3];
                                    CAN_frame_1[0] = CAN_data[0];
                                    CAN_frame_1[1] = CAN_data[1];
                                    CAN_frame_1[2] ='\0';

                                    char CAN_frame_2[3];
                                    CAN_frame_2[0] = CAN_data[2];
                                    CAN_frame_2[1] = CAN_data[3];
                                    CAN_frame_2[2] ='\0';

                                    dataA = hex2Decimal(CAN_frame_1);
                                    dataB = hex2Decimal(CAN_frame_2);


                                } else{

                                    dataA = hex2Decimal(CAN_data);
                                }
                                vPortFree(CAN_data);
                                CAN_data = NULL;
                                posPID = get_posPID((char*)response_data_frame);
                                data_values[posPID] = decode_CANdata(posPID, (double)dataA, (double)dataB);
                            }
                            show_liveData(data_values[posPID], posPID);
                            cont++;
                        }
                        vPortFree(pids_freezeData);
                        pids_freezeData = NULL;

                    }else{
                        cleanScreen();
                        drawString(20, 50, "No freeze frame data   are stored",  MENU_DATA_TEXT_COLOUR, ST7735_BLACK, 1, 20);
                    }

                } else{
                    cleanScreen();
                    drawString(10, 50, "Error decoding DTCs due to transmission error",  MENU_DATA_TEXT_COLOUR, ST7735_BLACK, 1, 10);
                }
                config_systemPauseTimer(4);
                system_pause();
                time_expired = false;
                while(!time_expired);
                cleanScreen();
                menu_cursor = 0;
                OnMenu = true;
                menu_showed = MENU_MODE;
                drawMenu();

            }else {

                cleanScreen();
                drawString(10, 50, "Error decoding DTCs due to reception error",  MENU_DATA_TEXT_COLOUR, ST7735_BLACK, 1, 10);
            }

        }else {

            cleanScreen();
            drawString(25, 60, "Mode not implemented           on this ECU",  MENU_DATA_TEXT_COLOUR, ST7735_BLACK, 1, 0);
            config_systemPauseTimer(3);
            system_pause();
            time_expired = false;
            while(!time_expired);
            cleanScreen();
            menu_cursor = 0;
            OnMenu = true;
            menu_showed = MENU_MODE;
            drawMenu();
        }
    }

}


void init_flagEvents(void){

    // Create events group
    flagEvents = xEventGroupCreate();
}

void init_deviceTasks(void){


    if ((xTaskCreate(Read_DTC, (portCHAR *)"Read_DTC", 512, NULL,tskIDLE_PRIORITY + 0, &DTC_taskHandler) != pdTRUE)){

            while(1);
    }

    if ((xTaskCreate(Main_task, (portCHAR *)"MAIN_TASK", 512, NULL,tskIDLE_PRIORITY + 0, &Main_taskHandler) != pdTRUE)){

            while(1);
    }

    if ((xTaskCreate(Live_all_data, (portCHAR *)"LIVE_DATA", 512, NULL,tskIDLE_PRIORITY + 0, &Live_data_taskHandler) != pdTRUE)){

            while(1);
    }

    if ((xTaskCreate(Erase_DTCs, (portCHAR *)"ERASE_DTCS", 512, NULL,tskIDLE_PRIORITY + 0, &Erase_DTCs_taskHandler) != pdTRUE)){

            while(1);
    }

    if ((xTaskCreate(Get_VIN, (portCHAR *)"GET_VIN", 512, NULL,tskIDLE_PRIORITY + 0, &Get_VIN_taskHandler) != pdTRUE)){

            while(1);
    }

    if ((xTaskCreate(Freeze_frame, (portCHAR *)"FREEZE_FRAME", 512, NULL,tskIDLE_PRIORITY + 0, &Freeze_frame_taskHandler) != pdTRUE)){

            while(1);
    }

}

uint32_t CAN_macro(uint32_t GPIO_peripheral, uint32_t GPIO_pin) {

    if ((GPIO_pin & GPIO_PIN_4) && (GPIO_peripheral & GPIO_PORTB_BASE))
        return (GPIO_PB4_CAN0RX);

    if ((GPIO_pin & GPIO_PIN_0) && (GPIO_peripheral & GPIO_PORTF_BASE))
        return (GPIO_PF0_CAN0RX);

    if ((GPIO_pin & GPIO_PIN_4) && (GPIO_peripheral & GPIO_PORTE_BASE))
        return (GPIO_PE4_CAN0RX);

    if ((GPIO_pin & GPIO_PIN_0) && (GPIO_peripheral & GPIO_PORTA_BASE))
        return (GPIO_PA0_CAN1RX);

    if ((GPIO_pin & GPIO_PIN_3) && (GPIO_peripheral & GPIO_PORTF_BASE))
        return (GPIO_PF3_CAN0TX);

    if ((GPIO_pin & GPIO_PIN_5) && (GPIO_peripheral & GPIO_PORTB_BASE))
        return (GPIO_PB5_CAN0TX);

    if ((GPIO_pin & GPIO_PIN_5) && (GPIO_peripheral & GPIO_PORTE_BASE))
        return (GPIO_PE5_CAN0TX);

    if ((GPIO_pin & GPIO_PIN_1) && (GPIO_peripheral & GPIO_PORTA_BASE))
        return (GPIO_PA1_CAN1TX);

    return 0;
}

uint32_t GPIO_periph_macro(uint32_t GPIO_peripheral) {

    if (GPIO_peripheral & GPIO_PORTB_BASE)
        return (SYSCTL_PERIPH_GPIOB);

    if (GPIO_peripheral & GPIO_PORTF_BASE)
        return (SYSCTL_PERIPH_GPIOF);

    if (GPIO_peripheral & GPIO_PORTE_BASE)
        return (SYSCTL_PERIPH_GPIOE);

    if (GPIO_peripheral & GPIO_PORTA_BASE)
        return (SYSCTL_PERIPH_GPIOA);


    return 0;
}

uint32_t CAN_periph_macro(uint32_t CAN_peripheral) {

    if (CAN_peripheral & CAN0_BASE)
        return (SYSCTL_PERIPH_CAN0);

    if (CAN_peripheral & CAN1_BASE)
        return (SYSCTL_PERIPH_CAN1);


    return 0;
}


void init_CanDevice(uint32_t GPIO_peripheral, uint32_t CAN_peripheral, uint32_t GPIO_pinTX, uint32_t GPIO_pinRX, uint32_t bitRate, bool interruption) {

    uint32_t macroPin_CAN, macroPeriph_GPIO, macroPeriph_CAN;

    macroPeriph_GPIO = GPIO_periph_macro(GPIO_peripheral);
    SysCtlPeripheralEnable(macroPeriph_GPIO);

    macroPin_CAN = CAN_macro(GPIO_peripheral, GPIO_pinTX);
    GPIOPinConfigure(macroPin_CAN);

    macroPin_CAN = CAN_macro(GPIO_peripheral, GPIO_pinRX);
    GPIOPinConfigure(macroPin_CAN);

    GPIOPinTypeCAN(GPIO_peripheral, GPIO_pinTX | GPIO_pinRX);
    GPIOPadConfigSet(GPIO_peripheral,  GPIO_pinTX | GPIO_pinRX,
                                 GPIO_STRENGTH_6MA, GPIO_PIN_TYPE_STD);

    macroPeriph_CAN = CAN_periph_macro(CAN_peripheral);
    SysCtlPeripheralEnable(macroPeriph_CAN);

    CANInit(CAN_peripheral);

    CANBitRateSet(CAN_peripheral, SysCtlClockGet(), bitRate);

    if (interruption){

        CANIntEnable(CAN_peripheral, CAN_INT_MASTER | CAN_INT_ERROR | CAN_INT_STATUS);

        if (CAN_peripheral & CAN0_BASE)
            IntEnable(INT_CAN0);
        else
            IntEnable(INT_CAN1);
    }

    IntPrioritySet(INT_CAN0, configMAX_SYSCALL_INTERRUPT_PRIORITY);
    CANEnable(CAN_peripheral);

}

/*void init_SSIperiph(void){

    // configure SSI1 to read from SD card
    //startSSI1();
    //initialize_sd(SSI1);
    //cs_high(SSI1);
    //change_speed(SSI1);
    //cs_low(SSI1);
}
*/
void check_CANerrors(void){

    if(g_ui32ErrFlag & CAN_STATUS_BUS_OFF){

        drawString(30, 50, "\nCAN controller has entered a Bus Off state\n", ST7735_WHITE, ST7735_BLACK, 1, 0);
        // Clear CAN_STATUS_BUS_OFF Flag
        g_ui32ErrFlag &= ~(CAN_STATUS_BUS_OFF);
    }

    if(g_ui32ErrFlag & CAN_STATUS_EWARN){

        drawString(30, 50, "\nCAN controller error level has reached warning level\n", ST7735_WHITE, ST7735_BLACK, 1, 0);
        // Clear CAN_STATUS_EWARN Flag
        g_ui32ErrFlag &= ~(CAN_STATUS_EWARN);
    }

    if(g_ui32ErrFlag & CAN_STATUS_EPASS){

        drawString(30, 50, "\nCAN controller error level has reached error passive level\n", ST7735_WHITE, ST7735_BLACK, 1, 0);
        // Clear CAN_STATUS_EPASS Flag
        g_ui32ErrFlag &= ~(CAN_STATUS_EPASS);
    }

    if(g_ui32ErrFlag & CAN_STATUS_RXOK){

        drawString(30, 50,"\nA message was received successfully since the last read of this status\n", ST7735_WHITE, ST7735_BLACK, 1, 0);
        // Clear CAN_STATUS_RXOK Flag
        g_ui32ErrFlag &= ~(CAN_STATUS_RXOK);
    }

    if(g_ui32ErrFlag & CAN_STATUS_TXOK){

        drawString(30, 50, "\nA message was transmitted successfully since the last read of this status\n", ST7735_WHITE, ST7735_BLACK, 1, 0);
        // Clear CAN_STATUS_TXOK Flag
        g_ui32ErrFlag &= ~(CAN_STATUS_TXOK);
    }

    if(g_ui32ErrFlag & CAN_STATUS_LEC_MSK){

        drawString(30, 50, "\nThis is the mask for the last error code field\n", ST7735_WHITE, ST7735_BLACK, 1, 0);
        // Clear CAN_STATUS_LEC_MSK Flag
        g_ui32ErrFlag &= ~(CAN_STATUS_LEC_MSK);
    }

    if(g_ui32ErrFlag & CAN_STATUS_LEC_STUFF){

        drawString(30, 50, "\nA bit stuffing error has occurred\n", ST7735_WHITE, ST7735_BLACK, 1, 0);
        // Clear CAN_STATUS_LEC_STUFF Flag
        g_ui32ErrFlag &= ~(CAN_STATUS_LEC_STUFF);
    }

    if(g_ui32ErrFlag & CAN_STATUS_LEC_FORM){

        drawString(30, 50, "\nA formatting error has occurred\n", ST7735_WHITE, ST7735_BLACK, 1, 0);
        // Clear CAN_STATUS_LEC_FORM Flag
        g_ui32ErrFlag &= ~(CAN_STATUS_LEC_FORM);
    }

    if(g_ui32ErrFlag & CAN_STATUS_LEC_ACK){

        drawString(30, 50, "\nAn acknowledge error has occurred\n", ST7735_WHITE, ST7735_BLACK, 1, 0);
        // Clear CAN_STATUS_LEC_ACK Flag
        g_ui32ErrFlag &= ~(CAN_STATUS_LEC_ACK);
    }

    if(g_ui32ErrFlag & CAN_STATUS_LEC_BIT1){

        drawString(30, 50, "\nThe bus remained a bit level of 1 for longer than is allowed\n", ST7735_WHITE, ST7735_BLACK, 1, 0);
        // Clear CAN_STATUS_LEC_BIT1 Flag
        g_ui32ErrFlag &= ~(CAN_STATUS_LEC_BIT1);
    }

    if(g_ui32ErrFlag & CAN_STATUS_LEC_BIT0){

        drawString(30, 50, "\nThe bus remained a bit level of 0 for longer than is allowed\n", ST7735_WHITE, ST7735_BLACK, 1, 0);
        // Clear CAN_STATUS_LEC_BIT0 Flag
        g_ui32ErrFlag &= ~(CAN_STATUS_LEC_BIT0);
    }

    if(g_ui32ErrFlag & CAN_STATUS_LEC_CRC){

        drawString(30, 50, "\nA CRC error has occurred\n", ST7735_WHITE, ST7735_BLACK, 1, 0);
        // Clear CAN_STATUS_LEC_CRC Flag
        g_ui32ErrFlag &= ~(CAN_STATUS_LEC_CRC);
    }

    if(g_ui32ErrFlag & CAN_STATUS_LEC_MASK){

        drawString(30, 50, "\nThis is the mask for the CAN Last Error Code (LEC)\n", ST7735_WHITE, ST7735_BLACK, 1, 0);
        // Clear CAN_STATUS_LEC_MASK Flag
        g_ui32ErrFlag &= ~(CAN_STATUS_LEC_MASK);
    }

    if (g_ui32ErrFlag & MSG_OBJ_DATA_LOST){

        drawString(30, 50, "\nCAN message loss detected\n", ST7735_WHITE, ST7735_BLACK, 1, 0);
        g_ui32ErrFlag &= ~(MSG_OBJ_DATA_LOST);
    }

}

// Return the size of the array frame in Hex format (number of bytes of the frame * 2)
uint16_t sizeOfFrame(const char* frame_Hex){

    char c;
    uint16_t size = 0;

    c = frame_Hex[size];
    while(c != '\0') {

       size++;
       c = frame_Hex[size];
    }

    return size;
}

int16_t get_posPID(char *CAN_frame){

    char PID = CAN_frame[2];

    for (int16_t i = 0; i < NUM_LIVE_DATA_PIDS; i++){
        if (PID == pids_liveData[i]){
            return i;
        }
    }
    return -1;
}

double decode_CANdata(uint8_t posPID, double dataA, double dataB){

    double data_decoded;

    switch(posPID){
    case 0:
        data_decoded = (double)dataA*100/255;
        break;

    case 1:
        data_decoded = dataA-40;
        break;

    case 2:
        data_decoded = (dataA-128)*100/128;
        break;

    case 3:
        data_decoded = (dataA-128)*100/128;
        break;

    case 4:
        data_decoded = ((dataA*256)+dataB)/4;
        break;

    case 5:
        data_decoded = dataA;
        break;

    }

    return data_decoded;
}

void show_liveData(double value, uint8_t dataPos){

    char output[7];

    snprintf(output, 6, "%f", value);

    if (output[4] == '.'){

        tabular(output);
    }
    drawString(120, 5+(dataPos*10), output, MENU_DATA_TEXT_COLOUR, ST7735_BLACK, 1, 20);
    drawString(5, 5+(dataPos*10), liveData_strings[dataPos], MENU_DATA_TEXT_COLOUR, ST7735_BLACK, 1, 20);
}

void config_systemPauseTimer(uint16_t time){

    uint32_t pause_time;

    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER2);

    TimerConfigure(TIMER2_BASE, TIMER_CFG_PERIODIC);

    pause_time = (SysCtlClockGet()*time);
    TimerLoadSet(TIMER2_BASE, TIMER_A, pause_time);

    TimerIntEnable(TIMER2_BASE, TIMER_TIMA_TIMEOUT);

    IntEnable(INT_TIMER2A);

    // RTOS masked interrupts higher than configMAX_SYSCALL_INTERRUPT_PRIORITY,
    // to use FreeRTOS API calls from ISR the priority has to be equal or lower than the macro.
    IntPrioritySet(INT_TIMER2A, configMAX_SYSCALL_INTERRUPT_PRIORITY);

}

void system_pause(void){

    TimerEnable(TIMER2_BASE, TIMER_A);
}

void systemPause_TimerISR(void){

    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    TimerIntClear(TIMER2_BASE, TIMER_TIMA_TIMEOUT);
    TimerDisable(TIMER2_BASE, TIMER_A);
    time_expired = true;

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);

}

bool is_Multiframe(char *CAN_frame){

    bool multiframe = false;

    if ((CAN_frame[2] != '4') && (CAN_frame[0] != '2')){

       multiframe = true;
    }

    return multiframe;

}

bool is_ConsecutiveFrame(char *CAN_frame_Hex){

    bool consecutive_frame = false;

    if (CAN_frame_Hex[0] == '2'){


        consecutive_frame = true;
    }

    return consecutive_frame;

}

int16_t get_consecutiveFrame_SequenceNumber(char *CAN_frame_Hex){

    if (is_ConsecutiveFrame(CAN_frame_Hex)){

        return (uint16_t)CAN_frame_Hex[3];
    }

    return -1;
}

int16_t get_numBytes(char *CAN_frame_Hex){

    char cadenaHex_numBytes[3];
    int16_t numBytes = -1;

    if (is_Multiframe(CAN_frame_Hex)){

        cadenaHex_numBytes[0] = CAN_frame_Hex[2];
        cadenaHex_numBytes[1] = CAN_frame_Hex[3];
        cadenaHex_numBytes[2] = '\0';
        numBytes = hex2Decimal(cadenaHex_numBytes);

    }else if (is_ConsecutiveFrame(CAN_frame_Hex)){

        numBytes = 8;

    }else {

        cadenaHex_numBytes[0] = CAN_frame_Hex[0];
        cadenaHex_numBytes[1] = CAN_frame_Hex[1];
        cadenaHex_numBytes[2] = '\0';
        numBytes = hex2Decimal(cadenaHex_numBytes);
    }

    return numBytes;
}

// Return an only data frame or complete frame with Hex format on CAN_frame variable.
void get_CANframe(char **CAN_frame, char *cadena, bool only_data, bool ascii){

    char cadenaHex[HEX_ARRAY+1];
    uint16_t size;

    // Convert it to Hexadecimal
    decimal2Hex(cadena, cadenaHex);

    int16_t numBytes = get_numBytes(cadenaHex);

    uint8_t j;
    if (!ascii){
        if ((*CAN_frame == NULL) && (numBytes >= 0)){
            if (only_data){
                if (is_Multiframe(cadenaHex)){

                    size = (numBytes)*2;
                    if (cadenaHex[5] == '9'){
                        j = 8;
                    }else {
                        j = 6;
                    }

                }else if (is_ConsecutiveFrame(cadenaHex)){

                    size = numBytes*2;
                    j = 0;
                }else {
                    if (cadenaHex[3] == '1'){
                        // Subtract the PID byte and mode byte
                        size = (numBytes-2)*2;
                        j = 6;

                    }else if (cadenaHex[3] == '2'){

                        // Subtract the PID byte, mode byte and frame byte
                        size = (numBytes-3)*2;
                        j = 8;
                    }else {
                        // Subtract the mode byte
                        size = (numBytes-1)*2;
                        j = 4;
                    }
                }
                // Add 1 for  '/0'
                size += 1;
                *CAN_frame = (char*)pvPortMalloc(size*sizeof(char));
                if (CAN_frame == NULL){

                    // TODO: print error message on screen
                    exit(EXIT_FAILURE);
                }
            }else {

                if (is_Multiframe(cadenaHex)){
                    size = (numBytes)*2;
                }else {
                    size = (numBytes+1)*2;
                }

                // Add 1 for  '/0'
                size += 1;
                *CAN_frame = (char*)pvPortMalloc(size*sizeof(char));
                if (CAN_frame == NULL){

                   // TODO: print error message on screen
                   exit(EXIT_FAILURE);
                }
                j = 0;
            }

            memcpy(*CAN_frame, (cadenaHex+j), size-1);
            *(*CAN_frame+size-1) = '\0';
        }
    }else {
        if (only_data){
            if (is_Multiframe(cadenaHex)){
                // Substract the first data byte (char start of header)
                size = 3;
                if (cadenaHex[5] == '9'){
                    j = 5;
                }else {
                    j = 3;
                }

            }else if (is_ConsecutiveFrame(cadenaHex)){

                size = 7;
                j = 1;
            }
            // Add 1 for  '/0'
            size += 1;
            *CAN_frame = (char*)pvPortMalloc(size*sizeof(char));

            memcpy(*CAN_frame, (cadena+j), size-1);
            *(*CAN_frame+size-1) = '\0';
        }
    }
}

bool decode_DTC(char *cadena_DTC_bin, char cadena_DTC_hex[], char DTC_decoded[]){

    bool decoded = false;
    int i = 0;

    if (cadena_DTC_hex[0] == 'A' && cadena_DTC_hex[1] == '5' && cadena_DTC_hex[2] == 'A' && cadena_DTC_hex[3] == '5'){


        return decoded;

    } else{
        while (!decoded){

            switch(cadena_DTC_bin[i]){
                case '0':
                    if (cadena_DTC_bin[i+1] == '0'){
                        if (i == 0){
                            DTC_decoded[0] = 'P';
                        }else {
                            DTC_decoded[1] = '0';
                        }
                    }else if (cadena_DTC_bin[i+1] == '1'){
                        if (i == 0){
                           DTC_decoded[0] = 'C';
                        }else {
                           DTC_decoded[1] = '1';
                        }
                    }
                    break;
                case '1':
                    if (cadena_DTC_bin[i+1] == '0'){
                        if (i == 0){
                           DTC_decoded[0] = 'B';
                        }else {
                           DTC_decoded[1] = '2';
                        }
                    }else if (cadena_DTC_bin[i+1] == '1'){
                        if (i == 0){
                           DTC_decoded[0] = 'U';
                        }else {
                           DTC_decoded[1] = '3';
                        }
                    }
                    break;
            }
            if (i == 2){
                decoded = true;
            }else{
                i = 2;
            }
        }

        DTC_decoded[2] =  cadena_DTC_hex[1];
        DTC_decoded[3] =  cadena_DTC_hex[2];
        DTC_decoded[4] =  cadena_DTC_hex[3];
        DTC_decoded[5] =  '\0';
    }

    return decoded;
}

bool get_DTC_decoded(char input_buffer_DTC[], char decoded_DTC_buffer[]){

    int size_input_buffer_DTC = sizeOfFrame(input_buffer_DTC);
    bool decoded;

    // Each hex char represent by 4 bits
    uint16_t num_Bytes_cadenaBin = size_input_buffer_DTC*4;

    char *cadena_bin = (char*)pvPortMalloc(num_Bytes_cadenaBin*sizeof(char));

    hex2Binary(input_buffer_DTC, &cadena_bin);

    decoded = decode_DTC(cadena_bin, input_buffer_DTC, decoded_DTC_buffer);

    vPortFree(cadena_bin);

    return decoded;
}

void showDTC(char decoded_DTC_buffer[], uint8_t space){

    //cleanScreen();
    for (int i = 0; i < NUM_CHAR_DTC; i++){

        drawChar(10+i*10, 10+space, decoded_DTC_buffer[i], MENU_DATA_TEXT_COLOUR, ST7735_BLACK, 1);
    }
}

int16_t get_numberOfConsecutiveFrameToReceive(char *CAN_FirstFrame){

    int16_t numBytes;
    int16_t numOfFrames;
    int resto;

    if (is_Multiframe(CAN_FirstFrame)){
        numBytes = get_numBytes(CAN_FirstFrame);
        if (numBytes >= 0){
            // 4 bytes sent on First Frame, the rest are sent on frames with 7 data bytes, but the last one is in the middle.
            numOfFrames = ((numBytes - 6)/7);
            resto = ((numBytes - 6)%7);
            if (resto != 0){
                numOfFrames += 1;
            }

        } else{

            return numBytes;
        }
    }else{
        numOfFrames = -1;
    }

    return numOfFrames;
}

void request_PIDs_supportedOnMode02(char **pids_supported){

    uint8_t request_data_frame_aux[MAX_BYTES];
    uint8_t response_data_frame_aux[MAX_BYTES];
    char *CAN_frame = NULL;
    char *CAN_frame_binary = NULL;
    char CAN_frame_Hex[5];
    char cadena_decimal[30];

    EventBits_t bitsReaded;

    CANRxMessage.pui8MsgData = response_data_frame_aux;
    CANMessageSet(CAN0_BASE, RXOBJECT, &CANRxMessage, MSG_OBJ_TYPE_RX);

    request_data_frame_aux[0] = 0x03;
    request_data_frame_aux[1] = 0x02;
    request_data_frame_aux[2] = 0x00;
    request_data_frame_aux[3] = 0x00;
    for (int i = 4; i < MAX_BYTES; i++){

        request_data_frame_aux[i] = 0x55;
    }
    CANTxMessage.ui32Flags = MSG_OBJ_NO_FLAGS;
    CANTxMessage.ui32MsgLen = 4;
    CANTxMessage.pui8MsgData = request_data_frame_aux;

    CANMessageSet(CAN0_BASE, TXOBJECT, &CANTxMessage, MSG_OBJ_TYPE_TX);

    bitsReaded = xEventGroupWaitBits(flagEvents, CAN_RX_INTERRUPT, pdTRUE, pdFALSE, portMAX_DELAY);
    if (bitsReaded & CAN_RX_INTERRUPT){

        CANMessageGet(CAN0_BASE, RXOBJECT, &CANRxMessage, 0);
        get_CANframe(&CAN_frame, (char*)response_data_frame_aux, false, false);

        CAN_frame_Hex[0] = CAN_frame[8];
        CAN_frame_Hex[1] = CAN_frame[9];
        CAN_frame_Hex[2] = CAN_frame[10];
        CAN_frame_Hex[3] = CAN_frame[11];
        CAN_frame_Hex[4] = '\0';
        hex2Binary(CAN_frame_Hex, &CAN_frame_binary);

        find_PIDsupported(CAN_frame_binary, cadena_decimal);

        int size = sizeOfFrame(cadena_decimal);

        *pids_supported = (char*)pvPortMalloc(size*sizeof(char));
        // Substract the PID 02
        memcpy(*pids_supported, cadena_decimal+1, size-1);
        pids_supported[size-1] = '\0';

        vPortFree(CAN_frame_binary);
        CAN_frame_binary = NULL;
    }
    xEventGroupClearBits(flagEvents, CAN_RX_INTERRUPT);
}

void request_PIDs_supportedOnMode01(char **pids_supported){

    uint8_t request_data_frame_aux[MAX_BYTES];
    uint8_t response_data_frame_aux[MAX_BYTES];
    char *CAN_frame = NULL;
    char *CAN_frame_binary = NULL;
    char CAN_frame_Hex[9];
    char cadena_decimal[30];

    EventBits_t bitsReaded;


    CANRxMessage.ui32MsgID = ECU_ID_Response;
    CANRxMessage.ui32MsgIDMask = MASK_RESPONSE_ID;
    CANRxMessage.ui32Flags = MSG_OBJ_RX_INT_ENABLE | MSG_OBJ_USE_ID_FILTER;
    CANRxMessage.ui32MsgLen = 8; // 8 bytes
    CANRxMessage.pui8MsgData = response_data_frame_aux;
    CANMessageSet(CAN0_BASE, RXOBJECT, &CANRxMessage, MSG_OBJ_TYPE_RX);

    request_data_frame_aux[0] = 0x02;
    request_data_frame_aux[1] = 0x01;
    request_data_frame_aux[2] = 0x00;
    for (int i = 3; i < MAX_BYTES; i++){

        request_data_frame_aux[i] = 0x55;
    }
    CANTxMessage.ui32Flags = MSG_OBJ_NO_FLAGS;
    CANTxMessage.ui32MsgLen = 3;
    CANTxMessage.pui8MsgData = request_data_frame_aux;
    CANTxMessage.ui32MsgIDMask = 0;
    CANTxMessage.ui32MsgID = REMOTE_REQUEST_ID;

    CANMessageSet(CAN0_BASE, TXOBJECT, &CANTxMessage, MSG_OBJ_TYPE_TX);

    bitsReaded = xEventGroupWaitBits(flagEvents, CAN_RX_INTERRUPT, pdTRUE, pdFALSE, portMAX_DELAY);
    if (bitsReaded & CAN_RX_INTERRUPT){

        CANMessageGet(CAN0_BASE, RXOBJECT, &CANRxMessage, 0);
        get_CANframe(&CAN_frame, (char*)response_data_frame_aux, false, false);

        CAN_frame_Hex[0] = CAN_frame[6];
        CAN_frame_Hex[1] = CAN_frame[7];
        CAN_frame_Hex[2] = CAN_frame[8];
        CAN_frame_Hex[3] = CAN_frame[9];
        CAN_frame_Hex[4] = CAN_frame[10];
        CAN_frame_Hex[5] = CAN_frame[11];
        CAN_frame_Hex[6] = CAN_frame[12];
        CAN_frame_Hex[7] = CAN_frame[13];
        CAN_frame_Hex[8] = '\0';
        hex2Binary(CAN_frame_Hex, &CAN_frame_binary);

        find_PIDsupported(CAN_frame_binary, cadena_decimal);

        int size = sizeOfFrame(cadena_decimal);

        *pids_supported = (char*)pvPortMalloc(size*sizeof(char));
        // Substract the PID 02
        memcpy(*pids_supported, cadena_decimal+1, size-1);
        pids_supported[size-1] = '\0';

        vPortFree(CAN_frame_binary);
        CAN_frame_binary = NULL;
    }
    xEventGroupClearBits(flagEvents, CAN_RX_INTERRUPT);
}

bool valid_DTC(char DTC[]){

    if (DTC[0] == 'P' && DTC[1] == '0'){
        if (DTC[2] == '0' && DTC[3] && DTC[4] == '0'){

            return false;
        }
    }
    return true;
}

void find_PIDsupported(char *CAN_frame_binary, char *decimal){

    int size = sizeOfFrame(CAN_frame_binary);
    int totalPIDs_supported = 0;

    for (int i = 0; i < size; i++){
        if (CAN_frame_binary[i] == '1'){

            decimal[totalPIDs_supported] = i + 1;
            totalPIDs_supported++;
        }
    }
    decimal[totalPIDs_supported] = '\0';
}
