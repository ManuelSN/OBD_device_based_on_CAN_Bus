/*
 * CAN_Device.h
 *
 *  Created on: 23 oct. 2020
 *      Author: Manuel Sánchez Natera
 *
 *      This work is licensed under the Creative Commons Attribution-NonCommercial 4.0 International License.
 *      To view a copy of this license, visit http://creativecommons.org/licenses/by-nc/4.0/ or send a letter to
 *      Creative Commons, PO Box 1866, Mountain View, CA 94042, USA.
 */

#ifndef CAN_DEVICE_H_
#define CAN_DEVICE_H_

// Libraries
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>

// Defines of the program
// ECU address
#define ECM_REQUEST 0x7E0
#define ECM_RESPONSE 0x7E8

#define TCM_REQUEST 0x7E1
#define TCM_RESPONSE 0x7E9

#define ABS_REQUEST 0x7E2
#define ABS_RESPONSE 0x7EA

#define REMOTE_REQUEST_ID 0x7DF
#define MASK_RESPONSE_ID 0x7FFU

//#define CAN0RXID ECM // default value
//#define CAN0TXID REMOTE_REQUEST_ID
// Message Objects
#define TXOBJECT 2
#define RXOBJECT 1

// CAN configuration
#define HEX_ARRAY 16
#define MAX_BYTES 8
#define NUM_CHAR_DTC 5
#define BIT_RATE 500000
#define NUM_LIVE_DATA_PIDS 6
#define FREEZE_SCREEN_TIME 2 // in seconds
#define MAX_VIN_BYTES 20

#define MAX_TIME_TO_WAIT_MS 200

// Mode defines
#define SELECT_CAN_COMMAND (1 << 0)
#define VEHICLE_INFORMATION (1 << 1)
#define READ_DTC (1 << 2)
#define READ_DTC_DRIVING_CYCLE (1 << 3)
#define CAN_RX_INTERRUPT (1 << 4)
#define CAN_TX_INTERRUPT (1 << 5)
#define CAN_ERROR_INTERRUPT (1 << 6)
#define LIVE_ALL_DATA (1 << 7)
#define FREEZE_FRAME (1 << 8)
#define ERASE_DTC (1 << 9)
#define SELECT_ECU_ADDRESS (1 << 10)

static uint8_t pids_liveData[] = {0x04, 0x05, 0x06, 0x07, 0x0C, 0x0D};
static const char *liveData_strings[]= {"Charge motor:",
                                     "Motor temperature:",
                                     "S.F.C.(Bank 1):",
                                     "L.F.C.(Bank 1):",
                                     "RPM:",
                                     "Speed:"
};

static const char *DTC_encoded[] = {"P0107",
                                    "P0207",
                                    "P0307",
                                    "B2245",
                                    "U1600"
};


// CAN Bus Peripheral Functions
uint32_t CAN_macro(uint32_t GPIO_peripheral, uint32_t GPIO_pin);
uint32_t GPIO_periph_macro(uint32_t GPIO_peripheral);
uint32_t CAN_periph_macro(uint32_t CAN_peripheral);
void init_CanDevice(uint32_t GPIO_peripheral, uint32_t CAN_peripheral, uint32_t GPIO_pinTX, uint32_t GPIO_pinRX, uint32_t bitRate, bool interruption);
void CANIntHandler(void);
void check_CANerrors(void);

// Tasks Functions
void init_deviceTasks(void);
//void init_SSIperiph(void);
void init_flagEvents(void);
static portTASK_FUNCTION(Read_DTC, pvParameters);
static portTASK_FUNCTION(Live_all_data, pvParameters);
static portTASK_FUNCTION(Erase_DTCs, pvParameters);
static portTASK_FUNCTION(Get_VIN, pvParameters);
static portTASK_FUNCTION(Freeze_frame, pvParameters);

// Auxiliary Functions
void get_CANframe(char **CAN_frame, char *cadena, bool only_data, bool ascii);
void config_systemPauseTimer(uint16_t time);
void system_pause(void);
void show_liveData(double value, uint8_t dataPos);
void find_PIDsupported(char *CAN_frame_binary, char *decimal);
void showDTC(char decoded_DTC_buffer[], uint8_t space);
void request_PIDs_supportedOnMode02(char **pids_supported);
void request_PIDs_supportedOnMode01(char **pids_supported);

double decode_CANdata(uint8_t posPID, double dataA, double dataB);
uint16_t sizeOfFrame(const char* frame_Hex);
int16_t get_posPID(char *CAN_frame);
int16_t get_consecutiveFrame_SequenceNumber(char *CAN_frame_Hex);
int16_t get_numBytes(char *CAN_frame_Hex);
int16_t get_numberOfConsecutiveFrameToReceive(char *CAN_FirstFrame);

bool valid_DTC(char DTC[]);
bool decode_DTC(char *cadena_DTC_bin, char *cadena_DTC_hex, char DTC_decoded[]);
bool get_DTC_decoded(char *input_buffer_DTC, char decoded_DTC_buffer[]);
bool is_ConsecutiveFrame(char *CAN_frame_Hex);
bool is_Multiframe(char *CAN_frame_Hex);


#endif /* CAN_DEVICE_H_ */
