//*****************************************************************************
//
// uartstdio.c - Utility driver to provide simple UART console functions.
//
// Copyright (c) 2007-2014 Texas Instruments Incorporated.  All rights reserved.
// Software License Agreement
// 
// Texas Instruments (TI) is supplying this software for use solely and
// exclusively on TI's microcontroller products. The software is owned by
// TI and/or its suppliers, and is protected under applicable copyright
// laws. You may not combine this software with "viral" open-source
// software in order to form a larger program.
// 
// THIS SOFTWARE IS PROVIDED "AS IS" AND WITH ALL FAULTS.
// NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT
// NOT LIMITED TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. TI SHALL NOT, UNDER ANY
// CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
// DAMAGES, FOR ANY REASON WHATSOEVER.
// 
//
//*****************************************************************************
//JMCG, EGP, NHR: Modificado para:
// -> portada al CC3200...
// -> Simplificar la rutina de interrupcion, eliminando codigo y pasandolo a funciones que se llaman desde tareas
// -> Mejorar la integracion con FreeRTOS mediante colas de mensajes
// -> Pasar parte del procesado a la funcion gets, que ademas se hace bloqueante. Esta funcion solo debe usarse en tareas
// -> Incluir la opcion de historial (funcion gets)



#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_uart.h"
#include "driverlib/debug.h"
#include "driverlib/interrupt.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "utils/uartstdio.h"

//*****************************************************************************
//
//! \addtogroup uartstdio_api
//! @{
//
//*****************************************************************************



//*****************************************************************************
//
// This global controls whether or not we are echoing characters back to the
// transmitter.  By default, echo is enabled but if using this module as a
// convenient method of implementing a buffered serial interface over which
// you will be running an application protocol, you are likely to want to
// disable echo by calling UARTEchoSet(false).
//
//*****************************************************************************
static bool g_bDisableEcho;

//*****************************************************************************
//
// Use RTOS queues as buffers,
//
//*****************************************************************************


/* The queue used to hold received characters. */
static QueueHandle_t xRxedChars;

/* The queue used to hold characters waiting transmission. */
static QueueHandle_t xCharsForTx;

static SemaphoreHandle_t TxMutex; //Enables the use of UARTprintf and UARTWrite in more than one task


#ifdef WANT_CMDLINE_HISTORY
// Extended History support variables
static char g_pcUARTRxHistoryBuffer[UART_RX_HISTORY_BUF_SIZE];
static unsigned short g_usUARTRxHistoryOffset[UART_RX_HISTORY_DEPTH];
static unsigned short g_usUARTRxHistoryBufIndex = 0;
static signed char g_bUARTRxHistoryNext = 0;
static signed char g_bUARTRxHistoryCur = -1;
static signed char g_bUARTRxHistoryCnt = 0;
static unsigned char g_ubEscMode = 0;
#endif /* WANT_CMDLINE_HISTORY */

//*****************************************************************************
//
// The base address of the chosen UART.
//
//*****************************************************************************
static uint32_t g_ui32Base = 0;

//*****************************************************************************
//
// A mapping from an integer between 0 and 15 to its ASCII character
// equivalent.
//
//*****************************************************************************
static const char * const g_pcHex = "0123456789abcdef";

//*****************************************************************************
//
// The list of possible base addresses for the console UART.
//
//*****************************************************************************
static const uint32_t g_ui32UARTBase[3] =
{
		UART0_BASE, UART1_BASE, UART2_BASE
};


//*****************************************************************************
//
// The list of possible interrupts for the console UART.
//
//*****************************************************************************
static const uint32_t g_ui32UARTInt[3] =
{
		INT_UART0, INT_UART1, INT_UART2
};

//*****************************************************************************
//
// The port number in use.
//
//*****************************************************************************
static uint32_t g_ui32PortNum; //De momento parece que no se usa �Eliminar?


//*****************************************************************************
//
// The list of UART peripherals.
//
//*****************************************************************************
static const uint32_t g_ui32UARTPeriph[3] =
{
		SYSCTL_PERIPH_UART0, SYSCTL_PERIPH_UART1, SYSCTL_PERIPH_UART2
};



//*****************************************************************************
//
// Take as many bytes from the transmit buffer as we have space for and move
// them into the UART transmit FIFO.
//
//*****************************************************************************

static void UARTPrimeTransmit(uint32_t ui32Base)
{
		//
		// Disable the UART interrupt.  If we don't do this there is a race
		// condition which can cause the read index to be corrupted.
		//
		MAP_UARTIntDisable(g_ui32Base, UART_INT_TX);

		//
		// Yes - take some characters out of the transmit buffer and feed
		// them to the UART transmit FIFO.
		//
		while(UARTSpaceAvail(ui32Base) && (uxQueueMessagesWaiting(xCharsForTx)))
		{
			uint8_t data;
			xQueueReceive(xCharsForTx,&data,portMAX_DELAY);
			MAP_UARTCharPutNonBlocking(ui32Base,data);
		}

		//
		// Reenable the UART interrupt.
		//
		MAP_UARTIntEnable(g_ui32Base, UART_INT_TX);
}




//*****************************************************************************
//
//! Configures the UART console.
//!
//! \param ui32PortNum is the number of UART port to use for the serial console
//! (0-2)
//! \param ui32Baud is the bit rate that the UART is to be configured to use.
//!
//! This function will configure the specified serial port to be used as a
//! serial console.  The serial parameters are set to the baud rate
//! specified by the \e ui32Baud parameter and use 8 bit, no parity, and 1 stop
//! bit.
//!
//! This function must be called prior to using any of the other UART console
//! functions: UARTprintf() or UARTgets().  This function assumes that the
//! caller has previously configured the relevant UART pins for operation as a
//! UART rather than as GPIOs.
//!
//! \return None.
//
//*****************************************************************************
//Initializes the UART and creates the messaque queues for TX and RX
void
UARTStdioConfig(uint32_t ui32PortNum, uint32_t ui32Baud, uint32_t ui32SrcClock)
{
	//
	// Check the arguments.
	//
	ASSERT((ui32PortNum == 0) || (ui32PortNum == 1) ||
			(ui32PortNum == 2));


	//
	// In buffered mode, we only allow a single instance to be opened.
	//
	ASSERT(g_ui32Base == 0);


	//
	// Select the base address of the UART.
	//
	g_ui32Base = g_ui32UARTBase[ui32PortNum];

	//
	// Enable the UART peripheral for use.
	//

	MAP_SysCtlPeripheralEnable(g_ui32UARTPeriph[ui32PortNum]);
	MAP_SysCtlPeripheralSleepEnable(g_ui32UARTPeriph[ui32PortNum]);

	//
	// Configure the UART for 115200, n, 8, 1
	//
	MAP_UARTConfigSetExpClk(g_ui32Base, ui32SrcClock, ui32Baud,
			(UART_CONFIG_PAR_NONE | UART_CONFIG_STOP_ONE |
					UART_CONFIG_WLEN_8));

	//
	// Set the UART to interrupt whenever the TX FIFO is almost empty or
	// when any character is received.
	//
	MAP_UARTFIFOLevelSet(g_ui32Base, UART_FIFO_TX1_8, UART_FIFO_RX1_8);

	//
	// Remember which interrupt we are dealing with.
	//
	g_ui32PortNum = ui32PortNum;

	//
	// We are configured for buffered output so enable the master interrupt
	// for this UART and the receive interrupts.  We don't actually enable the
	// transmit interrupt in the UART itself until some data has been placed
	// in the transmit buffer.
	//
	MAP_UARTIntDisable(g_ui32Base, 0xFFFFFFFF);

	// Set the interrupt priority
	MAP_IntPrioritySet(g_ui32UARTInt[ui32PortNum], configMAX_SYSCALL_INTERRUPT_PRIORITY);

	//creates the message queues and the mutex
	xRxedChars = xQueueCreate( UART_RX_BUFFER_SIZE, ( unsigned portBASE_TYPE ) sizeof( signed char ) );
	xCharsForTx = xQueueCreate( UART_TX_BUFFER_SIZE, ( unsigned portBASE_TYPE ) sizeof( signed char ) );

	if ((xRxedChars==NULL)||(xCharsForTx==NULL))
	    while (1); //Better use an assert!!

#if configUSE_MUTEXES
	TxMutex = xSemaphoreCreateMutex();
	if (TxMutex==NULL)
	    while (1); //Better use an assert!!
#endif

	//
	// Flush both the buffers.
	//
	UARTFlushRx();
	UARTFlushTx(true);

	MAP_UARTIntEnable(g_ui32Base, UART_INT_RX | UART_INT_RT);
	MAP_IntEnable(g_ui32UARTInt[ui32PortNum]);

	//
	// Enable the UART operation.
	//
	MAP_UARTEnable(g_ui32Base);
}

//*****************************************************************************
//
//! Writes a string of characters to the UART output.
//!
//! \param pcBuf points to a buffer containing the string to transmit.
//! \param ui32Len is the length of the string to transmit.
//!
//! This function will transmit the string to the UART output.  The number of
//! characters transmitted is determined by the \e ui32Len parameter.  This
//! function does no interpretation or translation of any characters.  Since
//! the output is sent to a UART, any LF (/n) characters encountered will be
//! replaced with a CRLF pair.
//!
//! Besides using the \e ui32Len parameter to stop transmitting the string, if
//! a null character (0) is encountered, then no more characters will be
//! transmitted and the function will return.
//!
//! In non-buffered mode, this function is blocking and will not return until
//! all the characters have been written to the output FIFO.  In buffered mode,
//! the characters are written to the UART transmit buffer and the call returns
//! immediately.  If insufficient space remains in the transmit buffer,
//! additional characters are discarded.
//!
//! \return Returns the count of characters written.
//
//*****************************************************************************
int
UARTwrite(const char *pcBuf, uint32_t ui32Len)
{
	unsigned int uIdx;

	//
	// Check for valid arguments.
	//
	ASSERT(pcBuf != 0);
	ASSERT(g_ui32Base != 0);

#if configUSE_MUTEXES
	xSemaphoreTake(TxMutex,portMAX_DELAY); //Use the mutex if available
#endif

	//
	// Send the characters
	//
	for(uIdx = 0; uIdx < ui32Len; uIdx++)
	{
		//
		// If the character to the UART is \n, then add a \r before it so that
		// \n is translated to \n\r in the output.
		//
		if(pcBuf[uIdx] == '\n')
		{
				const uint8_t data='\r';
				xQueueSend(xCharsForTx,&data,portMAX_DELAY);
				UARTPrimeTransmit(g_ui32Base);
		}

		//
		// Send the character to the UART output.
		//
		xQueueSend(xCharsForTx,(pcBuf+uIdx),portMAX_DELAY);
		UARTPrimeTransmit(g_ui32Base);
	}


#if configUSE_MUTEXES
    xSemaphoreGive(TxMutex); //Use the mutex if available
#endif

	//
	// Return the number of characters written.
	//
	return(uIdx);
}


//*****************************************************************************
//
//! A simple UART based get string function, with some line processing.
//!
//! \param pcBuf points to a buffer for the incoming string from the UART.
//! \param ui32Len is the length of the buffer for storage of the string,
//! including the trailing 0.
//!
//! This function will receive a string from the UART input and store the
//! characters in the buffer pointed to by \e pcBuf.  The characters will
//! continue to be stored until a termination character is received.  The
//! termination characters are CR, LF, or ESC.  A CRLF pair is treated as a
//! single termination character.  The termination characters are not stored in
//! the string.  The string will be terminated with a 0 and the function will
//! return.
//!
//! In both buffered and unbuffered modes, this function will block until
//! a termination character is received.  If non-blocking operation is required
//! in buffered mode, a call to UARTPeek() may be made to determine whether
//! a termination character already exists in the receive buffer prior to
//! calling UARTgets().
//!
//! Since the string will be null terminated, the user must ensure that the
//! buffer is sized to allow for the additional null character.
//!
//! \return Returns the count of characters that were stored, not including
//! the trailing 0.
//
//*****************************************************************************

//JMCG: Con respecto al original suministrado por Texas Instruments,
// Se ha pasado gran parte de la ISR a esta funcion (que se ejecutara desde tarea)
// En concreto se pasa aqu� la gesti�n de los car�cteres de borrado, del eco y del historial,
// ya que no tiene sentido hacerlo en la ISR.


int
UARTgets(char *pcBuf, uint32_t ui32Len)
{
	uint32_t ui32Count = 0;
	int8_t cChar;
	bool bLastWasCR = false;

	//
	// Check the arguments.
	//
	ASSERT(pcBuf != 0);
	ASSERT(ui32Len != 0);
	ASSERT(g_ui32Base != 0);

	//
	// Adjust the length back by 1 to leave space for the trailing
	// null terminator.
	//
	ui32Len--;

	//
	// Process characters until a newline is received.
	//
	while(ui32Count<ui32Len)
	{
		//
		// Read the next character from the receive buffer.
		//
		xQueueReceive(xRxedChars,&cChar,portMAX_DELAY);

		//
		// If echo is disabled, we skip the various text filtering
		// operations that would typically be required when supporting a
		// command line.
		//
		if(!g_bDisableEcho)
		{
			//
			// Handle backspace by erasing the last character in the
			// buffer.
			//
			if ((cChar == '\b')||(cChar == 0x7F))
			{
				//
				// If there are any characters already in the buffer, then
				// delete the last.
				//

				if((ui32Count>0))
				{
					//
					// Rub out the previous character on the users
					// terminal.
					//
					UARTwrite("\b \b", 3);
					ui32Count--; //Delete from buffer

				}

				//
				// Skip ahead to read the next character.
				//
				continue;
			}

			//
			// Check if in an ESC sequence (arrow keys)
			//
			// El c�digo de abajo implementa el historial de comandos...
#ifdef WANT_CMDLINE_HISTORY
			if (g_ubEscMode != 0)
			{
				// Test if this is the first byte after the ESCape
				if (g_ubEscMode == 1)
				{
					// Test the known escape codes
					if (cChar == '[')
					{
						g_ubEscMode = 2;
						continue;
					}
					else
						// Unknown code
					{
						g_ubEscMode = 0;
						continue;
					}
				}
				else
				{
					// Test for UP or DOWN arrow
					if ((cChar == 'A') || (cChar == 'B'))
					{
						unsigned short copyFrom, x;
						int current = g_bUARTRxHistoryCur;

						g_ubEscMode = 0;

						if (cChar == 'A')
						{
							// Test if this is the first history request
							if (current == -1)
								current = g_bUARTRxHistoryNext;

							// Now decrement to previous history
							if (--current < 0)
								current = UART_RX_HISTORY_DEPTH-1;

							// Get previous history into Rx buffer
							if ((g_bUARTRxHistoryNext == g_bUARTRxHistoryCur) ||
									(current > g_bUARTRxHistoryCnt))
							{
								// At end of history.  Just return
								UARTwrite("\x07", 1);
								continue;
							}
						}
						else
						{
							// Test if this is the first history request
							if (current == -1)
								continue;

							// Now increment to next history
							if (++current >= UART_RX_HISTORY_DEPTH)
								current = 0;

							// Test if we were on the last history item
							if (current == g_bUARTRxHistoryNext)
							{
								UARTwrite("\r\x1b[2K> ", 7);
								//Clear the line
								ui32Count=0;
								g_bUARTRxHistoryCur = -1;
								continue;
							}
						}
						g_bUARTRxHistoryCur = current;

						// Copy the history item to the Rx Buffer
						UARTwrite("\r\x1b[2K> ", 7);
						ui32Count=0; //Clear the line
						copyFrom = g_usUARTRxHistoryOffset[current];
						for (x = 0; g_pcUARTRxHistoryBuffer[copyFrom] != '\0'; x++)
						{
							pcBuf[ui32Count]=g_pcUARTRxHistoryBuffer[copyFrom++];

							UARTwrite((const char *) &pcBuf[ui32Count], 1);

							ui32Count++;
							if (copyFrom >= UART_RX_HISTORY_BUF_SIZE)
								copyFrom = 0;
						}
						pcBuf[ui32Count] = '\0';

						continue;
					}
					// Test for LEFT arrow
					else if (cChar == 'C')
					{
						g_ubEscMode = 0;
						continue;
					}
					// Test for RIGHT arrow
					else if (cChar == 'D')
					{
						g_ubEscMode = 0;
						continue;
					}
					else
						// Unknown sequence
					{
						g_ubEscMode = 0;
						continue;
					}
				}
			}
#endif /* WANT_CMDLINE_HISTORY */

			//
			// If this character is LF and last was CR, then just gobble up
			// the character since we already echoed the previous CR and we
			// don't want to store 2 characters in the buffer if we don't
			// need to.
			//
			if((cChar == '\n') && bLastWasCR)
			{
				bLastWasCR = false;
#ifdef WANT_CMDLINE_HISTORY
				g_ubEscMode = 0;
#endif /* WANT_CMDLINE_HISTORY */
				continue;
			}

#ifdef WANT_CMDLINE_HISTORY
			//
			// Test for ESC sequence
			//
			if (cChar == 0x1b)
			{
				bLastWasCR = false;
#ifdef WANT_CMDLINE_HISTORY
				g_ubEscMode = 1;
#endif /* WANT_CMDLINE_HISTORY */
				continue;
			}
#endif

			//
			// See if a newline or escape character was received.
			//
			if((cChar == '\r') || (cChar == '\n') || (cChar == 0x1b))
			{
				signed char  next, prev;
				int count;
				uint32_t x;

				//
				// If this character is LF and last was CR, then just gobble up
				// the character since we already echoed the previous CR and we
				// don't want to store 2 characters in the buffer if we don't
				// need to.
				//
				if((cChar == '\n') && bLastWasCR)
				{
					bLastWasCR = false;
					continue;
				}

				//
				// See if a newline or escape character was received.
				//
				if((cChar == '\r') || (cChar == '\n') || (cChar == 0x1b))
				{
					//
					// If the character is a CR, then it may be followed by an
					// LF which should be paired with the CR.  So remember that
					// a CR was received.
					//
					if(cChar == '\r')
					{
						bLastWasCR = 1;
					}

					//
					// Regardless of the line termination character received,
					// put a CR in the receive buffer as a marker telling
					// UARTgets() where the line ends.  We also send an
					// additional LF to ensure that the local terminal echo
					// receives both CR and LF.
					//
					cChar = '\r';
					UARTwrite("\n", 1);

					//
					// Add command to the command history
					//
					if (ui32Count > 0)
					{
						int y, duplicateCmd;

						// Test if this command matches the last command
						duplicateCmd = false;
						if (g_bUARTRxHistoryCnt > 0)
						{
							prev = g_bUARTRxHistoryNext-1;
							if (prev < 0)
								prev = UART_RX_HISTORY_DEPTH-1;

							x = 0;
							y = g_usUARTRxHistoryOffset[prev];
							for (count=ui32Count; count > 0; count--)
							{
								if (g_pcUARTRxHistoryBuffer[y++] != pcBuf[x++]) //g_pcUARTRxBuffer[x++])
									break;
								if (y >= UART_RX_HISTORY_BUF_SIZE)
									y = 0;
							}

							// Test if the command matches the last
							if ((count == 0) && (g_pcUARTRxHistoryBuffer[y] == 0))
								duplicateCmd = true;
						}

						// Now save the command in the history buffer
						if (!duplicateCmd)
						{
							next = g_bUARTRxHistoryNext++;
							if (g_bUARTRxHistoryNext == UART_RX_HISTORY_DEPTH)
								g_bUARTRxHistoryNext = 0;
							if (g_bUARTRxHistoryCnt != UART_RX_HISTORY_DEPTH)
								g_bUARTRxHistoryCnt++;
							g_usUARTRxHistoryOffset[next] = g_usUARTRxHistoryBufIndex;

							x =0;
							for (count = ui32Count ; count > 0 ; count--)
							{

								g_pcUARTRxHistoryBuffer[g_usUARTRxHistoryBufIndex++] =pcBuf[x++];
								if (g_usUARTRxHistoryBufIndex >= UART_RX_HISTORY_BUF_SIZE)
									g_usUARTRxHistoryBufIndex = 0;
							}
							g_pcUARTRxHistoryBuffer[g_usUARTRxHistoryBufIndex++] = '\0';
							if (g_usUARTRxHistoryBufIndex >= UART_RX_HISTORY_BUF_SIZE)
								g_usUARTRxHistoryBufIndex = 0;
						}
					}
					g_bUARTRxHistoryCur = -1;
				}

				break;
			} //This is the end of the end of line.....

			//Envia el eco...
			UARTwrite((const char *)&cChar, 1);

		}	//If disable echo....

		//
		// Process the received character as long as we are not at the end
		// of the buffer.  If the end of the buffer has been reached then
		// all additional characters are ignored until a newline is
		// received.
		//
		if(ui32Count < ui32Len)
		{
			//
			// Store the character in the caller supplied buffer.
			//
			pcBuf[ui32Count] = cChar;

			//
			// Increment the count of characters received.
			//
			ui32Count++;
		}

	} // The while (datos<esperados) {read and process}

//
// Add a null termination to the string.
//
	pcBuf[ui32Count] = 0;

//
// Return the count of int8_ts in the buffer, not counting the trailing 0.
//
	return(ui32Count);
}

//*****************************************************************************
//
//! Read a single character from the UART, blocking if necessary.
//!
//! This function will receive a single character from the UART and store it at
//! the supplied address.
//!
//! In both buffered and unbuffered modes, this function will block until a
//! character is received.  If non-blocking operation is required in buffered
//! mode, a call to UARTRxAvail() may be made to determine whether any
//! characters are currently available for reading.
//!
//! \return Returns the character read.
//
//*****************************************************************************
unsigned char
UARTgetc(void)
{
	unsigned char cChar;

	//
	// Wait for a character to be received and read a character from the buffer.
	//
	xQueueReceive(xRxedChars,&cChar,portMAX_DELAY);

	//
	// Return the character to the caller.
	//
	return(cChar);
}

//*****************************************************************************
//
//! A simple UART based vprintf function supporting \%c, \%d, \%p, \%s, \%u,
//! \%x, and \%X.
//!
//! \param pcString is the format string.
//! \param vaArgP is a variable argument list pointer whose content will depend
//! upon the format string passed in \e pcString.
//!
//! This function is very similar to the C library <tt>vprintf()</tt> function.
//! All of its output will be sent to the UART.  Only the following formatting
//! characters are supported:
//!
//! - \%c to print a character
//! - \%d or \%i to print a decimal value
//! - \%s to print a string
//! - \%u to print an unsigned decimal value
//! - \%x to print a hexadecimal value using lower case letters
//! - \%X to print a hexadecimal value using lower case letters (not upper case
//! letters as would typically be used)
//! - \%p to print a pointer as a hexadecimal value
//! - \%\% to print out a \% character
//!
//! For \%s, \%d, \%i, \%u, \%p, \%x, and \%X, an optional number may reside
//! between the \% and the format character, which specifies the minimum number
//! of characters to use for that value; if preceded by a 0 then the extra
//! characters will be filled with zeros instead of spaces.  For example,
//! ``\%8d'' will use eight characters to print the decimal value with spaces
//! added to reach eight; ``\%08d'' will use eight characters as well but will
//! add zeroes instead of spaces.
//!
//! The type of the arguments in the variable arguments list must match the
//! requirements of the format string.  For example, if an integer was passed
//! where a string was expected, an error of some kind will most likely occur.
//!
//! \return None.
//
//*****************************************************************************
void
UARTvprintf(const char *pcString, va_list vaArgP)
{
	uint32_t ui32Idx, ui32Value, ui32Pos, ui32Count, ui32Base, ui32Neg;
	char *pcStr, pcBuf[16], cFill;

	//
	// Check the arguments.
	//
	ASSERT(pcString != 0);

	//
	// Loop while there are more characters in the string.
	//
	while(*pcString)
	{
		//
		// Find the first non-% character, or the end of the string.
		//
		for(ui32Idx = 0;
				(pcString[ui32Idx] != '%') && (pcString[ui32Idx] != '\0');
				ui32Idx++)
		{
		}

		//
		// Write this portion of the string.
		//
		UARTwrite(pcString, ui32Idx);

		//
		// Skip the portion of the string that was written.
		//
		pcString += ui32Idx;

		//
		// See if the next character is a %.
		//
		if(*pcString == '%')
		{
			//
			// Skip the %.
			//
			pcString++;

			//
			// Set the digit count to zero, and the fill character to space
			// (in other words, to the defaults).
			//
			ui32Count = 0;
			cFill = ' ';

			//
			// It may be necessary to get back here to process more characters.
			// Goto's aren't pretty, but effective.  I feel extremely dirty for
			// using not one but two of the beasts.
			//
			again:

			//
			// Determine how to handle the next character.
			//
			switch(*pcString++)
			{
			//
			// Handle the digit characters.
			//
			case '0':
			case '1':
			case '2':
			case '3':
			case '4':
			case '5':
			case '6':
			case '7':
			case '8':
			case '9':
			{
				//
				// If this is a zero, and it is the first digit, then the
				// fill character is a zero instead of a space.
				//
				if((pcString[-1] == '0') && (ui32Count == 0))
				{
					cFill = '0';
				}

				//
				// Update the digit count.
				//
				ui32Count *= 10;
				ui32Count += pcString[-1] - '0';

				//
				// Get the next character.
				//
				goto again;
			}

			//
			// Handle the %c command.
			//
			case 'c':
			{
				//
				// Get the value from the varargs.
				//
				ui32Value = va_arg(vaArgP, uint32_t);

				//
				// Print out the character.
				//
				UARTwrite((char *)&ui32Value, 1);

				//
				// This command has been handled.
				//
				break;
			}

			//
			// Handle the %d and %i commands.
			//
			case 'd':
			case 'i':
			{
				//
				// Get the value from the varargs.
				//
				ui32Value = va_arg(vaArgP, uint32_t);

				//
				// Reset the buffer position.
				//
				ui32Pos = 0;

				//
				// If the value is negative, make it positive and indicate
				// that a minus sign is needed.
				//
				if((int32_t)ui32Value < 0)
				{
					//
					// Make the value positive.
					//
					ui32Value = -(int32_t)ui32Value;

					//
					// Indicate that the value is negative.
					//
					ui32Neg = 1;
				}
				else
				{
					//
					// Indicate that the value is positive so that a minus
					// sign isn't inserted.
					//
					ui32Neg = 0;
				}

				//
				// Set the base to 10.
				//
				ui32Base = 10;

				//
				// Convert the value to ASCII.
				//
				goto convert;
			}

			//
			// Handle the %s command.
			//
			case 's':
			{
				//
				// Get the string pointer from the varargs.
				//
				pcStr = va_arg(vaArgP, char *);

				//
				// Determine the length of the string.
				//
				for(ui32Idx = 0; pcStr[ui32Idx] != '\0'; ui32Idx++)
				{
				}

				//
				// Write the string.
				//
				UARTwrite(pcStr, ui32Idx);

				//
				// Write any required padding spaces
				//
				if(ui32Count > ui32Idx)
				{
					ui32Count -= ui32Idx;
					while(ui32Count--)
					{
						UARTwrite(" ", 1);
					}
				}

				//
				// This command has been handled.
				//
				break;
			}

			//
			// Handle the %u command.
			//
			case 'u':
			{
				//
				// Get the value from the varargs.
				//
				ui32Value = va_arg(vaArgP, uint32_t);

				//
				// Reset the buffer position.
				//
				ui32Pos = 0;

				//
				// Set the base to 10.
				//
				ui32Base = 10;

				//
				// Indicate that the value is positive so that a minus sign
				// isn't inserted.
				//
				ui32Neg = 0;

				//
				// Convert the value to ASCII.
				//
				goto convert;
			}

			//
			// Handle the %x and %X commands.  Note that they are treated
			// identically; in other words, %X will use lower case letters
			// for a-f instead of the upper case letters it should use.  We
			// also alias %p to %x.
			//
			case 'x':
			case 'X':
			case 'p':
			{
				//
				// Get the value from the varargs.
				//
				ui32Value = va_arg(vaArgP, uint32_t);

				//
				// Reset the buffer position.
				//
				ui32Pos = 0;

				//
				// Set the base to 16.
				//
				ui32Base = 16;

				//
				// Indicate that the value is positive so that a minus sign
				// isn't inserted.
				//
				ui32Neg = 0;

				//
				// Determine the number of digits in the string version of
				// the value.
				//
				convert:
				for(ui32Idx = 1;
						(((ui32Idx * ui32Base) <= ui32Value) &&
								(((ui32Idx * ui32Base) / ui32Base) == ui32Idx));
						ui32Idx *= ui32Base, ui32Count--)
				{
				}

				//
				// If the value is negative, reduce the count of padding
				// characters needed.
				//
				if(ui32Neg)
				{
					ui32Count--;
				}

				//
				// If the value is negative and the value is padded with
				// zeros, then place the minus sign before the padding.
				//
				if(ui32Neg && (cFill == '0'))
				{
					//
					// Place the minus sign in the output buffer.
					//
					pcBuf[ui32Pos++] = '-';

					//
					// The minus sign has been placed, so turn off the
					// negative flag.
					//
					ui32Neg = 0;
				}

				//
				// Provide additional padding at the beginning of the
				// string conversion if needed.
				//
				if((ui32Count > 1) && (ui32Count < 16))
				{
					for(ui32Count--; ui32Count; ui32Count--)
					{
						pcBuf[ui32Pos++] = cFill;
					}
				}

				//
				// If the value is negative, then place the minus sign
				// before the number.
				//
				if(ui32Neg)
				{
					//
					// Place the minus sign in the output buffer.
					//
					pcBuf[ui32Pos++] = '-';
				}

				//
				// Convert the value into a string.
				//
				for(; ui32Idx; ui32Idx /= ui32Base)
				{
					pcBuf[ui32Pos++] =
							g_pcHex[(ui32Value / ui32Idx) % ui32Base];
				}

				//
				// Write the string.
				//
				UARTwrite(pcBuf, ui32Pos);

				//
				// This command has been handled.
				//
				break;
			}

			//
			// Handle the %% command.
			//
			case '%':
			{
				//
				// Simply write a single %.
				//
				UARTwrite(pcString - 1, 1);

				//
				// This command has been handled.
				//
				break;
			}

			//
			// Handle all other commands.
			//
			default:
			{
				//
				// Indicate an error.
				//
				UARTwrite("ERROR", 5);

				//
				// This command has been handled.
				//
				break;
			}
			}
		}
	}
}

//*****************************************************************************
//
//! A simple UART based printf function supporting \%c, \%d, \%p, \%s, \%u,
//! \%x, and \%X.
//!
//! \param pcString is the format string.
//! \param ... are the optional arguments, which depend on the contents of the
//! format string.
//!
//! This function is very similar to the C library <tt>fprintf()</tt> function.
//! All of its output will be sent to the UART.  Only the following formatting
//! characters are supported:
//!
//! - \%c to print a character
//! - \%d or \%i to print a decimal value
//! - \%s to print a string
//! - \%u to print an unsigned decimal value
//! - \%x to print a hexadecimal value using lower case letters
//! - \%X to print a hexadecimal value using lower case letters (not upper case
//! letters as would typically be used)
//! - \%p to print a pointer as a hexadecimal value
//! - \%\% to print out a \% character
//!
//! For \%s, \%d, \%i, \%u, \%p, \%x, and \%X, an optional number may reside
//! between the \% and the format character, which specifies the minimum number
//! of characters to use for that value; if preceded by a 0 then the extra
//! characters will be filled with zeros instead of spaces.  For example,
//! ``\%8d'' will use eight characters to print the decimal value with spaces
//! added to reach eight; ``\%08d'' will use eight characters as well but will
//! add zeroes instead of spaces.
//!
//! The type of the arguments after \e pcString must match the requirements of
//! the format string.  For example, if an integer was passed where a string
//! was expected, an error of some kind will most likely occur.
//!
//! \return None.
//
//*****************************************************************************
void
UARTprintf(const char *pcString, ...)
{
	va_list vaArgP;

	//
	// Start the varargs processing.
	//
	va_start(vaArgP, pcString);

	UARTvprintf(pcString, vaArgP);

	//
	// We're finished with the varargs now.
	//
	va_end(vaArgP);
}

//*****************************************************************************
//
//! Returns the number of bytes available in the receive buffer.
//!
//! This function, available only when the module is built to operate in
//! buffered mode using \b UART_BUFFERED, may be used to determine the number
//! of bytes of data currently available in the receive buffer.
//!
//! \return Returns the number of available bytes.
//
//*****************************************************************************

int
UARTRxBytesAvail(void)
{
	return((uxQueueMessagesWaiting(xRxedChars)));
}



//*****************************************************************************
//
//! Returns the number of bytes free in the transmit buffer.
//!
//! This function, available only when the module is built to operate in
//! buffered mode using \b UART_BUFFERED, may be used to determine the amount
//! of space currently available in the transmit buffer.
//!
//! \return Returns the number of free bytes.
//
//*****************************************************************************
int
UARTTxBytesFree(void)
{
	return(uxQueueSpacesAvailable(xCharsForTx));
}




//*****************************************************************************
//
//! Flushes the receive buffer.
//!
//! This function, available only when the module is built to operate in
//! buffered mode using \b UART_BUFFERED, may be used to discard any data
//! received from the UART but not yet read using UARTgets().
//!
//! \return None.
//
//*****************************************************************************
void
UARTFlushRx(void)
{
	//
	// Flush the receive buffer.
	//
	xQueueReset( xRxedChars );

}


//*****************************************************************************
//
//! Flushes the transmit buffer.
//!
//! \param bDiscard indicates whether any remaining data in the buffer should
//! be discarded (\b true) or transmitted (\b false).
//!
//! This function, available only when the module is built to operate in
//! buffered mode using \b UART_BUFFERED, may be used to flush the transmit
//! buffer, either discarding or transmitting any data received via calls to
//! UARTprintf() that is waiting to be transmitted.  On return, the transmit
//! buffer will be empty.
//!
//! \return None.
//
//*****************************************************************************

void UARTFlushTx(bool bDiscard)
{
	//
	// Should the remaining data be discarded or transmitted?
	//
	if(bDiscard)
	{
		xQueueReset( xCharsForTx );

	}
	else
	{
		//
		// Wait for all remaining data to be transmitted before returning.
		//
		while(uxQueueMessagesWaiting(xCharsForTx))
		{
		}
	}
}


//*****************************************************************************
//
//! Enables or disables echoing of received characters to the transmitter.
//!
//! \param bEnable must be set to \b true to enable echo or \b false to
//! disable it.
//!
//! This function, available only when the module is built to operate in
//! buffered mode using \b UART_BUFFERED, may be used to control whether or not
//! received characters are automatically echoed back to the transmitter.  By
//! default, echo is enabled and this is typically the desired behavior if
//! the module is being used to support a serial command line.  In applications
//! where this module is being used to provide a convenient, buffered serial
//! interface over which application-specific binary protocols are being run,
//! however, echo may be undesirable and this function can be used to disable
//! it.
//!
//! \return None.
//
//*****************************************************************************

void
UARTEchoSet(bool bEnable)
{
	g_bDisableEcho = !bEnable;
}


//*****************************************************************************
//
//! Handles UART interrupts.
//!
//! This function handles interrupts from the UART.  It will copy data from the
//! transmit buffer to the UART transmit FIFO if space is available, and it
//! will copy data from the UART receive FIFO to the receive buffer if data is
//! available.
//!
//! \return None.
//
//*****************************************************************************
//JMCG: Modificado de forma que la ISR quede lo m�s corta posible y utilice las colas de mensaje. El procesado de los datos recibido
// lo completa la funcion UARTgetc o UARTgets, que se ejecutar�n desde una tarea.


void UARTStdioIntHandler(void)
{
	uint32_t ui32Ints;
	int8_t cChar;
	int32_t i32Char;
	portBASE_TYPE xHigherPriorityTaskWoken=false;


	//
	// Get and clear the current interrupt source(s)
	//
	ui32Ints = MAP_UARTIntStatus(g_ui32Base, true);
	MAP_UARTIntClear(g_ui32Base, ui32Ints);

	//
	// Are we being interrupted because the TX FIFO has space available?
	//
	if(ui32Ints & UART_INT_TX)
	{
		//
		// Move as many bytes as we can into the transmit FIFO.
		//
		while(MAP_UARTSpaceAvail(g_ui32Base) && !xQueueIsQueueEmptyFromISR(xCharsForTx))
		{
			uint8_t data;
			xQueueReceiveFromISR(xCharsForTx,&data,&xHigherPriorityTaskWoken);
			MAP_UARTCharPutNonBlocking(g_ui32Base,data);
		}

		//
		// If the output buffer is empty, turn off the transmit interrupt.
		//
		if(xQueueIsQueueEmptyFromISR(xCharsForTx))
		{
			MAP_UARTIntDisable(g_ui32Base, UART_INT_TX);
		}
	}

	//
	// Are we being interrupted due to a received character?
	//
	if(ui32Ints & (UART_INT_RX | UART_INT_RT))
	{
		//
		// Get all the available characters from the UART.
		//
		while(MAP_UARTCharsAvail(g_ui32Base))
		{
			//
			// Read a character
			//
			i32Char = MAP_UARTCharGetNonBlocking(g_ui32Base);
			cChar = (unsigned char)(i32Char & 0xFF);

			//
			// If there is space in the receive buffer, put the character
			// there, otherwise throw it away.
			//

			xQueueSendFromISR(xRxedChars,&cChar,&xHigherPriorityTaskWoken);
		}
	}

	portYIELD_FROM_ISR( xHigherPriorityTaskWoken );

}




//*****************************************************************************
//
// Close the Doxygen group.
//! @}
//
//*****************************************************************************
