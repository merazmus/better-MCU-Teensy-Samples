#ifndef UARTDRIVER_H
#define UARTDRIVER_H

#include <inttypes.h>
#include <stddef.h>
#include <stdint.h>

/*
 *  Initialize UART Driver.
 */
void UARTDriver_Init(void);

/*
 *  Write bytes from table to transmit buffer.
 *
 *  @param table        pointer to table with bytes that 
 *                      you want to write to transmit buffer 
 *  @param len          length of table
 *  
 *  @return             False if overflow in TX buffer occured, true otherwise
 */
bool UARTDriver_WriteBytes(uint8_t *table, uint16_t len);

/*
 *  Read Byte from Receive Buffer.
 *
 *  @param read_byte        pointer for received byte 
 *  
 *  @return                 False if RX buffer is empty, true otherwise 
 */
bool UARTDriver_ReadByte(uint8_t *read_byte);

/*
 *  Function for polling received bytes from UART DMA buffer
 */
void UARTDriver_RxDMAPoll(void);

#endif    //UARTDRIVER_H
