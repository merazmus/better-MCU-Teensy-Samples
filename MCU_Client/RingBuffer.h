#ifndef RINGBUFFER_H
#define RINGBUFFER_H

#include <inttypes.h>
#include <stddef.h>
#include <stdint.h>

#include "Config.h"

typedef struct RingBuffer_Tag
{
    uint8_t *p_buf;
    size_t   buf_len;
    size_t   wr;
    size_t   rd;
} RingBuffer_T;

/*
 *  Initialize ring buffer.
 *
 *  @param p_ring_buffer  Pointer to ring buffer instance @def RingBuffer_T
 *  @param bufPointer     Pointer to initialized uint8_t[] buffer
 *  @param bufLen         Length of the buffer.
 *  @return               void
 */
void RingBuffer_Init(RingBuffer_T *p_ring_buffer, uint8_t *bufPointer, size_t bufLen);

/*
 *  Get information if any bytes are present in the ring buffer.
 *
 *  @param ring_buffer    Pointer to ring buffer instance @def RingBuffer_T
 *  @return               True if it's empty, false otherwise
 */
bool RingBuffer_isEmpty(RingBuffer_T *ring_buffer);

/*
 *  Write bytes from table to ring buffer
 *
 *  @param p_ring_buffer  Pointer to ring buffer instance @def RingBuffer_T
 *  @param table          pointer to table with bytes to be queued
 *  @param table_len      length of table
 *  @return               True if success, false if queue this table could cause
 *                        overflow (table is not queued) 
 */
bool RingBuffer_QueueBytes(RingBuffer_T *p_ring_buffer, uint8_t *table, uint16_t table_len);

/*
 *  Get byte from ring buffer.
 *
 *  @param p_ring_buffer    Pointer to ring buffer instance @def RingBuffer_T
 *  @return                 True if success, false if empty
 */
bool RingBuffer_DequeueByte(RingBuffer_T *p_ring_buffer, uint8_t *read_byte);

/*
 *  Inform Ring Buffer how many bytes were queued to it without using
 *  RingBuffer_QueueBytes (needed for DMA). 
 *
 *  @param p_ring_buffer  Pointer to ring buffer instance @def RingBuffer_T
 *  @param value          number of queued bytes
 *  @return               True if success, false if buffer is overflowed
 */
bool RingBuffer_IncrementWrIndex(RingBuffer_T *p_ring_buffer, uint16_t value);

/*
 *  Inform Ring Buffer how many bytes were dequeued from it without using
 *  RingBuffer_DequeueByte (needed for DMA).
 *
 *  @param p_ring_buffer  Pointer to ring buffer instance @def RingBuffer_T
 *  @param value          number of dequeued bytes
 *  @return               True if success, false otherwise
 */
void RingBuffer_IncrementRdIndex(RingBuffer_T *p_ring_buffer, uint16_t value);

/*
 *  Get information about length of queued data.
 *
 *  @param p_ring_buffer  Pointer to ring buffer instance @def RingBuffer_T
 *  @return               length of data
 */
uint16_t RingBuffer_DataLen(RingBuffer_T *p_ring_buffer);

/*
 *  Get pointer to first element of buffer, and maximum length that could be
 *  sent like normal buffer
 *
 *  @param p_ring_buffer  Pointer to ring buffer instance @def RingBuffer_T
 *  @param buf_len        output, length of continuous buffer
 * 
 *  @return               pointer to first element of buffer
 */
uint8_t *RingBuffer_GetMaxContinuousBuffer(RingBuffer_T *p_ring_buffer, uint16_t *buf_len);

#endif    //RINGBUFFER_H
