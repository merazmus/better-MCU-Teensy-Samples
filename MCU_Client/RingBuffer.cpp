#include "RingBuffer.h"

static bool     IsOverflow(RingBuffer_T *p_ring_buffer, uint16_t len);
static uint16_t MaxQueueBufferLen(RingBuffer_T *p_ring_buffer, uint16_t table_len);
static bool     IsFull(RingBuffer_T *p_ring_buffer);

void RingBuffer_Init(RingBuffer_T *p_ring_buffer, uint8_t *p_buf_pointer, size_t buf_len)
{
    p_ring_buffer->p_buf   = p_buf_pointer;
    p_ring_buffer->buf_len = buf_len;
    p_ring_buffer->wr      = 0;
    p_ring_buffer->rd      = 0;
}

bool RingBuffer_isEmpty(RingBuffer_T *p_ring_buffer)
{
    return p_ring_buffer->wr == p_ring_buffer->rd;
}

bool RingBuffer_IncrementWrIndex(RingBuffer_T *p_ring_buffer, uint16_t value)
{
    if (IsOverflow(p_ring_buffer, value))
    {
        return false;
    }
    p_ring_buffer->wr = (p_ring_buffer->wr + value) % p_ring_buffer->buf_len;
    return true;
}

void RingBuffer_IncrementRdIndex(RingBuffer_T *p_ring_buffer, uint16_t value)
{
    p_ring_buffer->rd = (p_ring_buffer->rd + value) % p_ring_buffer->buf_len;
}

bool RingBuffer_DequeueByte(RingBuffer_T *p_ring_buffer, uint8_t *read_byte)
{
    if (RingBuffer_isEmpty(p_ring_buffer))
    {
        return false;
    }
    *read_byte = p_ring_buffer->p_buf[(p_ring_buffer->rd)++];

    if (p_ring_buffer->rd >= p_ring_buffer->buf_len)
    {
        p_ring_buffer->rd = 0;
    }

    return true;
}

bool RingBuffer_QueueBytes(RingBuffer_T *p_ring_buffer, uint8_t *table, uint16_t table_len)
{
    if (IsOverflow(p_ring_buffer, table_len))
    {
        return false;
    }

    uint16_t cpy_len = MaxQueueBufferLen(p_ring_buffer, table_len);
    memcpy(&p_ring_buffer->p_buf[p_ring_buffer->wr], table, cpy_len);

    if (cpy_len != table_len)
    {
        uint16_t rest_of_table_len = table_len - cpy_len;
        memcpy(p_ring_buffer->p_buf, &table[cpy_len], rest_of_table_len);
    }

    RingBuffer_IncrementWrIndex(p_ring_buffer, table_len);

    return true;
}

uint8_t *RingBuffer_GetMaxContinuousBuffer(RingBuffer_T *p_ring_buffer, uint16_t *buf_len)
{
    if (p_ring_buffer->rd + RingBuffer_DataLen(p_ring_buffer) > p_ring_buffer->buf_len)
    {
        *buf_len = p_ring_buffer->buf_len - p_ring_buffer->rd;
    }
    else
    {
        *buf_len = RingBuffer_DataLen(p_ring_buffer);
    }

    return &p_ring_buffer->p_buf[p_ring_buffer->rd];
}

uint16_t RingBuffer_DataLen(RingBuffer_T *p_ring_buffer)
{
    return (p_ring_buffer->wr - p_ring_buffer->rd) % p_ring_buffer->buf_len;
}

static bool IsOverflow(RingBuffer_T *p_ring_buffer, uint16_t len)
{
    return (len + RingBuffer_DataLen(p_ring_buffer)) > p_ring_buffer->buf_len;
}

static uint16_t MaxQueueBufferLen(RingBuffer_T *p_ring_buffer, uint16_t table_len)
{
    if (IsOverflow(p_ring_buffer, table_len))
    {
        return 0;
    }

    if (p_ring_buffer->wr + table_len > p_ring_buffer->buf_len)
    {
        return p_ring_buffer->buf_len - p_ring_buffer->wr;
    }
    else
    {
        return table_len;
    }
}

static bool IsFull(RingBuffer_T *p_ring_buffer)
{
    return ((p_ring_buffer->wr + 1) % p_ring_buffer->buf_len) == p_ring_buffer->rd;
}
