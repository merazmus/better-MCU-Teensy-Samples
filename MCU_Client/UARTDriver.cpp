#include "UARTDriver.h"

#include <DMAChannel.h>

#include "Config.h"
#include "RingBuffer.h"
#include "kinetis.h"

#define RX_BUFFER_LEN 256
#define TX_BUFFER_LEN 1024

#define C2_RX_ENABLE (UART_C2_TE | UART_C2_RE | UART_C2_RIE)
#define C2_TX_ACTIVE (UART_C2_TIE)
#define C2_TX_INACTIVE (~C2_TX_ACTIVE)
#define C4_UART_DMA_ENABLED (UART_C5_TDMAS | UART_C5_RDMAS)

static DMAChannel rx_dma;
static DMAChannel tx_dma;

static RingBuffer_T dma_rx_buffer;
static RingBuffer_T dma_tx_buffer;

/*
 *  According to http://cache.freescale.com/files/microcontrollers/doc/ref_manual/KL26P121M48SF4RM.pdf
 *  page 380, DMA buffers must be aligned to a 0-modulo-(circular buffer size) boundary.
 */
static __attribute__((section(".dmabuffers"), aligned(TX_BUFFER_LEN))) uint8_t tx_buf[TX_BUFFER_LEN];
static __attribute__((section(".dmabuffers"), aligned(RX_BUFFER_LEN))) uint8_t rx_buf[RX_BUFFER_LEN];

static uint32_t last_bcr_value     = 0;
static uint16_t cur_tx_message_len = 0;

static void DMA_TransmitRequest();
static void DMA_OnTXCompletion();
static void DMA_OnRXCompletion();
static bool IsTXActive();

void UARTDriver_Init()
{
    // Connect clock to UART1
    SIM_SCGC4 |= SIM_SCGC4_UART1;

    // Setup baud rate
    uint32_t baud_rate = BAUD2DIV2(UART_INTERFACE_BAUDRATE);
    UART1_BDH          = (baud_rate >> 8) & 0x1F;
    UART1_BDL          = baud_rate & 0xFF;

    // Initialize pins
    CORE_PIN9_CONFIG  = PORT_PCR_PE | PORT_PCR_PS | PORT_PCR_PFE | PORT_PCR_MUX(3);
    CORE_PIN10_CONFIG = PORT_PCR_DSE | PORT_PCR_SRE | PORT_PCR_MUX(3);

    // TX DMA configuration
    RingBuffer_Init(&dma_tx_buffer, tx_buf, TX_BUFFER_LEN);
    tx_dma.destination(UART1_D);
    tx_dma.interruptAtCompletion();
    tx_dma.disableOnCompletion();
    tx_dma.attachInterrupt(DMA_OnTXCompletion);
    tx_dma.triggerAtHardwareEvent(DMAMUX_SOURCE_UART1_TX);

    // RX DMA configuration
    RingBuffer_Init(&dma_rx_buffer, rx_buf, RX_BUFFER_LEN);
    rx_dma.source(UART1_D);
    rx_dma.destinationCircular(rx_buf, RX_BUFFER_LEN);
    rx_dma.transferCount(DMA_DSR_BCR_BCR(0x000FFFFF));
    rx_dma.attachInterrupt(DMA_OnRXCompletion);
    rx_dma.interruptAtCompletion();
    rx_dma.triggerAtHardwareEvent(DMAMUX_SOURCE_UART1_RX);
    rx_dma.enable();
    last_bcr_value = DMA_DSR_BCR_BCR(DMA_DSR_BCR0);    // BCR value is needed for counting received bytes

    // UART Register settings
    UART1_C1 = 0;
    UART1_C2 = C2_RX_ENABLE;
    UART1_MA1 |= C4_UART_DMA_ENABLED;    // UART1_MA1 is UART1_C4 register (bug with address mapping in teensyduino libraries)
}

bool UARTDriver_WriteBytes(uint8_t *table, uint16_t table_len)
{
    if (!RingBuffer_QueueBytes(&dma_tx_buffer, table, table_len))
    {
        return false;
    }

    DMA_TransmitRequest();
    return true;
}

void DMA_TransmitRequest()
{
    if (IsTXActive())
    {
        return;
    }

    uint8_t *tx_begin_pointer = RingBuffer_GetMaxContinuousBuffer(&dma_tx_buffer, &cur_tx_message_len);
    if (cur_tx_message_len == 0)
    {
        return;
    }
    tx_dma.sourceBuffer(tx_begin_pointer, cur_tx_message_len);
    UART1_C2 |= C2_TX_ACTIVE;
    tx_dma.enable();
}

bool UARTDriver_ReadByte(uint8_t *read_byte)
{
    return RingBuffer_DequeueByte(&dma_rx_buffer, read_byte);
}

void UARTDriver_RxDMAPoll()
{
    uint32_t cur_bcr_value        = DMA_DSR_BCR_BCR(DMA_DSR_BCR0);
    uint16_t received_bytes_value = last_bcr_value - cur_bcr_value;
    if (received_bytes_value == 0)
    {
        return;
    }

    RingBuffer_IncrementWrIndex(&dma_rx_buffer, received_bytes_value);
    last_bcr_value = cur_bcr_value;
}

static bool IsTXActive()
{
    return (UART1_C2 & C2_TX_ACTIVE);
}

static void DMA_OnTXCompletion()
{
    tx_dma.clearInterrupt();
    UART1_C2 &= C2_TX_INACTIVE;
    RingBuffer_IncrementRdIndex(&dma_tx_buffer, cur_tx_message_len);

    if (!RingBuffer_isEmpty(&dma_tx_buffer))
    {
        DMA_TransmitRequest();
    }
}

static void DMA_OnRXCompletion()
{
    rx_dma.clearInterrupt();
    rx_dma.clearComplete();
    rx_dma.disable();
    rx_dma.transferCount(DMA_DSR_BCR_BCR(0x000FFFFF));
    last_bcr_value = DMA_DSR_BCR_BCR(DMA_DSR_BCR0);
    rx_dma.enable();
}