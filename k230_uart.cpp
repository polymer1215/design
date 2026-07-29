#include "k230_uart.hpp"

#include "ti_msp_dl_config.h"

namespace {

constexpr std::uint16_t kReceiveBufferSize = 512U;
constexpr std::uint16_t kReceiveBufferMask = kReceiveBufferSize - 1U;
static_assert(
    (kReceiveBufferSize & kReceiveBufferMask) == 0U,
    "UART receive buffer size must be a power of two");

volatile std::uint8_t g_receiveBuffer[kReceiveBufferSize];
volatile std::uint16_t g_receiveHead = 0U;
volatile std::uint16_t g_receiveTail = 0U;
volatile std::uint32_t g_receivedBytes = 0U;
volatile std::uint32_t g_transmittedBytes = 0U;
volatile std::uint32_t g_droppedBytes = 0U;

void storeReceivedByte(std::uint8_t data)
{
    const std::uint16_t head = g_receiveHead;
    const std::uint16_t next =
        static_cast<std::uint16_t>((head + 1U) & kReceiveBufferMask);

    ++g_receivedBytes;
    if (next == g_receiveTail) {
        ++g_droppedBytes;
        return;
    }

    g_receiveBuffer[head] = data;
    g_receiveHead = next;
}

}  // namespace

namespace K230Uart {

void init()
{
    /*
     * RX FIFO interrupts are transition-triggered. Stop both interrupt stages
     * before resetting the software queue so a frame received during boot
     * cannot leave a stale FIFO-threshold interrupt behind.
     */
    NVIC_DisableIRQ(UART_K230_INST_INT_IRQN);
    DL_UART_Main_disableInterrupt(
        UART_K230_INST, DL_UART_MAIN_INTERRUPT_RX);

    g_receiveHead = 0U;
    g_receiveTail = 0U;
    g_receivedBytes = 0U;
    g_transmittedBytes = 0U;
    g_droppedBytes = 0U;

    DL_UART_Main_clearInterruptStatus(
        UART_K230_INST, DL_UART_MAIN_INTERRUPT_RX);
    NVIC_ClearPendingIRQ(UART_K230_INST_INT_IRQN);

    /*
     * Preserve bytes that arrived after SYSCFG enabled UART2 but before this
     * function ran. Draining below the threshold also rearms the next
     * transition-triggered RX interrupt.
     */
    std::uint8_t data;
    while (DL_UART_Main_receiveDataCheck(UART_K230_INST, &data)) {
        storeReceivedByte(data);
    }

    DL_UART_Main_enableInterrupt(
        UART_K230_INST, DL_UART_MAIN_INTERRUPT_RX);
    NVIC_EnableIRQ(UART_K230_INST_INT_IRQN);
}

bool read(std::uint8_t &data)
{
    const std::uint16_t tail = g_receiveTail;
    if (tail == g_receiveHead) {
        return false;
    }

    data = g_receiveBuffer[tail];
    g_receiveTail =
        static_cast<std::uint16_t>((tail + 1U) & kReceiveBufferMask);
    return true;
}

void write(std::uint8_t data)
{
    DL_UART_Main_transmitDataBlocking(UART_K230_INST, data);
    ++g_transmittedBytes;
}

void write(const std::uint8_t *data, std::size_t length)
{
    if (data == nullptr) {
        return;
    }

    for (std::size_t i = 0U; i < length; ++i) {
        write(data[i]);
    }
}

std::size_t serviceEcho(std::size_t maxBytes)
{
    std::size_t count = 0U;
    std::uint8_t data;
    while (count < maxBytes && read(data)) {
        write(data);
        ++count;
    }
    return count;
}

Statistics statistics()
{
    Statistics result;

    __disable_irq();
    result.receivedBytes = g_receivedBytes;
    result.transmittedBytes = g_transmittedBytes;
    result.droppedBytes = g_droppedBytes;
    __enable_irq();

    return result;
}

}  // namespace K230Uart

extern "C" void UART_K230_INST_IRQHandler(void)
{
    if (DL_UART_Main_getPendingInterrupt(UART_K230_INST) !=
        DL_UART_MAIN_IIDX_RX) {
        return;
    }

    std::uint8_t data;
    while (DL_UART_Main_receiveDataCheck(UART_K230_INST, &data)) {
        storeReceivedByte(data);
    }
}
