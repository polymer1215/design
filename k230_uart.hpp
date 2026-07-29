#ifndef K230_UART_HPP
#define K230_UART_HPP

#include <cstddef>
#include <cstdint>

namespace K230Uart {

struct Statistics {
    std::uint32_t receivedBytes;
    std::uint32_t transmittedBytes;
    std::uint32_t droppedBytes;
};

// Enables UART2 RX interrupts after SYSCFG_DL_init().
void init();

// Echoes up to maxBytes of buffered K230 data without interpreting it.
std::size_t serviceEcho(std::size_t maxBytes = 64U);

// Nonblocking raw-byte receive API for later packet processing.
bool read(std::uint8_t &data);

// Blocking raw-byte transmit helpers.
void write(std::uint8_t data);
void write(const std::uint8_t *data, std::size_t length);

Statistics statistics();

}  // namespace K230Uart

#endif
