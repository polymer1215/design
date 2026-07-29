#ifndef K230_PROTOCOL_HPP
#define K230_PROTOCOL_HPP

#include <cstdint>

namespace K230Protocol {

struct BallPosition {
    bool detected;
    std::int16_t x;
    std::int16_t y;
    std::uint32_t sequence;
};

void reset();

/*
 * Consumes one byte from the UART stream. Returns true only after a complete,
 * valid BALL,x,y line has been decoded and written to position.
 */
bool consume(std::uint8_t data, BallPosition &position);

}  // namespace K230Protocol

#endif
