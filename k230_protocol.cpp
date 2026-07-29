#include "k230_protocol.hpp"

namespace {

constexpr std::uint8_t kLineBufferSize = 24U;
char g_lineBuffer[kLineBufferSize] = {};
std::uint8_t g_lineLength = 0U;
bool g_discardUntilNewline = false;
std::uint32_t g_frameSequence = 0U;

bool parseInteger(
    const char *line, std::uint8_t &index, char terminator,
    std::int16_t &result)
{
    bool negative = false;
    if (line[index] == '-') {
        negative = true;
        ++index;
    }

    std::uint16_t value = 0U;
    std::uint8_t digits = 0U;
    while (line[index] >= '0' && line[index] <= '9') {
        value = static_cast<std::uint16_t>(
            value * 10U + static_cast<std::uint16_t>(line[index] - '0'));
        if (value > 999U) {
            return false;
        }
        ++index;
        ++digits;
    }

    if (digits == 0U || line[index] != terminator) {
        return false;
    }
    if (terminator != '\0') {
        ++index;
    }

    if (negative) {
        if (value != 1U) {
            return false;
        }
        result = -1;
    } else {
        result = static_cast<std::int16_t>(value);
    }
    return true;
}

bool parseBallFrame(
    const char *line, K230Protocol::BallPosition &position)
{
    constexpr char kPrefix[] = "BALL,";
    for (std::uint8_t i = 0U; i < sizeof(kPrefix) - 1U; ++i) {
        if (line[i] != kPrefix[i]) {
            return false;
        }
    }

    std::uint8_t index = sizeof(kPrefix) - 1U;
    std::int16_t x;
    std::int16_t y;
    if (!parseInteger(line, index, ',', x) ||
        !parseInteger(line, index, '\0', y)) {
        return false;
    }

    const bool noTarget = x == -1 && y == -1;
    const bool validTarget =
        x >= 0 && x <= 639 && y >= 0 && y <= 479;
    if (!noTarget && !validTarget) {
        return false;
    }

    position.detected = validTarget;
    position.x = x;
    position.y = y;
    position.sequence = ++g_frameSequence;
    return true;
}

}  // namespace

namespace K230Protocol {

void reset()
{
    g_lineLength = 0U;
    g_discardUntilNewline = false;
    g_frameSequence = 0U;
}

bool consume(std::uint8_t data, BallPosition &position)
{
    if (data == '\r') {
        return false;
    }

    if (data == '\n') {
        if (g_discardUntilNewline) {
            g_discardUntilNewline = false;
            g_lineLength = 0U;
            return false;
        }

        g_lineBuffer[g_lineLength] = '\0';
        const bool decoded = parseBallFrame(g_lineBuffer, position);
        g_lineLength = 0U;
        return decoded;
    }

    if (g_discardUntilNewline) {
        return false;
    }

    if (g_lineLength >= kLineBufferSize - 1U) {
        g_lineLength = 0U;
        g_discardUntilNewline = true;
        return false;
    }

    g_lineBuffer[g_lineLength++] = static_cast<char>(data);
    return false;
}

}  // namespace K230Protocol
