#ifndef GRAY_SENSOR_HPP_
#define GRAY_SENSOR_HPP_

#include <cstdint>

namespace GraySensor {

constexpr std::uint8_t kDefaultAddress = 0x4CU;
constexpr std::uint8_t kChannelCount = 8U;

struct Sample {
    bool connected;
    std::uint8_t digital;
    std::uint8_t analog[kChannelCount];
    std::uint32_t sequence;
    std::uint32_t sampledAt;
    std::uint32_t errors;
};

void init();
void update(std::uint32_t encoderSequence);
Sample latest();

bool ping();
bool readDigital(std::uint8_t &digital);
bool readAnalog(std::uint8_t (&analog)[kChannelCount]);

}  // namespace GraySensor

#endif  // GRAY_SENSOR_HPP_
