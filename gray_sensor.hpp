#ifndef GRAY_SENSOR_HPP_
#define GRAY_SENSOR_HPP_

#include <cstdint>

namespace GraySensor {

constexpr std::uint8_t kChannelCount = 8U;

struct Sample {
    std::uint8_t digital;
    std::uint32_t sequence;
    std::uint32_t sampledAt;
};

void init();
void update(std::uint32_t encoderSequence);
Sample latest();

}  // namespace GraySensor

#endif  // GRAY_SENSOR_HPP_
