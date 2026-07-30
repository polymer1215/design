#ifndef ENCODER_HPP
#define ENCODER_HPP

#include <cstdint>

namespace Encoder {

constexpr std::uint32_t kSampleRateHz = 100U;
constexpr std::int32_t kMotorLinesPerRevolution = 13;
constexpr std::int32_t kGearRatio = 28;
constexpr std::int32_t kQuadratureMultiplier = 4;
constexpr std::int32_t kCountsPerWheelRevolution =
    kMotorLinesPerRevolution * kGearRatio * kQuadratureMultiplier;
// Match the redefined vehicle-forward direction used by the motor driver.
constexpr std::int32_t kRightEncoderPolarity = 1;
constexpr std::int32_t kLeftEncoderPolarity = -1;

struct Sample {
    std::int32_t rightTotal;
    std::int32_t leftTotal;
    std::int32_t rightDelta;
    std::int32_t leftDelta;
    std::uint32_t rightAEdges;
    std::uint32_t rightBEdges;
    std::uint32_t leftAEdges;
    std::uint32_t leftBEdges;
    std::uint32_t sequence;
};

bool init();
Sample latest();

}  // namespace Encoder

#endif
