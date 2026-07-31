#ifndef STEPPER_HPP
#define STEPPER_HPP

#include <cstdint>

namespace Stepper {

constexpr std::uint32_t kFullStepsPerRevolution = 200U;
constexpr std::uint32_t kMicrostepDivision = 16U;
constexpr std::uint32_t kPulsesPerRevolution =
    kFullStepsPerRevolution * kMicrostepDivision;
constexpr std::int32_t kTravelLimitDegrees = 30;
constexpr std::int32_t kTravelLimitPulses =
    static_cast<std::int32_t>(
        (kPulsesPerRevolution * kTravelLimitDegrees + 180U) / 360U);
static_assert(kTravelLimitPulses == 267,
    "The confirmed 16-microstep setup should use 267 pulses for 30 degrees");

constexpr std::uint32_t kMinimumStepFrequencyHz = 10U;
constexpr std::uint32_t kMaximumStepFrequencyHz = 10000U;

struct Status {
    bool enabled;
    bool running;
    bool continuous;
    bool directionPositive;
    std::uint32_t frequencyHz;
    std::uint32_t pulsesRemaining;
    std::int32_t positionPulses;
};

void init();
// Enabling waits 1 ms for D36A wake-up; do not call it from an ISR.
void setEnabled(bool enabled);
bool isEnabled();

// Commands are non-blocking. Invalid frequency commands stop existing motion.
bool moveRelative(std::int32_t pulses, std::uint32_t frequencyHz);
bool moveTo(std::int32_t targetPulses, std::uint32_t frequencyHz);
bool moveToDegrees(float targetDegrees, std::uint32_t frequencyHz);
bool setVelocity(std::int32_t signedFrequencyHz);

void stop();
bool isBusy();

Status status();
std::int32_t positionPulses();
// Redefining the open-loop position stops any active motion and clamps to
// +/-kTravelLimitPulses.
void setPositionPulses(std::int32_t position);

std::int32_t degreesToPulses(float degrees);
float pulsesToDegrees(std::int32_t pulses);

}  // namespace Stepper

#endif
