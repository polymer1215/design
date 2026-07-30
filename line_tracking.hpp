#ifndef LINE_TRACKING_HPP
#define LINE_TRACKING_HPP

#include <cstdint>

namespace LineTracking {

constexpr std::int16_t kDefaultBaseCommand = 200;
constexpr float kDefaultKp = 55.0F;
constexpr float kDefaultKi = 0.0F;
// 0.1 s at 100 Hz is equivalent to the previous per-sample Kd of 10.
constexpr float kDefaultKd = 0.03F;
constexpr std::int16_t kCorrectionLimit = 500;
constexpr std::uint8_t kWideLineSensorCount = 3U;
constexpr std::uint8_t kCenterSensorMask = 0x18U;

/*
 * The line controller is enabled after init(). Set this to false to coast
 * both motors while leaving the sensor and encoder sampling active.
 */
extern volatile bool enabled;

struct Status {
    // Unfiltered active-low result, with OUT1 in bit 7 and OUT8 in bit 0.
    std::uint8_t detectedLineMask;
    // Mask actually passed to the PID error lookup.
    std::uint8_t lineMask;
    std::uint8_t blackSensorCount;
    bool wideLineDetected;
    std::int8_t error;
    std::int16_t correction;
    std::int16_t leftCommand;
    std::int16_t rightCommand;
};

void init();
void setTunings(float kp, float ki, float kd);
void setBaseCommand(std::int16_t command);
void stop();
void update(std::uint32_t sampleSequence);
Status latest();

}  // namespace LineTracking

#endif  // LINE_TRACKING_HPP
