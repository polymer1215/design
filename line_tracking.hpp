#ifndef LINE_TRACKING_HPP
#define LINE_TRACKING_HPP

#include <cstdint>

namespace LineTracking {

constexpr float kDefaultBaseRpm = 100.0F;
constexpr float kDefaultKp = 7.5F;
constexpr float kDefaultKi = 0.0F;
constexpr float kDefaultKd = 0.0F;
constexpr float kCorrectionLimitRpm = 100.0F;
constexpr std::uint8_t kWideLineSensorCount = 3U;

/*
 * The line controller is enabled after init(). Set this to false to command
 * zero RPM while leaving the sensor and encoder sampling active.
 */
extern volatile bool enabled;

struct Status {
    // Unfiltered active-low result, with OUT1 in bit 7 and OUT8 in bit 0.
    std::uint8_t detectedLineMask;
    // Complete mask passed to the weighted-position calculation.
    std::uint8_t lineMask;
    std::uint8_t blackSensorCount;
    bool wideLineDetected;
    float error;
    float correctionRpm;
    float leftTargetRpm;
    float rightTargetRpm;
};

void init();
void setTunings(float kp, float ki, float kd);
void setBaseRpm(float rpm);
void stop();
void update(std::uint32_t sampleSequence);
Status latest();

}  // namespace LineTracking

#endif  // LINE_TRACKING_HPP
