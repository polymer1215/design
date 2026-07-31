#ifndef SPEED_CONTROLLER_HPP
#define SPEED_CONTROLLER_HPP

#include <cstdint>

namespace SpeedControl {

constexpr float kDefaultKp = 11.1F;
constexpr float kDefaultKi = 70.0F;
constexpr float kDefaultKd = 0.0F;
constexpr float kDefaultTargetRpm = 100.0F;
constexpr std::int16_t kOutputLimit = 800;

/*
 * Runtime PID switch. Set to false (0) to stop both motor outputs while
 * keeping encoder speed measurement active.
 */
extern volatile bool pidEnabled;

struct Status {
    float leftTargetRpm;
    float rightTargetRpm;
    float leftMeasuredRpm;
    float rightMeasuredRpm;
    std::int16_t leftOutput;
    std::int16_t rightOutput;
};

void init();
// Positive RPM selects the vehicle-forward direction defined by TB6612.
void setTargetRpm(float leftRpm, float rightRpm);
void setTunings(float kp, float ki, float kd);
void stop();
// Hold both TB6612 channels in active short-brake mode until a new target or
// stop command releases the brake.
void brake();
Status latest();

/* Called by the 100 Hz encoder sampling ISR. */
void updateFromEncoder(
    std::int32_t leftDeltaCounts, std::int32_t rightDeltaCounts);

}  // namespace SpeedControl

#endif
