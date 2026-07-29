#ifndef SPEED_CONTROLLER_HPP
#define SPEED_CONTROLLER_HPP

#include <cstdint>

namespace SpeedControl {

constexpr float kDefaultKp = 4.0F;
constexpr float kDefaultKi = 6.0F;
constexpr float kDefaultKd = 0.0F;
constexpr float kDefaultTargetRpm = 100.0F;
constexpr std::int16_t kOutputLimit = 700;

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
void setTargetRpm(float leftRpm, float rightRpm);
void setTunings(float kp, float ki, float kd);
void stop();
Status latest();

/* Called by the 100 Hz encoder sampling ISR. */
void updateFromEncoder(
    std::int32_t leftDeltaCounts, std::int32_t rightDeltaCounts);

}  // namespace SpeedControl

#endif
