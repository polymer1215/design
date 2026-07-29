#include "speed_controller.hpp"

#include "encoder.hpp"
#include "tb6612.hpp"
#include "ti_msp_dl_config.h"

namespace {

constexpr float kControlPeriodSeconds =
    1.0F / static_cast<float>(Encoder::kSampleRateHz);
constexpr float kCountsToRpm =
    (60.0F * static_cast<float>(Encoder::kSampleRateHz)) /
    static_cast<float>(Encoder::kCountsPerWheelRevolution);
constexpr float kSpeedFilterAlpha = 0.25F;

struct PidState {
    float integral;
    float previousMeasurement;
};

volatile float g_leftTargetRpm = 0.0F;
volatile float g_rightTargetRpm = 0.0F;
volatile float g_leftMeasuredRpm = 0.0F;
volatile float g_rightMeasuredRpm = 0.0F;
volatile std::int16_t g_leftOutput = 0;
volatile std::int16_t g_rightOutput = 0;

float g_kp = SpeedControl::kDefaultKp;
float g_ki = SpeedControl::kDefaultKi;
float g_kd = SpeedControl::kDefaultKd;
PidState g_leftPid = {};
PidState g_rightPid = {};

float clampFloat(float value, float minimum, float maximum)
{
    if (value > maximum) {
        return maximum;
    }
    if (value < minimum) {
        return minimum;
    }
    return value;
}

std::int16_t updatePid(
    PidState &state, float target, float measurement)
{
    if (target == 0.0F) {
        state.integral = 0.0F;
        state.previousMeasurement = measurement;
        return 0;
    }

    const float error = target - measurement;
    const float derivative =
        -(measurement - state.previousMeasurement) / kControlPeriodSeconds;
    const float candidateIntegral =
        state.integral + error * kControlPeriodSeconds;

    const float rawOutput =
        g_kp * error + g_ki * candidateIntegral + g_kd * derivative;
    const float limitedOutput = clampFloat(rawOutput,
        -static_cast<float>(SpeedControl::kOutputLimit),
         static_cast<float>(SpeedControl::kOutputLimit));

    /*
     * Conditional integration prevents windup while saturated, but permits
     * integration when the error is driving the output back into range.
     */
    const bool saturatedHigh =
        rawOutput > static_cast<float>(SpeedControl::kOutputLimit);
    const bool saturatedLow =
        rawOutput < -static_cast<float>(SpeedControl::kOutputLimit);
    if ((!saturatedHigh && !saturatedLow) ||
        (saturatedHigh && error < 0.0F) ||
        (saturatedLow && error > 0.0F)) {
        state.integral = candidateIntegral;
    }

    state.previousMeasurement = measurement;
    return static_cast<std::int16_t>(
        limitedOutput >= 0.0F ? limitedOutput + 0.5F
                              : limitedOutput - 0.5F);
}

}  // namespace

namespace SpeedControl {

volatile bool pidEnabled = false;

void init()
{
    __disable_irq();
    g_leftTargetRpm = 0.0F;
    g_rightTargetRpm = 0.0F;
    g_leftMeasuredRpm = 0.0F;
    g_rightMeasuredRpm = 0.0F;
    g_leftOutput = 0;
    g_rightOutput = 0;
    g_leftPid = {};
    g_rightPid = {};
    __enable_irq();
    TB6612::coast();
}

void setTargetRpm(float leftRpm, float rightRpm)
{
    __disable_irq();
    g_leftTargetRpm = leftRpm;
    g_rightTargetRpm = rightRpm;
    __enable_irq();
}

void setTunings(float kp, float ki, float kd)
{
    __disable_irq();
    g_kp = kp;
    g_ki = ki;
    g_kd = kd;
    g_leftPid.integral = 0.0F;
    g_rightPid.integral = 0.0F;
    __enable_irq();
}

void stop()
{
    setTargetRpm(0.0F, 0.0F);
    TB6612::coast();
}

Status latest()
{
    Status status;

    __disable_irq();
    status.leftTargetRpm = g_leftTargetRpm;
    status.rightTargetRpm = g_rightTargetRpm;
    status.leftMeasuredRpm = g_leftMeasuredRpm;
    status.rightMeasuredRpm = g_rightMeasuredRpm;
    status.leftOutput = g_leftOutput;
    status.rightOutput = g_rightOutput;
    __enable_irq();

    return status;
}

void updateFromEncoder(
    std::int32_t leftDeltaCounts, std::int32_t rightDeltaCounts)
{
    const float leftRawRpm =
        static_cast<float>(leftDeltaCounts) * kCountsToRpm;
    const float rightRawRpm =
        static_cast<float>(rightDeltaCounts) * kCountsToRpm;

    g_leftMeasuredRpm +=
        kSpeedFilterAlpha * (leftRawRpm - g_leftMeasuredRpm);
    g_rightMeasuredRpm +=
        kSpeedFilterAlpha * (rightRawRpm - g_rightMeasuredRpm);

    if (!pidEnabled) {
        g_leftPid.integral = 0.0F;
        g_rightPid.integral = 0.0F;
        g_leftPid.previousMeasurement = g_leftMeasuredRpm;
        g_rightPid.previousMeasurement = g_rightMeasuredRpm;
        g_leftOutput = 0;
        g_rightOutput = 0;
        TB6612::coast();
        return;
    }

    g_leftOutput =
        updatePid(g_leftPid, g_leftTargetRpm, g_leftMeasuredRpm);
    g_rightOutput =
        updatePid(g_rightPid, g_rightTargetRpm, g_rightMeasuredRpm);

    TB6612::setSpeeds(g_leftOutput, g_rightOutput);
}

}  // namespace SpeedControl
