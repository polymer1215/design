#include "speed_controller.hpp"

#include "encoder.hpp"
#include "pid_controller.hpp"
#include "tb6612.hpp"
#include "ti_msp_dl_config.h"

namespace {

constexpr float kControlPeriodSeconds =
    1.0F / static_cast<float>(Encoder::kSampleRateHz);
constexpr float kCountsToRpm =
    (60.0F * static_cast<float>(Encoder::kSampleRateHz)) /
    static_cast<float>(Encoder::kCountsPerWheelRevolution);
constexpr std::uint32_t kSpeedMeasurementWindowSamples = 4U;

volatile float g_leftTargetRpm = 0.0F;
volatile float g_rightTargetRpm = 0.0F;
volatile float g_leftMeasuredRpm = 0.0F;
volatile float g_rightMeasuredRpm = 0.0F;
volatile std::int16_t g_leftOutput = 0;
volatile std::int16_t g_rightOutput = 0;
volatile bool g_brakeRequested = false;

std::int32_t g_leftCountHistory[kSpeedMeasurementWindowSamples] = {};
std::int32_t g_rightCountHistory[kSpeedMeasurementWindowSamples] = {};
std::int32_t g_leftWindowCounts = 0;
std::int32_t g_rightWindowCounts = 0;
std::uint32_t g_speedHistoryIndex = 0U;
std::uint32_t g_speedHistorySamples = 0U;

Control::PidController g_leftPid;
Control::PidController g_rightPid;

void resetSpeedMeasurement()
{
    for (std::uint32_t i = 0U;
         i < kSpeedMeasurementWindowSamples; ++i) {
        g_leftCountHistory[i] = 0;
        g_rightCountHistory[i] = 0;
    }
    g_leftWindowCounts = 0;
    g_rightWindowCounts = 0;
    g_speedHistoryIndex = 0U;
    g_speedHistorySamples = 0U;
}

void updateSpeedMeasurement(
    std::int32_t leftDeltaCounts, std::int32_t rightDeltaCounts)
{
    g_leftWindowCounts -= g_leftCountHistory[g_speedHistoryIndex];
    g_rightWindowCounts -= g_rightCountHistory[g_speedHistoryIndex];

    g_leftCountHistory[g_speedHistoryIndex] = leftDeltaCounts;
    g_rightCountHistory[g_speedHistoryIndex] = rightDeltaCounts;
    g_leftWindowCounts += leftDeltaCounts;
    g_rightWindowCounts += rightDeltaCounts;

    g_speedHistoryIndex =
        (g_speedHistoryIndex + 1U) % kSpeedMeasurementWindowSamples;
    if (g_speedHistorySamples < kSpeedMeasurementWindowSamples) {
        ++g_speedHistorySamples;
    }

    const float windowScale =
        kCountsToRpm / static_cast<float>(g_speedHistorySamples);
    g_leftMeasuredRpm =
        static_cast<float>(g_leftWindowCounts) * windowScale;
    g_rightMeasuredRpm =
        static_cast<float>(g_rightWindowCounts) * windowScale;
}

float integralLimitFor(float ki)
{
    const float magnitude = (ki >= 0.0F) ? ki : -ki;
    if (magnitude == 0.0F) {
        return static_cast<float>(SpeedControl::kOutputLimit);
    }
    return static_cast<float>(SpeedControl::kOutputLimit) / magnitude;
}

Control::PidConfig makePidConfig(float kp, float ki, float kd)
{
    const float integralLimit = integralLimitFor(ki);
    return {
        kp,
        ki,
        kd,
        -static_cast<float>(SpeedControl::kOutputLimit),
        static_cast<float>(SpeedControl::kOutputLimit),
        -integralLimit,
        integralLimit,
        Control::DerivativeMode::Measurement,
    };
}

std::int16_t roundCommand(float output)
{
    return static_cast<std::int16_t>(
        output >= 0.0F ? output + 0.5F : output - 0.5F);
}

std::int16_t updatePid(
    Control::PidController &pid, float target, float measurement)
{
    if (target == 0.0F) {
        pid.reset(measurement);
        return 0;
    }
    return roundCommand(
        pid.update(target, measurement, kControlPeriodSeconds));
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
    g_brakeRequested = false;
    resetSpeedMeasurement();
    g_leftPid.configure(makePidConfig(
        kDefaultKp, kDefaultKi, kDefaultKd));
    g_rightPid.configure(makePidConfig(
        kDefaultKp, kDefaultKi, kDefaultKd));
    __enable_irq();
    TB6612::coast();
}

void setTargetRpm(float leftRpm, float rightRpm)
{
    __disable_irq();
    g_leftTargetRpm = leftRpm;
    g_rightTargetRpm = rightRpm;
    g_brakeRequested = false;
    __enable_irq();
}

void setTunings(float kp, float ki, float kd)
{
    __disable_irq();
    const Control::PidConfig config = makePidConfig(kp, ki, kd);
    g_leftPid.configure(config);
    g_rightPid.configure(config);
    __enable_irq();
}

void stop()
{
    setTargetRpm(0.0F, 0.0F);
    TB6612::coast();
}

void brake()
{
    __disable_irq();
    g_leftTargetRpm = 0.0F;
    g_rightTargetRpm = 0.0F;
    g_leftOutput = 0;
    g_rightOutput = 0;
    g_brakeRequested = true;
    __enable_irq();
    TB6612::brake();
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
    updateSpeedMeasurement(leftDeltaCounts, rightDeltaCounts);

    if (g_brakeRequested) {
        g_leftPid.reset(g_leftMeasuredRpm);
        g_rightPid.reset(g_rightMeasuredRpm);
        g_leftOutput = 0;
        g_rightOutput = 0;
        TB6612::brake();
        return;
    }

    if (!pidEnabled) {
        g_leftPid.reset(g_leftMeasuredRpm);
        g_rightPid.reset(g_rightMeasuredRpm);
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
