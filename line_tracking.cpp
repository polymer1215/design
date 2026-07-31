#include "line_tracking.hpp"

#include "encoder.hpp"
#include "gray_sensor.hpp"
#include "pid_controller.hpp"
#include "speed_controller.hpp"
#include "ti_msp_dl_config.h"

namespace {

constexpr float kControlPeriodSeconds =
    1.0F / static_cast<float>(Encoder::kSampleRateHz);

float g_baseRpm = LineTracking::kDefaultBaseRpm;

Control::PidController g_steeringPid;
std::uint32_t g_lastSequence = 0;
std::uint32_t g_lastPidSequence = 0;
LineTracking::Status g_status = {};

float clampValue(float value, float minimum, float maximum)
{
    if (value > maximum) {
        return maximum;
    }
    if (value < minimum) {
        return minimum;
    }
    return value;
}

std::uint8_t reverseBits(std::uint8_t value)
{
    value = static_cast<std::uint8_t>(
        ((value & 0x55U) << 1U) | ((value & 0xAAU) >> 1U));
    value = static_cast<std::uint8_t>(
        ((value & 0x33U) << 2U) | ((value & 0xCCU) >> 2U));
    return static_cast<std::uint8_t>((value << 4U) | (value >> 4U));
}

std::uint8_t makeLineMask(std::uint8_t rawPins)
{
    /*
     * The sensor outputs are open-drain and active-low. GraySensor packs
     * OUT1 in bit 0, while the reference algorithm packs OUT1 in bit 7.
     */
    return reverseBits(static_cast<std::uint8_t>(~rawPins));
}

std::uint8_t countSetBits(std::uint8_t mask)
{
    std::uint8_t count = 0U;
    while (mask != 0U) {
        count += mask & 1U;
        mask >>= 1U;
    }
    return count;
}

float weightedErrorForMask(std::uint8_t mask)
{
    constexpr float kSensorWeights[8] = {
        6.0F, 4.0F, 2.0F, 0.0F,
        -0.0F, -2.0F, -4.0F, -6.0F
    };
    float weightSum = 0.0F;
    std::uint8_t activeCount = 0U;

    for (std::uint8_t channel = 0U; channel < 8U; ++channel) {
        const std::uint8_t channelBit =
            static_cast<std::uint8_t>(0x80U >> channel);
        if ((mask & channelBit) != 0U) {
            weightSum += kSensorWeights[channel];
            ++activeCount;
        }
    }

    return activeCount == 0U
        ? 0.0F
        : weightSum / static_cast<float>(activeCount);
}

float integralLimitFor(float ki)
{
    const float magnitude = (ki >= 0.0F) ? ki : -ki;
    if (magnitude == 0.0F) {
        return LineTracking::kCorrectionLimitRpm;
    }
    return LineTracking::kCorrectionLimitRpm / magnitude;
}

Control::PidConfig makePidConfig(float kp, float ki, float kd)
{
    const float integralLimit = integralLimitFor(ki);
    return {
        kp,
        ki,
        kd,
        -LineTracking::kCorrectionLimitRpm,
        LineTracking::kCorrectionLimitRpm,
        -integralLimit,
        integralLimit,
        Control::DerivativeMode::Measurement,
    };
}

float computeCorrection(
    float error, std::uint32_t sampleSequence)
{
    std::uint32_t elapsedSamples = 1U;
    if (g_lastPidSequence != 0U) {
        elapsedSamples = sampleSequence - g_lastPidSequence;
        if (elapsedSamples == 0U) {
            elapsedSamples = 1U;
        }
    }
    g_lastPidSequence = sampleSequence;

    const float dtSeconds =
        static_cast<float>(elapsedSamples) * kControlPeriodSeconds;
    return g_steeringPid.update(
        0.0F, -error, dtSeconds);
}

void applySpeedTargets(float correctionRpm)
{
    const float leftTargetRpm =
        clampValue(g_baseRpm - correctionRpm, 0.0F, 2.0F * g_baseRpm);
    const float rightTargetRpm =
        clampValue(g_baseRpm + correctionRpm, 0.0F, 2.0F * g_baseRpm);

    g_status.correctionRpm = correctionRpm;
    g_status.leftTargetRpm = leftTargetRpm;
    g_status.rightTargetRpm = rightTargetRpm;
    SpeedControl::setTargetRpm(leftTargetRpm, rightTargetRpm);
}

void resetOuterLoop(std::uint32_t sampleSequence)
{
    g_steeringPid.reset();
    g_lastPidSequence = sampleSequence;
}

void applyStoppedTargets()
{
    g_status.correctionRpm = 0.0F;
    g_status.leftTargetRpm = 0.0F;
    g_status.rightTargetRpm = 0.0F;
    SpeedControl::setTargetRpm(0.0F, 0.0F);
}

}  // namespace

namespace LineTracking {

volatile bool enabled = true;

void init()
{
    g_baseRpm = kDefaultBaseRpm;
    g_steeringPid.configure(makePidConfig(
        kDefaultKp, kDefaultKi, kDefaultKd));
    g_lastSequence = 0;
    g_lastPidSequence = 0;
    g_status = {};
    enabled = true;
    applySpeedTargets(0.0F);
}

void setTunings(float kp, float ki, float kd)
{
    __disable_irq();
    g_steeringPid.configure(makePidConfig(kp, ki, kd));
    g_lastPidSequence = 0;
    __enable_irq();
}

void setBaseRpm(float rpm)
{
    g_baseRpm = rpm > 0.0F ? rpm : 0.0F;
    applySpeedTargets(g_status.correctionRpm);
}

void stop()
{
    enabled = false;
    resetOuterLoop(g_lastSequence);
    g_status.error = 0.0F;
    g_status.correctionRpm = 0.0F;
    g_status.leftTargetRpm = 0.0F;
    g_status.rightTargetRpm = 0.0F;
    SpeedControl::setTargetRpm(0.0F, 0.0F);
}

void update(std::uint32_t sampleSequence)
{
    if (sampleSequence == g_lastSequence) {
        return;
    }
    g_lastSequence = sampleSequence;

    const std::uint8_t detectedMask =
        makeLineMask(GraySensor::latest().digital);
    const std::uint8_t blackSensorCount = countSetBits(detectedMask);

    g_status.detectedLineMask = detectedMask;
    g_status.lineMask = detectedMask;
    g_status.blackSensorCount = blackSensorCount;
    g_status.wideLineDetected =
        blackSensorCount >= kWideLineSensorCount;

    if (!enabled) {
        resetOuterLoop(sampleSequence);
        g_status.error = 0.0F;
        applyStoppedTargets();
        return;
    }

    if (blackSensorCount == 0U) {
        /*
         * Pause steering updates while the line is lost. Keep the last
         * left/right targets so the vehicle continues with the same motion
         * it had immediately before losing the line. Advancing the PID time
         * reference prevents the lost interval from becoming one large dt
         * when the line is detected again.
         */
        g_lastPidSequence = sampleSequence;
        return;
    }

    g_status.error = weightedErrorForMask(detectedMask);
    applySpeedTargets(
        computeCorrection(g_status.error, sampleSequence));
}

Status latest()
{
    return g_status;
}

}  // namespace LineTracking
