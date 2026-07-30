#include "line_tracking.hpp"

#include "encoder.hpp"
#include "gray_sensor.hpp"
#include "pid_controller.hpp"
#include "tb6612.hpp"
#include "ti_msp_dl_config.h"

namespace {

constexpr float kControlPeriodSeconds =
    1.0F / static_cast<float>(Encoder::kSampleRateHz);

std::int16_t g_baseCommand = LineTracking::kDefaultBaseCommand;

Control::PidController g_steeringPid;
std::uint8_t g_candidateMask = 0;
std::uint8_t g_sameMaskCount = 0;
std::uint32_t g_lastSequence = 0;
std::uint32_t g_lastPidSequence = 0;
LineTracking::Status g_status = {};

std::int32_t clampValue(
    std::int32_t value, std::int32_t minimum, std::int32_t maximum)
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

std::uint8_t makeControlMask(
    std::uint8_t detectedMask, std::uint8_t blackSensorCount)
{
    if (blackSensorCount >= LineTracking::kWideLineSensorCount) {
        /*
         * Keep the original detection in Status for the future stop-line
         * decision. During the current drive-through test, only sensors 4
         * and 5 are allowed to influence steering on a wide black region.
         */
        return detectedMask & LineTracking::kCenterSensorMask;
    }
    return detectedMask;
}

bool maskIsStable(std::uint8_t mask)
{
    if (mask != g_candidateMask) {
        g_candidateMask = mask;
        g_sameMaskCount = 1U;
        return false;
    }

    if (g_sameMaskCount < 2U) {
        ++g_sameMaskCount;
    }
    return g_sameMaskCount >= 2U;
}

std::int8_t errorForMask(std::uint8_t mask, std::int8_t previousError)
{
    switch (mask) {
        case 0x00:
        case 0x18:
        case 0x3C:
        case 0x7E:
            return 0;

        case 0x08:
        case 0x10:
            return 1;

        case 0x20:
        case 0x30:
        case 0x38:
            return 2;

        case 0x40:
        case 0x60:
        case 0x78:
        case 0x7C:
            return 4;

        case 0x80:
        case 0xA0:
        case 0xC0:
        case 0xE0:
        case 0xF0:
        case 0xF8:
        case 0xFC:
        case 0xFE:
            return 6;

        case 0x04:
        case 0x0C:
        case 0x1C:
            return -2;

        case 0x02:
        case 0x06:
        case 0x0E:
        case 0x1E:
        case 0x3E:
            return -4;

        case 0x01:
        case 0x03:
        case 0x07:
        case 0x0F:
        case 0x1F:
        case 0x3F:
        case 0x7D:
            return -6;

        default:
            // Keep the last steering direction for an unlisted transition.
            return previousError;
    }
}

float integralLimitFor(float ki)
{
    const float magnitude = (ki >= 0.0F) ? ki : -ki;
    if (magnitude == 0.0F) {
        return static_cast<float>(LineTracking::kCorrectionLimit);
    }
    return static_cast<float>(LineTracking::kCorrectionLimit) / magnitude;
}

Control::PidConfig makePidConfig(float kp, float ki, float kd)
{
    const float integralLimit = integralLimitFor(ki);
    return {
        kp,
        ki,
        kd,
        -static_cast<float>(LineTracking::kCorrectionLimit),
        static_cast<float>(LineTracking::kCorrectionLimit),
        -integralLimit,
        integralLimit,
        Control::DerivativeMode::Measurement,
    };
}

std::int16_t roundCorrection(float correction)
{
    return static_cast<std::int16_t>(
        correction >= 0.0F ? correction + 0.5F
                           : correction - 0.5F);
}

std::int16_t computeCorrection(
    std::int8_t error, std::uint32_t sampleSequence)
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
    const float output = g_steeringPid.update(
        0.0F, -static_cast<float>(error), dtSeconds);
    return roundCorrection(output);
}

void applyMotorCommands(std::int16_t correction)
{
    const std::int16_t left = static_cast<std::int16_t>(clampValue(
        static_cast<std::int32_t>(g_baseCommand) - correction,
        -TB6612::kMaxCommand, TB6612::kMaxCommand));
    const std::int16_t right = static_cast<std::int16_t>(clampValue(
        static_cast<std::int32_t>(g_baseCommand) + correction,
        -TB6612::kMaxCommand, TB6612::kMaxCommand));

    g_status.correction = correction;
    g_status.leftCommand = left;
    g_status.rightCommand = right;
    TB6612::setSpeeds(left, right);
}

}  // namespace

namespace LineTracking {

volatile bool enabled = true;

void init()
{
    g_baseCommand = kDefaultBaseCommand;
    g_steeringPid.configure(makePidConfig(
        kDefaultKp, kDefaultKi, kDefaultKd));
    g_candidateMask = 0;
    g_sameMaskCount = 0;
    g_lastSequence = 0;
    g_lastPidSequence = 0;
    g_status = {};
    enabled = true;
    TB6612::coast();
}

void setTunings(float kp, float ki, float kd)
{
    __disable_irq();
    g_steeringPid.configure(makePidConfig(kp, ki, kd));
    g_lastPidSequence = 0;
    __enable_irq();
}

void setBaseCommand(std::int16_t command)
{
    g_baseCommand = static_cast<std::int16_t>(
        clampValue(command, -TB6612::kMaxCommand, TB6612::kMaxCommand));
}

void stop()
{
    enabled = false;
    g_steeringPid.reset();
    g_lastPidSequence = 0;
    g_status.correction = 0;
    g_status.leftCommand = 0;
    g_status.rightCommand = 0;
    TB6612::coast();
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
    const std::uint8_t controlMask =
        makeControlMask(detectedMask, blackSensorCount);

    g_status.detectedLineMask = detectedMask;
    g_status.lineMask = controlMask;
    g_status.blackSensorCount = blackSensorCount;
    g_status.wideLineDetected =
        blackSensorCount >= kWideLineSensorCount;

    if (!enabled) {
        g_steeringPid.reset(-static_cast<float>(g_status.error));
        g_lastPidSequence = sampleSequence;
        TB6612::coast();
        return;
    }

    /*
     * Debounce the mask used for steering. Changes on discarded outer
     * sensors must not delay the drive-through response on a wide line.
     */
    if (!maskIsStable(controlMask)) {
        return;
    }

    g_status.error = errorForMask(controlMask, g_status.error);
    applyMotorCommands(
        computeCorrection(g_status.error, sampleSequence));
}

Status latest()
{
    return g_status;
}

}  // namespace LineTracking
