#include "stepper.hpp"

#include <cstdint>

#include "ti_msp_dl_config.h"

namespace {

constexpr std::uint32_t kStepperPins =
    STEPPER_ST1_PIN | STEPPER_DIR1_PIN | STEPPER_EN1_PIN;
constexpr std::uint32_t kTimerClockHz = CPUCLK_FREQ;
constexpr std::uint32_t kEnableDelayCycles = CPUCLK_FREQ / 1000U;

volatile bool g_enabled = false;
volatile bool g_running = false;
volatile bool g_continuous = false;
volatile bool g_directionPositive = true;
volatile bool g_stepHigh = false;
volatile std::uint32_t g_frequencyHz = 0U;
volatile std::uint32_t g_pulsesRemaining = 0U;
volatile std::int32_t g_positionPulses = 0;

std::uint32_t lockInterrupts()
{
    const std::uint32_t primask = __get_PRIMASK();
    __disable_irq();
    return primask;
}

void unlockInterrupts(std::uint32_t primask)
{
    __set_PRIMASK(primask);
}

void stopLocked()
{
    DL_TimerG_stopCounter(STEPPER_TIMER_INST);
    DL_TimerG_clearInterruptStatus(
        STEPPER_TIMER_INST, DL_TIMERG_INTERRUPT_ZERO_EVENT);
    DL_GPIO_clearPins(STEPPER_PORT, STEPPER_ST1_PIN);

    g_running = false;
    g_continuous = false;
    g_stepHigh = false;
    g_frequencyHz = 0U;
    g_pulsesRemaining = 0U;
}

bool frequencyIsValid(std::uint32_t frequencyHz)
{
    return frequencyHz >= Stepper::kMinimumStepFrequencyHz &&
        frequencyHz <= Stepper::kMaximumStepFrequencyHz;
}

bool positionIsWithinLimits(std::int64_t position)
{
    return position >= -Stepper::kTravelLimitPulses &&
        position <= Stepper::kTravelLimitPulses;
}

std::uint32_t halfPeriodLoad(std::uint32_t frequencyHz)
{
    const std::uint32_t halfPeriodTicks =
        kTimerClockHz / (2U * frequencyHz);
    return halfPeriodTicks - 1U;
}

void startLocked(
    bool directionPositive,
    std::uint32_t frequencyHz,
    bool continuous,
    std::uint32_t pulses)
{
    stopLocked();

    if (directionPositive) {
        DL_GPIO_setPins(STEPPER_PORT, STEPPER_DIR1_PIN);
    } else {
        DL_GPIO_clearPins(STEPPER_PORT, STEPPER_DIR1_PIN);
    }

    const std::uint32_t load = halfPeriodLoad(frequencyHz);
    DL_TimerG_setLoadValue(STEPPER_TIMER_INST, load);
    DL_TimerG_setTimerCount(STEPPER_TIMER_INST, load);

    g_directionPositive = directionPositive;
    g_frequencyHz = frequencyHz;
    g_continuous = continuous;
    g_pulsesRemaining = pulses;
    g_stepHigh = false;
    g_running = true;

    DL_TimerG_clearInterruptStatus(
        STEPPER_TIMER_INST, DL_TIMERG_INTERRUPT_ZERO_EVENT);
    DL_TimerG_startCounter(STEPPER_TIMER_INST);
}

std::uint32_t magnitude(std::int32_t value)
{
    return value < 0
        ? static_cast<std::uint32_t>(-static_cast<std::int64_t>(value))
        : static_cast<std::uint32_t>(value);
}

}  // namespace

namespace Stepper {

void init()
{
    const std::uint32_t primask = lockInterrupts();

    DL_TimerG_stopCounter(STEPPER_TIMER_INST);
    DL_TimerG_clearInterruptStatus(
        STEPPER_TIMER_INST, DL_TIMERG_INTERRUPT_ZERO_EVENT);
    DL_GPIO_clearPins(STEPPER_PORT, kStepperPins);

    g_enabled = false;
    g_running = false;
    g_continuous = false;
    g_directionPositive = true;
    g_stepHigh = false;
    g_frequencyHz = 0U;
    g_pulsesRemaining = 0U;
    g_positionPulses = 0;

    NVIC_ClearPendingIRQ(STEPPER_TIMER_INST_INT_IRQN);
    NVIC_EnableIRQ(STEPPER_TIMER_INST_INT_IRQN);
    unlockInterrupts(primask);
}

void setEnabled(bool enabled)
{
    if (!enabled) {
        const std::uint32_t primask = lockInterrupts();
        stopLocked();
        DL_GPIO_clearPins(STEPPER_PORT, STEPPER_EN1_PIN);
        g_enabled = false;
        unlockInterrupts(primask);
        return;
    }

    if (isEnabled()) {
        return;
    }

    DL_GPIO_setPins(STEPPER_PORT, STEPPER_EN1_PIN);
    delay_cycles(kEnableDelayCycles);

    const std::uint32_t primask = lockInterrupts();
    g_enabled = true;
    unlockInterrupts(primask);
}

bool isEnabled()
{
    const std::uint32_t primask = lockInterrupts();
    const bool enabled = g_enabled;
    unlockInterrupts(primask);
    return enabled;
}

bool moveRelative(std::int32_t pulses, std::uint32_t frequencyHz)
{
    if (pulses == 0) {
        stop();
        return true;
    }
    if (!frequencyIsValid(frequencyHz)) {
        stop();
        return false;
    }

    const std::uint32_t primask = lockInterrupts();
    if (!g_enabled) {
        unlockInterrupts(primask);
        return false;
    }

    const std::int64_t target =
        static_cast<std::int64_t>(g_positionPulses) + pulses;
    if (!positionIsWithinLimits(target)) {
        stopLocked();
        unlockInterrupts(primask);
        return false;
    }

    startLocked(pulses > 0, frequencyHz, false, magnitude(pulses));
    unlockInterrupts(primask);
    return true;
}

bool moveTo(std::int32_t targetPulses, std::uint32_t frequencyHz)
{
    if (!positionIsWithinLimits(targetPulses) ||
        !frequencyIsValid(frequencyHz)) {
        stop();
        return false;
    }

    const std::uint32_t primask = lockInterrupts();
    if (!g_enabled) {
        unlockInterrupts(primask);
        return false;
    }

    const std::int32_t delta = targetPulses - g_positionPulses;
    if (delta == 0) {
        stopLocked();
        unlockInterrupts(primask);
        return true;
    }

    startLocked(delta > 0, frequencyHz, false, magnitude(delta));
    unlockInterrupts(primask);
    return true;
}

bool moveToDegrees(float targetDegrees, std::uint32_t frequencyHz)
{
    return moveTo(degreesToPulses(targetDegrees), frequencyHz);
}

bool setVelocity(std::int32_t signedFrequencyHz)
{
    if (signedFrequencyHz == 0) {
        stop();
        return true;
    }

    const std::uint32_t frequencyHz = magnitude(signedFrequencyHz);
    if (!frequencyIsValid(frequencyHz)) {
        stop();
        return false;
    }

    const bool directionPositive = signedFrequencyHz > 0;
    const std::uint32_t primask = lockInterrupts();
    if (!g_enabled) {
        unlockInterrupts(primask);
        return false;
    }

    if (g_running && g_continuous &&
        g_directionPositive == directionPositive &&
        g_frequencyHz == frequencyHz) {
        unlockInterrupts(primask);
        return true;
    }

    startLocked(directionPositive, frequencyHz, true, 0U);
    unlockInterrupts(primask);
    return true;
}

void stop()
{
    const std::uint32_t primask = lockInterrupts();
    stopLocked();
    unlockInterrupts(primask);
}

bool isBusy()
{
    const std::uint32_t primask = lockInterrupts();
    const bool running = g_running;
    unlockInterrupts(primask);
    return running;
}

Status status()
{
    const std::uint32_t primask = lockInterrupts();
    const Status result = {
        g_enabled,
        g_running,
        g_continuous,
        g_directionPositive,
        g_frequencyHz,
        g_pulsesRemaining,
        g_positionPulses
    };
    unlockInterrupts(primask);
    return result;
}

std::int32_t positionPulses()
{
    const std::uint32_t primask = lockInterrupts();
    const std::int32_t position = g_positionPulses;
    unlockInterrupts(primask);
    return position;
}

void setPositionPulses(std::int32_t position)
{
    const std::uint32_t primask = lockInterrupts();
    stopLocked();
    if (position > kTravelLimitPulses) {
        g_positionPulses = kTravelLimitPulses;
    } else if (position < -kTravelLimitPulses) {
        g_positionPulses = -kTravelLimitPulses;
    } else {
        g_positionPulses = position;
    }
    unlockInterrupts(primask);
}

std::int32_t degreesToPulses(float degrees)
{
    const float pulses =
        degrees * static_cast<float>(kPulsesPerRevolution) / 360.0F;
    return static_cast<std::int32_t>(
        pulses >= 0.0F ? pulses + 0.5F : pulses - 0.5F);
}

float pulsesToDegrees(std::int32_t pulses)
{
    return static_cast<float>(pulses) * 360.0F /
        static_cast<float>(kPulsesPerRevolution);
}

}  // namespace Stepper

extern "C" void STEPPER_TIMER_INST_IRQHandler(void)
{
    switch (DL_TimerG_getPendingInterrupt(STEPPER_TIMER_INST)) {
        case DL_TIMER_IIDX_ZERO:
            if (!g_running || !g_enabled) {
                stopLocked();
                break;
            }

            if (!g_stepHigh) {
                const std::int32_t nextPosition =
                    g_positionPulses + (g_directionPositive ? 1 : -1);
                if (!positionIsWithinLimits(nextPosition)) {
                    stopLocked();
                    break;
                }

                DL_GPIO_setPins(STEPPER_PORT, STEPPER_ST1_PIN);
                g_stepHigh = true;

                g_positionPulses = nextPosition;
                if (!g_continuous) {
                    --g_pulsesRemaining;
                }
            } else {
                DL_GPIO_clearPins(STEPPER_PORT, STEPPER_ST1_PIN);
                g_stepHigh = false;

                if (!g_continuous && g_pulsesRemaining == 0U) {
                    stopLocked();
                }
            }
            break;
        default:
            break;
    }
}
