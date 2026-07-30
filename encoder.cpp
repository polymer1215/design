#include "encoder.hpp"

#include "speed_controller.hpp"
#include "ti_msp_dl_config.h"

namespace {

constexpr std::uint32_t kEncoderPins =
    ENCODER_E1A_LEFT_PIN | ENCODER_E1B_LEFT_PIN |
    ENCODER_E2A_RIGHT_PIN | ENCODER_E2B_RIGHT_PIN;

/*
 * Quadrature transition table indexed by (previousAB << 2) | currentAB.
 * Invalid transitions (including both bits changing together) count as zero.
 */
constexpr std::int8_t kTransition[16] = {
     0, -1,  1,  0,
     1,  0,  0, -1,
    -1,  0,  0,  1,
     0,  1, -1,  0
};

volatile std::int32_t g_rightCount = 0;
volatile std::int32_t g_leftCount = 0;
volatile std::int32_t g_rightDelta = 0;
volatile std::int32_t g_leftDelta = 0;
volatile std::uint32_t g_rightAEdges = 0;
volatile std::uint32_t g_rightBEdges = 0;
volatile std::uint32_t g_leftAEdges = 0;
volatile std::uint32_t g_leftBEdges = 0;
volatile std::uint32_t g_sampleSequence = 0;

std::int32_t g_previousRightSample = 0;
std::int32_t g_previousLeftSample = 0;
std::uint8_t g_previousRightState = 0;
std::uint8_t g_previousLeftState = 0;

std::uint8_t readRightState(std::uint32_t pins)
{
    return static_cast<std::uint8_t>(
        ((pins & ENCODER_E2A_RIGHT_PIN) ? 2U : 0U) |
        ((pins & ENCODER_E2B_RIGHT_PIN) ? 1U : 0U));
}

std::uint8_t readLeftState(std::uint32_t pins)
{
    return static_cast<std::uint8_t>(
        ((pins & ENCODER_E1A_LEFT_PIN) ? 2U : 0U) |
        ((pins & ENCODER_E1B_LEFT_PIN) ? 1U : 0U));
}

void updateRight(std::uint32_t pins)
{
    const std::uint8_t state = readRightState(pins);
    const std::uint8_t index =
        static_cast<std::uint8_t>((g_previousRightState << 2U) | state);
    g_rightCount += Encoder::kRightEncoderPolarity * kTransition[index];
    g_previousRightState = state;
}

void updateLeft(std::uint32_t pins)
{
    const std::uint8_t state = readLeftState(pins);
    const std::uint8_t index =
        static_cast<std::uint8_t>((g_previousLeftState << 2U) | state);
    g_leftCount += Encoder::kLeftEncoderPolarity * kTransition[index];
    g_previousLeftState = state;
}

void handleEncoderInterrupt()
{
    const std::uint32_t status =
        DL_GPIO_getEnabledInterruptStatus(ENCODER_PORT, kEncoderPins);
    const std::uint32_t pins = DL_GPIO_readPins(ENCODER_PORT, kEncoderPins);

    if (status & ENCODER_E1A_LEFT_PIN) {
        ++g_leftAEdges;
    }
    if (status & ENCODER_E1B_LEFT_PIN) {
        ++g_leftBEdges;
    }
    if (status & ENCODER_E2A_RIGHT_PIN) {
        ++g_rightAEdges;
    }
    if (status & ENCODER_E2B_RIGHT_PIN) {
        ++g_rightBEdges;
    }

    if (status &
        (ENCODER_E1A_LEFT_PIN | ENCODER_E1B_LEFT_PIN)) {
        updateLeft(pins);
    }
    if (status &
        (ENCODER_E2A_RIGHT_PIN | ENCODER_E2B_RIGHT_PIN)) {
        updateRight(pins);
    }

    DL_GPIO_clearInterruptStatus(ENCODER_PORT, status);
}

}  // namespace

namespace Encoder {

bool init()
{
    const std::uint32_t pins = DL_GPIO_readPins(ENCODER_PORT, kEncoderPins);
    g_previousRightState = readRightState(pins);
    g_previousLeftState = readLeftState(pins);

    DL_GPIO_clearInterruptStatus(ENCODER_PORT, kEncoderPins);
    NVIC_ClearPendingIRQ(ENCODER_INT_IRQN);
    NVIC_EnableIRQ(ENCODER_INT_IRQN);

    return SysTick_Config(CPUCLK_FREQ / kSampleRateHz) == 0U;
}

Sample latest()
{
    Sample result;

    __disable_irq();
    result.rightTotal = g_rightCount;
    result.leftTotal = g_leftCount;
    result.rightDelta = g_rightDelta;
    result.leftDelta = g_leftDelta;
    result.rightAEdges = g_rightAEdges;
    result.rightBEdges = g_rightBEdges;
    result.leftAEdges = g_leftAEdges;
    result.leftBEdges = g_leftBEdges;
    result.sequence = g_sampleSequence;
    __enable_irq();

    return result;
}

}  // namespace Encoder

extern "C" void GROUP1_IRQHandler(void)
{
    if (DL_Interrupt_getPendingGroup(DL_INTERRUPT_GROUP_1) ==
        ENCODER_INT_IIDX) {
        handleEncoderInterrupt();
    }
}

extern "C" void SysTick_Handler(void)
{
    const std::int32_t right = g_rightCount;
    const std::int32_t left = g_leftCount;

    g_rightDelta = right - g_previousRightSample;
    g_leftDelta = left - g_previousLeftSample;
    g_previousRightSample = right;
    g_previousLeftSample = left;
    ++g_sampleSequence;

    SpeedControl::updateFromEncoder(g_leftDelta, g_rightDelta);
}
