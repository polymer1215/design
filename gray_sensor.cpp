#include "gray_sensor.hpp"

#include "ti_msp_dl_config.h"

namespace GraySensor {

namespace {

constexpr std::uint32_t kSensorPins =
    GRAY_SENSOR_OUT1_PIN | GRAY_SENSOR_OUT2_PIN |
    GRAY_SENSOR_OUT3_PIN | GRAY_SENSOR_OUT4_PIN |
    GRAY_SENSOR_OUT5_PIN | GRAY_SENSOR_OUT6_PIN |
    GRAY_SENSOR_OUT7_PIN | GRAY_SENSOR_OUT8_PIN;

Sample g_sample = {};

}  // namespace

void init()
{
    g_sample = {};
    g_sample.sampledAt = 0xFFFFFFFFU;
}

void update(std::uint32_t encoderSequence)
{
    if (encoderSequence == g_sample.sampledAt) {
        return;
    }

    const std::uint32_t pins =
        DL_GPIO_readPins(GRAY_SENSOR_PORT, kSensorPins);
    g_sample.digital = static_cast<std::uint8_t>(
        ((pins & GRAY_SENSOR_OUT1_PIN) ? (1U << 0U) : 0U) |
        ((pins & GRAY_SENSOR_OUT2_PIN) ? (1U << 1U) : 0U) |
        ((pins & GRAY_SENSOR_OUT3_PIN) ? (1U << 2U) : 0U) |
        ((pins & GRAY_SENSOR_OUT4_PIN) ? (1U << 3U) : 0U) |
        ((pins & GRAY_SENSOR_OUT5_PIN) ? (1U << 4U) : 0U) |
        ((pins & GRAY_SENSOR_OUT6_PIN) ? (1U << 5U) : 0U) |
        ((pins & GRAY_SENSOR_OUT7_PIN) ? (1U << 6U) : 0U) |
        ((pins & GRAY_SENSOR_OUT8_PIN) ? (1U << 7U) : 0U));
    ++g_sample.sequence;
    g_sample.sampledAt = encoderSequence;
}

Sample latest()
{
    return g_sample;
}

}  // namespace GraySensor
