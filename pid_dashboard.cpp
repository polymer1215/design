#include "pid_dashboard.hpp"

#include "encoder.hpp"
#include "oled.h"
#include "speed_controller.hpp"

namespace {

constexpr std::uint32_t kRefreshPeriodSamples =
    Encoder::kSampleRateHz / 5U;

std::uint32_t g_displayedSequence = 0U;

void formatSigned(char *output, std::int32_t value, std::uint8_t digits)
{
    *output++ = value < 0 ? '-' : '+';
    const std::uint32_t magnitude = value < 0
        ? static_cast<std::uint32_t>(-(value + 1)) + 1U
        : static_cast<std::uint32_t>(value);

    std::uint32_t divisor = 1U;
    for (std::uint8_t i = 1U; i < digits; ++i) {
        divisor *= 10U;
    }

    while (digits-- > 0U) {
        *output++ =
            static_cast<char>('0' + ((magnitude / divisor) % 10U));
        divisor /= 10U;
    }
}

void formatSpeedLine(
    char *line, char wheel, float targetRpm, float measuredRpm)
{
    line[0] = wheel;
    line[1] = ' ';
    line[2] = 'T';
    line[3] = ':';
    formatSigned(
        &line[4], static_cast<std::int32_t>(targetRpm), 3U);
    line[8] = ' ';
    line[9] = 'M';
    line[10] = ':';
    formatSigned(
        &line[11], static_cast<std::int32_t>(measuredRpm), 3U);
    line[15] = '\0';
}

void formatOutputLine(char *line, char wheel, std::int16_t output)
{
    line[0] = wheel;
    line[1] = ' ';
    line[2] = 'P';
    line[3] = 'W';
    line[4] = 'M';
    line[5] = ':';
    formatSigned(&line[6], output, 4U);
    line[11] = '\0';
}

void showSpeedControl(const SpeedControl::Status &status)
{
    char rightSpeed[16];
    char leftSpeed[16];
    char rightOutput[12];
    char leftOutput[12];

    formatSpeedLine(rightSpeed, 'R',
        status.rightTargetRpm, status.rightMeasuredRpm);
    formatOutputLine(rightOutput, 'R', status.rightOutput);
    formatSpeedLine(leftSpeed, 'L',
        status.leftTargetRpm, status.leftMeasuredRpm);
    formatOutputLine(leftOutput, 'L', status.leftOutput);

    OLED_Clear();
    OLED_ShowString(
        0, 0, reinterpret_cast<const u8 *>(rightSpeed), 16, 1);
    OLED_ShowString(
        0, 16, reinterpret_cast<const u8 *>(rightOutput), 16, 1);
    OLED_ShowString(
        0, 32, reinterpret_cast<const u8 *>(leftSpeed), 16, 1);
    OLED_ShowString(
        0, 48, reinterpret_cast<const u8 *>(leftOutput), 16, 1);
    OLED_Refresh();
}

}  // namespace

namespace PidDashboard {

void init()
{
    OLED_Init();
    OLED_ColorTurn(0);
    OLED_DisplayTurn(0);
    g_displayedSequence = 0U;
}

void update(std::uint32_t sampleSequence)
{
    if (!OLED_IsConnected() ||
        (sampleSequence - g_displayedSequence) <
            kRefreshPeriodSamples) {
        return;
    }

    g_displayedSequence = sampleSequence;
    showSpeedControl(SpeedControl::latest());
}

}  // namespace PidDashboard
