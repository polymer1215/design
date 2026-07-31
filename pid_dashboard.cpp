#include "pid_dashboard.hpp"

#include "encoder.hpp"
#include "k230_uart.hpp"
#include "oled.h"
#include "speed_controller.hpp"
#include "stepper.hpp"

namespace {

constexpr std::uint32_t kRefreshPeriodSamples =
    Encoder::kSampleRateHz / 5U;

std::uint32_t g_displayedSequence = 0U;
PidDashboard::View g_view = PidDashboard::View::K230Monitor;
K230Protocol::BallPosition g_ballPosition = {};
bool g_hasBallFrame = false;

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

void formatUnsigned(
    char *output, std::uint32_t value, std::uint8_t digits)
{
    std::uint32_t divisor = 1U;
    for (std::uint8_t i = 1U; i < digits; ++i) {
        divisor *= 10U;
    }

    while (digits-- > 0U) {
        *output++ =
            static_cast<char>('0' + ((value / divisor) % 10U));
        divisor /= 10U;
    }
}

void formatCoordinateLine(
    char *line, bool detected, std::int16_t coordinate)
{
    line[0] = 'X';
    line[1] = ':';
    if (detected) {
        formatUnsigned(
            &line[2], static_cast<std::uint32_t>(coordinate), 3U);
    } else {
        line[2] = '-';
        line[3] = '-';
        line[4] = '-';
    }
    line[5] = '\0';
}

void showK230Monitor()
{
    const K230Uart::Statistics statistics = K230Uart::statistics();
    const Stepper::Status stepperStatus = Stepper::status();

    const char *title;
    if (g_hasBallFrame) {
        title = g_ballPosition.detected ? "K230 BALL" : "K230 NO BALL";
    } else {
        title = statistics.receivedBytes == 0U
            ? "K230 WAIT"
            : "K230 RAW";
    }

    char coordinateLine[6];
    char receivedLine[10] = "RX:";
    char statusLine[16] = "D:0000 STEP:OFF";

    formatCoordinateLine(
        coordinateLine,
        g_hasBallFrame && g_ballPosition.detected,
        g_ballPosition.x);
    formatUnsigned(&receivedLine[3], statistics.receivedBytes, 6U);
    receivedLine[9] = '\0';
    formatUnsigned(&statusLine[2], statistics.droppedBytes, 4U);
    if (stepperStatus.enabled) {
        statusLine[12] = 'O';
        statusLine[13] = 'N';
        statusLine[14] = '\0';
    }

    OLED_Clear();
    OLED_ShowString(
        0, 0, reinterpret_cast<const u8 *>(title), 16, 1);
    OLED_ShowString(
        0, 16, reinterpret_cast<const u8 *>(coordinateLine), 16, 1);
    OLED_ShowString(
        0, 32, reinterpret_cast<const u8 *>(receivedLine), 16, 1);
    OLED_ShowString(
        0, 48, reinterpret_cast<const u8 *>(statusLine), 16, 1);
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
    g_view = View::K230Monitor;
    g_ballPosition = {};
    g_hasBallFrame = false;
}

void setView(View view)
{
    g_view = view;
    g_displayedSequence = 0U;
}

View currentView()
{
    return g_view;
}

void setBallPosition(const K230Protocol::BallPosition &position)
{
    g_ballPosition = position;
    g_hasBallFrame = true;
}

void update(std::uint32_t sampleSequence)
{
    if (!OLED_IsConnected() ||
        (sampleSequence - g_displayedSequence) <
            kRefreshPeriodSamples) {
        return;
    }

    g_displayedSequence = sampleSequence;
    switch (g_view) {
        case View::K230Monitor:
            showK230Monitor();
            break;
        case View::SpeedControl:
            showSpeedControl(SpeedControl::latest());
            break;
    }
}

}  // namespace PidDashboard
