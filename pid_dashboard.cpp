#include "pid_dashboard.hpp"

#include "encoder.hpp"
#include "gray_sensor.hpp"
#include "k230_uart.hpp"
#include "oled.h"
#include "speed_controller.hpp"

namespace {

constexpr std::uint32_t kRefreshPeriodSamples =
    Encoder::kSampleRateHz / 5U;

std::uint32_t g_displayedSequence = 0U;
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

void show(const SpeedControl::Status &status)
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
    char *line, char axis, bool detected, std::int16_t coordinate)
{
    line[0] = axis;
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

void showBallPosition(const K230Protocol::BallPosition &position)
{
    char xLine[6];
    char yLine[6];
    char sequenceLine[10] = "RX:";

    formatCoordinateLine(
        xLine, 'X', position.detected, position.x);
    formatCoordinateLine(
        yLine, 'Y', position.detected, position.y);
    formatUnsigned(&sequenceLine[3], position.sequence, 6U);
    sequenceLine[9] = '\0';

    const u8 *title = reinterpret_cast<const u8 *>(
        position.detected ? "K230 BALL" : "K230 NO BALL");

    OLED_Clear();
    OLED_ShowString(
        0, 0, title, 16, 1);
    OLED_ShowString(
        0, 16, reinterpret_cast<const u8 *>(xLine), 16, 1);
    OLED_ShowString(
        0, 32, reinterpret_cast<const u8 *>(yLine), 16, 1);
    OLED_ShowString(
        0, 48, reinterpret_cast<const u8 *>(sequenceLine), 16, 1);
    OLED_Refresh();
}

void showK230Status(const K230Uart::Statistics &statistics)
{
    char receivedLine[10] = "RX:";
    char droppedLine[10] = "DROP:";

    formatUnsigned(&receivedLine[3], statistics.receivedBytes, 6U);
    receivedLine[9] = '\0';
    formatUnsigned(&droppedLine[5], statistics.droppedBytes, 4U);
    droppedLine[9] = '\0';

    const u8 *title = reinterpret_cast<const u8 *>(
        statistics.receivedBytes == 0U ? "K230 WAIT" : "K230 RAW");

    OLED_Clear();
    OLED_ShowString(0, 0, title, 16, 1);
    OLED_ShowString(
        0, 16, reinterpret_cast<const u8 *>(receivedLine), 16, 1);
    OLED_ShowString(
        0, 32, reinterpret_cast<const u8 *>(droppedLine), 16, 1);
    OLED_ShowString(
        0, 48, reinterpret_cast<const u8 *>("NEED BALL,x,y"), 16, 1);
    OLED_Refresh();
}

char hexDigit(std::uint8_t value)
{
    return static_cast<char>(
        value < 10U ? '0' + value : 'A' + value - 10U);
}

void formatAnalogPair(char *line, std::uint8_t firstChannel,
    std::uint8_t first, std::uint8_t second)
{
    line[0] = static_cast<char>('1' + firstChannel);
    line[1] = ':';
    formatUnsigned(&line[2], first, 3U);
    line[5] = ' ';
    line[6] = static_cast<char>('2' + firstChannel);
    line[7] = ':';
    formatUnsigned(&line[8], second, 3U);
    line[11] = '\0';
}

void showGraySensor(const GraySensor::Sample &sample)
{
    char title[15] = "GRAY WAIT A:4C";
    char digitalLine[15] = "D:00 N:000000";
    char errorLine[11] = "ERR:000000";
    char pairs[4][12] = {};

    if (sample.connected) {
        title[5] = 'O';
        title[6] = 'K';
        title[7] = ' ';
        title[8] = ' ';
    }
    digitalLine[2] = hexDigit((sample.digital >> 4U) & 0x0FU);
    digitalLine[3] = hexDigit(sample.digital & 0x0FU);
    formatUnsigned(&digitalLine[7], sample.sequence, 6U);
    formatUnsigned(&errorLine[4], sample.errors, 6U);

    for (std::uint8_t pair = 0U; pair < 4U; ++pair) {
        const std::uint8_t channel = pair * 2U;
        formatAnalogPair(pairs[pair], channel,
            sample.analog[channel], sample.analog[channel + 1U]);
    }

    OLED_Clear();
    OLED_ShowString(
        0, 0, reinterpret_cast<const u8 *>(title), 8, 1);
    OLED_ShowString(
        0, 8, reinterpret_cast<const u8 *>(digitalLine), 8, 1);
    for (std::uint8_t pair = 0U; pair < 4U; ++pair) {
        OLED_ShowString(0, static_cast<u8>(16U + pair * 8U),
            reinterpret_cast<const u8 *>(pairs[pair]), 8, 1);
    }
    OLED_ShowString(
        0, 48, reinterpret_cast<const u8 *>(errorLine), 8, 1);
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
    g_ballPosition = {};
    g_hasBallFrame = false;
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
    const std::uint32_t seconds =
        sampleSequence / Encoder::kSampleRateHz;
    if ((seconds & 1U) == 0U) {
        showGraySensor(GraySensor::latest());
    } else if (g_hasBallFrame) {
        showBallPosition(g_ballPosition);
    } else if (((seconds / 2U) & 1U) == 0U) {
        showK230Status(K230Uart::statistics());
    } else {
        show(SpeedControl::latest());
    }
}

}  // namespace PidDashboard
