#include "pid_dashboard.hpp"

#include "encoder.hpp"
#include "oled.h"
#include "speed_controller.hpp"

namespace {

constexpr std::uint32_t kRefreshPeriodSamples =
    Encoder::kSampleRateHz / 5U;
constexpr std::uint32_t kK230PageHoldSamples =
    2U * Encoder::kSampleRateHz;
constexpr std::uint8_t kK230DisplayBytes = 19U;

std::uint32_t g_displayedSequence = 0U;
std::uint32_t g_lastK230Sequence = 0U;
std::uint8_t g_k230Bytes[kK230DisplayBytes] = {};
std::uint8_t g_k230ByteCount = 0U;
bool g_k230DataPending = false;

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

char hexDigit(std::uint8_t value)
{
    value &= 0x0FU;
    return value < 10U
        ? static_cast<char>('0' + value)
        : static_cast<char>('A' + (value - 10U));
}

void formatHexLine(
    char *line, std::uint8_t firstByte, std::uint8_t byteCount,
    bool showHeader)
{
    std::uint8_t position = 0U;
    if (showHeader) {
        line[position++] = 'R';
        line[position++] = 'X';
        line[position++] = ':';
    }

    for (std::uint8_t i = 0U; i < byteCount; ++i) {
        const std::uint8_t index =
            static_cast<std::uint8_t>(firstByte + i);
        if (i > 0U) {
            line[position++] = ' ';
        }

        if (index < g_k230ByteCount) {
            const std::uint8_t data = g_k230Bytes[index];
            line[position++] = hexDigit(data >> 4U);
            line[position++] = hexDigit(data);
        } else {
            line[position++] = '-';
            line[position++] = '-';
        }
    }
    line[position] = '\0';
}

void showK230Data()
{
    char line0[16];
    char line1[16];
    char line2[16];
    char line3[16];

    formatHexLine(line0, 0U, 4U, true);
    formatHexLine(line1, 4U, 5U, false);
    formatHexLine(line2, 9U, 5U, false);
    formatHexLine(line3, 14U, 5U, false);

    OLED_Clear();
    OLED_ShowString(
        0, 0, reinterpret_cast<const u8 *>(line0), 16, 1);
    OLED_ShowString(
        0, 16, reinterpret_cast<const u8 *>(line1), 16, 1);
    OLED_ShowString(
        0, 32, reinterpret_cast<const u8 *>(line2), 16, 1);
    OLED_ShowString(
        0, 48, reinterpret_cast<const u8 *>(line3), 16, 1);
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
    g_lastK230Sequence = 0U;
    g_k230ByteCount = 0U;
    g_k230DataPending = false;
}

void pushK230Byte(std::uint8_t data)
{
    if (g_k230ByteCount < kK230DisplayBytes) {
        g_k230Bytes[g_k230ByteCount++] = data;
    } else {
        for (std::uint8_t i = 1U; i < kK230DisplayBytes; ++i) {
            g_k230Bytes[i - 1U] = g_k230Bytes[i];
        }
        g_k230Bytes[kK230DisplayBytes - 1U] = data;
    }
    g_k230DataPending = true;
}

void update(std::uint32_t sampleSequence)
{
    if (!OLED_IsConnected() ||
        (sampleSequence - g_displayedSequence) <
            kRefreshPeriodSamples) {
        return;
    }

    g_displayedSequence = sampleSequence;
    if (g_k230DataPending) {
        g_lastK230Sequence = sampleSequence;
        g_k230DataPending = false;
    }

    if (g_k230ByteCount > 0U &&
        (sampleSequence - g_lastK230Sequence) <
            kK230PageHoldSamples) {
        showK230Data();
    } else {
        show(SpeedControl::latest());
    }
}

}  // namespace PidDashboard
