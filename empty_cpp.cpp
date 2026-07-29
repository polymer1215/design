/*
 * Copyright (c) 2023, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "ti_msp_dl_config.h"
#include "encoder.hpp"
#include "k230_uart.hpp"
#include "oled.h"
#include "speed_controller.hpp"
#include "tb6612.hpp"

namespace {

constexpr float kSpeedTestStartRpm = 50.0F;
constexpr float kSpeedTestStepRpm = 50.0F;
constexpr float kSpeedTestMaximumRpm = 500.0F;
constexpr std::uint32_t kSpeedTestStepTicks =
    2U * Encoder::kSampleRateHz;

void formatSigned(char *output, std::int32_t value, std::uint8_t digits)
{
    std::uint32_t magnitude;

    *output++ = value < 0 ? '-' : '+';
    magnitude = value < 0
        ? static_cast<std::uint32_t>(-(value + 1)) + 1U
        : static_cast<std::uint32_t>(value);

    std::uint32_t divisor = 1U;
    for (std::uint8_t i = 1U; i < digits; ++i) {
        divisor *= 10U;
    }

    while (digits-- > 0U) {
        *output++ = static_cast<char>('0' + ((magnitude / divisor) % 10U));
        divisor /= 10U;
    }
}

void formatPidSpeedLine(
    char *line, char wheel, float targetRpm, float measuredRpm)
{
    const std::int32_t target = static_cast<std::int32_t>(targetRpm);
    const std::int32_t measured = static_cast<std::int32_t>(measuredRpm);
    line[0] = wheel;
    line[1] = ' ';
    line[2] = 'T';
    line[3] = ':';
    formatSigned(&line[4], target, 3U);
    line[8] = ' ';
    line[9] = 'M';
    line[10] = ':';
    formatSigned(&line[11], measured, 3U);
    line[15] = '\0';
}

void formatPidOutputLine(char *line, char wheel, std::int16_t output)
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

void showPidData(const SpeedControl::Status &status)
{
    char rightSpeed[16];
    char leftSpeed[16];
    char rightOutput[12];
    char leftOutput[12];

    formatPidSpeedLine(rightSpeed, 'R',
        status.rightTargetRpm, status.rightMeasuredRpm);
    formatPidOutputLine(rightOutput, 'R', status.rightOutput);
    formatPidSpeedLine(leftSpeed, 'L',
        status.leftTargetRpm, status.leftMeasuredRpm);
    formatPidOutputLine(leftOutput, 'L', status.leftOutput);

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

int main(void)
{
    SYSCFG_DL_init();
    TB6612::init();
    OLED_Init();
    OLED_ColorTurn(0);
    OLED_DisplayTurn(0);
    K230Uart::init();

    SpeedControl::init();
    SpeedControl::setTunings(
        SpeedControl::kDefaultKp,
        SpeedControl::kDefaultKi,
        SpeedControl::kDefaultKd);
    SpeedControl::setTargetRpm(
        kSpeedTestStartRpm,
        kSpeedTestStartRpm);
    Encoder::init();

    float speedTestTargetRpm = kSpeedTestStartRpm;
    std::uint32_t nextSpeedStep = kSpeedTestStepTicks;
    std::uint32_t displayedSequence = 0U;
    while (1) {
        K230Uart::serviceEcho();
        const Encoder::Sample sample = Encoder::latest();

        while (speedTestTargetRpm < kSpeedTestMaximumRpm &&
            sample.sequence >= nextSpeedStep) {
            speedTestTargetRpm += kSpeedTestStepRpm;
            SpeedControl::setTargetRpm(
                speedTestTargetRpm, speedTestTargetRpm);
            nextSpeedStep += kSpeedTestStepTicks;
        }

        if (OLED_IsConnected() &&
            (sample.sequence - displayedSequence) >= 20U) {
            displayedSequence = sample.sequence;
            showPidData(SpeedControl::latest());
        }
        __WFI();
    }
}
