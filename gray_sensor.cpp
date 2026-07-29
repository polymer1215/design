#include "gray_sensor.hpp"

#include "encoder.hpp"
#include "ti_msp_dl_config.h"

namespace GraySensor {

namespace {

constexpr std::uint8_t kPingCommand = 0xAAU;
constexpr std::uint8_t kPingResponse = 0x66U;
constexpr std::uint8_t kDigitalCommand = 0xDDU;
constexpr std::uint8_t kAllAnalogCommand = 0xB0U;

constexpr std::uint32_t kPollPeriodSamples =
    Encoder::kSampleRateHz / 20U;
constexpr std::uint32_t kRetryPeriodSamples =
    Encoder::kSampleRateHz / 10U;
constexpr std::uint32_t kTransferTimeoutCycles = 800000U;
constexpr std::uint32_t kCommandResponseDelayCycles = 160000U;

Sample g_sample = {};
std::uint32_t g_lastAttemptSequence = 0U;

void recoverBus()
{
    DL_I2C_resetControllerTransfer(I2C_OLED_INST);
    DL_I2C_flushControllerTXFIFO(I2C_OLED_INST);
    DL_I2C_flushControllerRXFIFO(I2C_OLED_INST);
}

bool waitForIdle()
{
    std::uint32_t timeout = kTransferTimeoutCycles;
    while ((DL_I2C_getControllerStatus(I2C_OLED_INST) &
               DL_I2C_CONTROLLER_STATUS_IDLE) == 0U) {
        if (--timeout == 0U) {
            recoverBus();
            return false;
        }
    }
    return true;
}

bool waitForTransferComplete()
{
    std::uint32_t timeout = kTransferTimeoutCycles;
    while ((DL_I2C_getControllerStatus(I2C_OLED_INST) &
               DL_I2C_CONTROLLER_STATUS_BUSY_BUS) != 0U) {
        if ((DL_I2C_getControllerStatus(I2C_OLED_INST) &
                DL_I2C_CONTROLLER_STATUS_ERROR) != 0U ||
            --timeout == 0U) {
            recoverBus();
            return false;
        }
    }

    if ((DL_I2C_getControllerStatus(I2C_OLED_INST) &
            DL_I2C_CONTROLLER_STATUS_ERROR) != 0U) {
        recoverBus();
        return false;
    }
    return true;
}

bool writeCommand(std::uint8_t command)
{
    if (!waitForIdle()) {
        return false;
    }

    DL_I2C_flushControllerTXFIFO(I2C_OLED_INST);
    DL_I2C_fillControllerTXFIFO(I2C_OLED_INST, &command, 1U);
    DL_I2C_startControllerTransfer(I2C_OLED_INST, kDefaultAddress,
        DL_I2C_CONTROLLER_DIRECTION_TX, 1U);
    return waitForTransferComplete();
}

bool readBytes(std::uint8_t *data, std::uint8_t length)
{
    if (data == nullptr || length == 0U || !waitForIdle()) {
        return false;
    }

    DL_I2C_flushControllerRXFIFO(I2C_OLED_INST);
    DL_I2C_startControllerTransfer(I2C_OLED_INST, kDefaultAddress,
        DL_I2C_CONTROLLER_DIRECTION_RX, length);

    std::uint8_t count = 0U;
    std::uint32_t timeout = kTransferTimeoutCycles;
    while (count < length) {
        while (!DL_I2C_isControllerRXFIFOEmpty(I2C_OLED_INST) &&
               count < length) {
            data[count++] =
                DL_I2C_receiveControllerData(I2C_OLED_INST);
        }

        if ((DL_I2C_getControllerStatus(I2C_OLED_INST) &
                DL_I2C_CONTROLLER_STATUS_ERROR) != 0U ||
            --timeout == 0U) {
            recoverBus();
            return false;
        }
    }

    return waitForTransferComplete();
}

bool commandRead(
    std::uint8_t command, std::uint8_t *data, std::uint8_t length)
{
    if (!writeCommand(command)) {
        return false;
    }

    /*
     * The sensor retains the latest command. A short processing interval
     * makes the write-with-STOP/read-with-START form reliable while keeping
     * this shared OLED bus simple and recoverable.
     */
    delay_cycles(kCommandResponseDelayCycles);
    return readBytes(data, length);
}

}  // namespace

void init()
{
    g_sample = {};
    g_lastAttemptSequence = 0U - kRetryPeriodSamples;
}

void update(std::uint32_t encoderSequence)
{
    const std::uint32_t period =
        g_sample.connected ? kPollPeriodSamples : kRetryPeriodSamples;
    if ((encoderSequence - g_lastAttemptSequence) < period) {
        return;
    }
    g_lastAttemptSequence = encoderSequence;

    if (!g_sample.connected && !ping()) {
        ++g_sample.errors;
        return;
    }

    std::uint8_t digital = 0U;
    std::uint8_t analog[kChannelCount] = {};
    if (!readDigital(digital) || !readAnalog(analog)) {
        g_sample.connected = false;
        ++g_sample.errors;
        return;
    }

    g_sample.connected = true;
    g_sample.digital = digital;
    for (std::uint8_t i = 0U; i < kChannelCount; ++i) {
        g_sample.analog[i] = analog[i];
    }
    ++g_sample.sequence;
    g_sample.sampledAt = encoderSequence;
}

Sample latest()
{
    return g_sample;
}

bool ping()
{
    std::uint8_t response = 0U;
    return commandRead(kPingCommand, &response, 1U) &&
        response == kPingResponse;
}

bool readDigital(std::uint8_t &digital)
{
    return commandRead(kDigitalCommand, &digital, 1U);
}

bool readAnalog(std::uint8_t (&analog)[kChannelCount])
{
    return commandRead(kAllAnalogCommand, analog, kChannelCount);
}

}  // namespace GraySensor
