#include "car_app.hpp"

#include "encoder.hpp"
#include "gray_sensor.hpp"
#include "k230_protocol.hpp"
#include "k230_uart.hpp"
#include "line_tracking.hpp"
#include "pid_controller.hpp"
#include "pid_dashboard.hpp"
#include "speed_controller.hpp"
#include "stepper.hpp"
#include "tb6612.hpp"

namespace CarApp {

namespace {

AppMode g_currentMode = AppMode::K230StepperDebug;
bool g_modeActive = false;
constexpr std::int16_t kBallTargetX = 345;
constexpr std::int16_t kBallDeadbandPixels = 20;
constexpr std::uint32_t kBallFrameTimeoutSamples =
    Encoder::kSampleRateHz * 3U / 10U;
constexpr float kBallVelocityFilterTimeConstantSeconds = 0.10F;
constexpr float kBallVelocityMinimumDisplacementPixels = 3.0F;
constexpr float kBallVelocityOutputDeadbandPixelsPerSecond = 8.0F;
constexpr std::uint32_t kBalanceStepFrequencyHz = 1200U;
// Invert if positive motor angle makes the ball move away from X=345.
constexpr std::int32_t kBalanceMotorPolarity = -1;
constexpr float kBalanceKp = 0.7F;
constexpr float kBalanceKi = 0.07F;
constexpr float kBalanceKd = 0.6F;

bool g_stepperZeroEstablished = false;
Control::PidController g_ballPid({
    kBalanceKp,
    kBalanceKi,
    kBalanceKd,
    -static_cast<float>(Stepper::kTravelLimitPulses),
    static_cast<float>(Stepper::kTravelLimitPulses),
    -1500.0F,
    1500.0F,
    Control::DerivativeMode::Measurement,
});
K230Protocol::BallPosition g_latestBallPosition = {};
bool g_hasBallFrame = false;
bool g_ballPidActive = false;
std::uint32_t g_lastBallFrameSampleSequence = 0U;
std::uint32_t g_lastControlledBallSequence = 0U;
std::uint32_t g_lastBalanceUpdateSampleSequence = 0U;
float g_previousBallX = 0.0F;
float g_filteredBallVelocityPixelsPerSecond = 0.0F;
bool g_ballVelocityInitialized = false;

std::int32_t roundToInt(float value)
{
    return static_cast<std::int32_t>(
        value >= 0.0F ? value + 0.5F : value - 0.5F);
}

float absoluteValue(float value)
{
    return value >= 0.0F ? value : -value;
}

void resetBallBalance()
{
    if (g_ballPidActive) {
        Stepper::stop();
    }
    g_ballPid.reset();
    g_ballPidActive = false;
    g_lastControlledBallSequence = 0U;
    g_lastBalanceUpdateSampleSequence = 0U;
    g_previousBallX = 0.0F;
    g_filteredBallVelocityPixelsPerSecond = 0.0F;
    g_ballVelocityInitialized = false;
}

void serviceK230Link(std::uint32_t sampleSequence)
{
    constexpr std::size_t kMaximumBytesPerRun = 64U;
    std::size_t count = 0U;
    std::uint8_t data;
    K230Protocol::BallPosition position;

    while (count < kMaximumBytesPerRun && K230Uart::read(data)) {
        K230Uart::write(data);
        if (K230Protocol::consume(data, position)) {
            g_latestBallPosition = position;
            g_hasBallFrame = true;
            g_lastBallFrameSampleSequence = sampleSequence;
            PidDashboard::setBallPosition(position);
        }
        ++count;
    }
}

void updateBallBalance(std::uint32_t sampleSequence)
{
    const bool frameFresh =
        g_hasBallFrame &&
        (sampleSequence - g_lastBallFrameSampleSequence) <=
            kBallFrameTimeoutSamples;
    if (!frameFresh || !g_latestBallPosition.detected) {
        resetBallBalance();
        return;
    }

    if (g_latestBallPosition.sequence ==
        g_lastControlledBallSequence) {
        return;
    }

    const float rawMeasurement =
        static_cast<float>(g_latestBallPosition.x);
    float proportionalMeasurement = rawMeasurement;
    const std::int16_t error =
        static_cast<std::int16_t>(
            kBallTargetX - g_latestBallPosition.x);
    if (error >= -kBallDeadbandPixels &&
        error <= kBallDeadbandPixels) {
        proportionalMeasurement =
            static_cast<float>(kBallTargetX);
    }

    float dtSeconds = 1.0F /
        static_cast<float>(Encoder::kSampleRateHz);
    if (g_ballPidActive) {
        std::uint32_t elapsedSamples =
            g_lastBallFrameSampleSequence -
            g_lastBalanceUpdateSampleSequence;
        if (elapsedSamples == 0U) {
            elapsedSamples = 1U;
        } else if (elapsedSamples > kBallFrameTimeoutSamples) {
            elapsedSamples = kBallFrameTimeoutSamples;
        }
        dtSeconds = static_cast<float>(elapsedSamples) /
            static_cast<float>(Encoder::kSampleRateHz);
    } else {
        g_ballPid.reset(proportionalMeasurement);
        g_ballPidActive = true;
    }

    float rawBallVelocity = 0.0F;
    if (g_ballVelocityInitialized) {
        const float displacement = rawMeasurement - g_previousBallX;
        if (absoluteValue(displacement) >
                kBallVelocityMinimumDisplacementPixels) {
            rawBallVelocity = displacement / dtSeconds;
        }
    } else {
        g_ballVelocityInitialized = true;
    }
    g_previousBallX = rawMeasurement;
    const float filterAlpha =
        dtSeconds /
        (kBallVelocityFilterTimeConstantSeconds + dtSeconds);
    g_filteredBallVelocityPixelsPerSecond +=
        filterAlpha *
        (rawBallVelocity -
            g_filteredBallVelocityPixelsPerSecond);
    const float effectiveBallVelocity =
        absoluteValue(g_filteredBallVelocityPixelsPerSecond) <
            kBallVelocityOutputDeadbandPixelsPerSecond
        ? 0.0F
        : g_filteredBallVelocityPixelsPerSecond;

    // u = Kp * (targetX - measuredX)
    //     + Ki * integral(error)
    //     - Kd * effectiveBallVelocity
    const float pidOutput =
        g_ballPid.updateWithMeasurementRate(
            static_cast<float>(kBallTargetX),
            proportionalMeasurement,
            effectiveBallVelocity,
            dtSeconds);
    const std::int32_t targetPulses =
        kBalanceMotorPolarity * roundToInt(pidOutput);
    if (!Stepper::moveTo(targetPulses, kBalanceStepFrequencyHz)) {
        resetBallBalance();
        return;
    }

    g_lastControlledBallSequence =
        g_latestBallPosition.sequence;
    g_lastBalanceUpdateSampleSequence =
        g_lastBallFrameSampleSequence;
}

void enterK230StepperDebug()
{
    LineTracking::stop();
    SpeedControl::pidEnabled = false;
    SpeedControl::stop();
    TB6612::coast();

    Stepper::stop();
    Stepper::setEnabled(true);
    if (!g_stepperZeroEstablished) {
        Stepper::setPositionPulses(0);
        g_stepperZeroEstablished = true;
    }
    resetBallBalance();
    PidDashboard::setView(PidDashboard::View::K230Monitor);
}

void updateK230StepperDebug(std::uint32_t sampleSequence)
{
    updateBallBalance(sampleSequence);
    PidDashboard::update(sampleSequence);
}

void exitK230StepperDebug()
{
    resetBallBalance();
    Stepper::stop();
    Stepper::setEnabled(false);
}

void enterLineTracking()
{
    Stepper::stop();
    Stepper::setEnabled(false);

    GraySensor::init();
    LineTracking::init();
    SpeedControl::pidEnabled = true;
    PidDashboard::setView(PidDashboard::View::SpeedControl);
}

void updateLineTracking(std::uint32_t sampleSequence)
{
    GraySensor::update(sampleSequence);
    LineTracking::update(sampleSequence);
    PidDashboard::update(sampleSequence);
}

void exitLineTracking()
{
    LineTracking::stop();
    SpeedControl::pidEnabled = false;
    SpeedControl::stop();
    TB6612::coast();
}

void exitCurrentMode()
{
    switch (g_currentMode) {
        case AppMode::K230StepperDebug:
            exitK230StepperDebug();
            break;
        case AppMode::LineTracking:
            exitLineTracking();
            break;
    }
}

void enterCurrentMode()
{
    switch (g_currentMode) {
        case AppMode::K230StepperDebug:
            enterK230StepperDebug();
            break;
        case AppMode::LineTracking:
            enterLineTracking();
            break;
    }
}

}  // namespace

void init()
{
    /*
     * Arm K230 reception before OLED initialization. OLED_Init() deliberately
     * waits for the panel to power up, during which the K230 may already send
     * its startup frame.
     */
    K230Protocol::reset();
    K230Uart::init();

    Stepper::init();
    TB6612::init();
    PidDashboard::init();

    SpeedControl::init();
    SpeedControl::setTunings(
        SpeedControl::kDefaultKp,
        SpeedControl::kDefaultKi,
        SpeedControl::kDefaultKd);
    Encoder::init();

    g_stepperZeroEstablished = false;
    g_latestBallPosition = {};
    g_hasBallFrame = false;
    g_lastBallFrameSampleSequence = 0U;
    resetBallBalance();
    g_modeActive = false;
    selectMode(AppMode::K230StepperDebug);
}

void runOnce()
{
    const Encoder::Sample sample = Encoder::latest();
    serviceK230Link(sample.sequence);
    switch (g_currentMode) {
        case AppMode::K230StepperDebug:
            updateK230StepperDebug(sample.sequence);
            break;
        case AppMode::LineTracking:
            updateLineTracking(sample.sequence);
            break;
    }
}

void selectMode(AppMode mode)
{
    if (g_modeActive && mode == g_currentMode) {
        return;
    }

    if (g_modeActive) {
        exitCurrentMode();
    }

    g_currentMode = mode;
    enterCurrentMode();
    g_modeActive = true;
}

AppMode currentMode()
{
    return g_currentMode;
}

}  // namespace CarApp
