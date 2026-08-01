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
#include "ti_msp_dl_config.h"

namespace CarApp {

namespace {

AppMode g_currentMode = AppMode::K230StepperDebug;
bool g_modeActive = false;
enum class StartupState {
    WaitingForK230,
    SelectingMode,
    Running,
};
StartupState g_startupState = StartupState::WaitingForK230;
std::uint8_t g_b21PressCount = 0U;
constexpr std::uint32_t kButtonDebounceSamples =
    (Encoder::kSampleRateHz * 30U + 999U) / 1000U;
struct DebouncedButton {
    bool stablePressed;
    bool candidatePressed;
    std::uint32_t candidateSince;
    bool pressPending;
};
DebouncedButton g_b21Button = {};
DebouncedButton g_userButton = {};
bool g_mode1Stopped = false;
constexpr std::uint32_t kMode1StopDistanceMillimeters = 5940U;
// Calibrate this value by measuring the loaded tyre rolling circumference.
// The local reference chassis uses a measured 67.00 mm wheel diameter.
constexpr std::uint32_t kMode1WheelCircumferenceMicrometers = 210487U;
std::int32_t g_mode1PreviousRightCount = 0;
std::int32_t g_mode1PreviousLeftCount = 0;
std::uint64_t g_mode1WheelCountSum = 0U;
constexpr std::int16_t kMode3BallTargetX = 348;
constexpr float kMode3LineTargetRpm = 70.0F;
// Mode 3 line-tracking outer-loop gains are independent from mode 1.
constexpr float kMode3LineKp = 3.0F;
constexpr float kMode3LineKi = 0.0F;
constexpr float kMode3LineKd = 0.02F;
constexpr std::uint32_t kMode3LineRampSamples =
    4U * Encoder::kSampleRateHz;
std::uint32_t g_mode3LineRampStartSequence = 0U;
std::uint32_t g_mode3LineRampLastSequence = 0U;
bool g_mode3LineRampComplete = false;
constexpr std::int16_t kMode2InitialBallTargetX = 525;
constexpr std::int16_t kMode2TurnaroundX = 460;
constexpr std::int16_t kMode2FinalBallTargetX = 233;
constexpr std::int16_t kMode2BallDeadbandPixels = 18;
constexpr std::int16_t kMode3BallDeadbandPixels = 10;
constexpr std::uint32_t kBallFrameTimeoutSamples =
    Encoder::kSampleRateHz * 3U / 10U;
constexpr std::int16_t kMaximumBallFrameDeltaX = 120;
constexpr float kBallVelocityFilterTimeConstantSeconds = 0.10F;
constexpr float kBallVelocityMinimumDisplacementPixels = 3.0F;
constexpr float kBallVelocityOutputDeadbandPixelsPerSecond = 8.0F;
constexpr std::uint32_t kBalanceStepFrequencyHz = 1200U;
// Invert if positive motor angle makes the ball move away from the target.
constexpr std::int32_t kBalanceMotorPolarity = -1;
// Mode 2 gains are tuned independently and retain the current manual values.
constexpr float kMode2BalanceKp = 0.80F;
constexpr float kMode2BalanceKi = 0.080F;
constexpr float kMode2BalanceKd = 0.6F;
constexpr float kMode3BalanceKp = 0.7F;
constexpr float kMode3BalanceKi = 0.07F;
constexpr float kMode3BalanceKd = 0.60F;

bool g_stepperZeroEstablished = false;
Control::PidController g_ballPid({
    kMode2BalanceKp,
    kMode2BalanceKi,
    kMode2BalanceKd,
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
bool g_mode2TargetSwitched = false;
std::int16_t g_mode4BallTargetX = kMode3BallTargetX;
std::int16_t g_mode4CandidateTargetX = kMode3BallTargetX;
bool g_hasMode4CandidateTarget = false;

bool buttonPinIsPressed(std::uint32_t pins, std::uint32_t buttonPin)
{
    return (pins & buttonPin) == 0U;
}

void initializeButton(
    DebouncedButton &button, bool pressed, std::uint32_t sampleSequence)
{
    button.stablePressed = pressed;
    button.candidatePressed = pressed;
    button.candidateSince = sampleSequence;
    button.pressPending = false;
}

void initializeButtons(std::uint32_t sampleSequence)
{
    const std::uint32_t pins = DL_GPIO_readPins(
        BUTTONS_PORT, BUTTONS_B21_BUTTON_PIN | BUTTONS_USER_BUTTON_PIN);
    initializeButton(g_b21Button,
        buttonPinIsPressed(pins, BUTTONS_B21_BUTTON_PIN), sampleSequence);
    initializeButton(g_userButton,
        buttonPinIsPressed(pins, BUTTONS_USER_BUTTON_PIN), sampleSequence);
}

void updateButton(
    DebouncedButton &button, bool pressed, std::uint32_t sampleSequence)
{
    if (pressed != button.candidatePressed) {
        button.candidatePressed = pressed;
        button.candidateSince = sampleSequence;
        return;
    }
    if (pressed == button.stablePressed ||
        (sampleSequence - button.candidateSince) < kButtonDebounceSamples) {
        return;
    }

    button.stablePressed = pressed;
    if (pressed) {
        button.pressPending = true;
    }
}

void updateButtons(std::uint32_t sampleSequence)
{
    const std::uint32_t pins = DL_GPIO_readPins(
        BUTTONS_PORT, BUTTONS_B21_BUTTON_PIN | BUTTONS_USER_BUTTON_PIN);
    updateButton(g_b21Button,
        buttonPinIsPressed(pins, BUTTONS_B21_BUTTON_PIN), sampleSequence);
    updateButton(g_userButton,
        buttonPinIsPressed(pins, BUTTONS_USER_BUTTON_PIN), sampleSequence);
}

bool takeButtonPress(DebouncedButton &button)
{
    const bool pending = button.pressPending;
    button.pressPending = false;
    return pending;
}

std::int32_t roundToInt(float value)
{
    return static_cast<std::int32_t>(
        value >= 0.0F ? value + 0.5F : value - 0.5F);
}

float absoluteValue(float value)
{
    return value >= 0.0F ? value : -value;
}

std::int32_t absoluteDifference(std::int16_t left, std::int16_t right)
{
    const std::int32_t difference =
        static_cast<std::int32_t>(left) - right;
    return difference >= 0 ? difference : -difference;
}

bool ballFrameHasExcessiveJump(
    const K230Protocol::BallPosition &position)
{
    if (!g_hasBallFrame || !g_latestBallPosition.detected ||
        !position.detected) {
        return false;
    }

    return absoluteDifference(position.x, g_latestBallPosition.x) >
        kMaximumBallFrameDeltaX;
}

std::uint64_t absoluteCountDelta(
    std::int32_t current, std::int32_t previous)
{
    const std::int64_t delta =
        static_cast<std::int64_t>(current) - previous;
    return static_cast<std::uint64_t>(delta >= 0 ? delta : -delta);
}

std::uint32_t updateMode1Distance(const Encoder::Sample &sample)
{
    g_mode1WheelCountSum +=
        absoluteCountDelta(sample.rightTotal, g_mode1PreviousRightCount) +
        absoluteCountDelta(sample.leftTotal, g_mode1PreviousLeftCount);
    g_mode1PreviousRightCount = sample.rightTotal;
    g_mode1PreviousLeftCount = sample.leftTotal;

    // Centre travel = average(left travel, right travel).
    constexpr std::uint64_t kDenominator =
        2ULL * Encoder::kCountsPerWheelRevolution * 1000ULL;
    const std::uint64_t distanceMillimeters =
        (g_mode1WheelCountSum * kMode1WheelCircumferenceMicrometers +
            kDenominator / 2ULL) /
        kDenominator;
    return distanceMillimeters > 0xFFFFFFFFULL
        ? 0xFFFFFFFFU
        : static_cast<std::uint32_t>(distanceMillimeters);
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
            if (ballFrameHasExcessiveJump(position)) {
                // Keep the last accepted coordinates but advance the frame
                // sequence so this received frame still runs one PID update
                // using the previous measurement.
                position.detected = g_latestBallPosition.detected;
                position.x = g_latestBallPosition.x;
            }
            g_latestBallPosition = position;
            g_hasBallFrame = true;
            g_lastBallFrameSampleSequence = sampleSequence;
            PidDashboard::setBallPosition(position);
        }
        ++count;
    }
}

bool ballFrameIsFresh(std::uint32_t sampleSequence)
{
    return g_hasBallFrame &&
        (sampleSequence - g_lastBallFrameSampleSequence) <=
            kBallFrameTimeoutSamples;
}

void updateBallBalance(
    std::uint32_t sampleSequence, std::int16_t targetX,
    std::int16_t deadbandPixels)
{
    if (!ballFrameIsFresh(sampleSequence) ||
        !g_latestBallPosition.detected) {
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
            targetX - g_latestBallPosition.x);
    if (error >= -deadbandPixels && error <= deadbandPixels) {
        proportionalMeasurement =
            static_cast<float>(targetX);
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
            static_cast<float>(targetX),
            proportionalMeasurement,
            effectiveBallVelocity,
            dtSeconds);
    const std::int32_t targetPulses =
        kBalanceMotorPolarity * roundToInt(pidOutput);
    if (!Stepper::isEnabled()) {
        // Modes 2/3/4 leave the mechanism free after power-up and only wake
        // the driver when the first usable frame actually starts PID control.
        Stepper::setEnabled(true);
    }
    if (!Stepper::moveTo(targetPulses, kBalanceStepFrequencyHz)) {
        resetBallBalance();
        return;
    }

    g_lastControlledBallSequence =
        g_latestBallPosition.sequence;
    g_lastBalanceUpdateSampleSequence =
        g_lastBallFrameSampleSequence;
}

void updateMode2BallBalance(std::uint32_t sampleSequence)
{
    if (!g_mode2TargetSwitched && ballFrameIsFresh(sampleSequence) &&
        g_latestBallPosition.detected &&
        g_latestBallPosition.x >= kMode2TurnaroundX) {
        g_mode2TargetSwitched = true;
    }

    updateBallBalance(sampleSequence,
        g_mode2TargetSwitched
            ? kMode2FinalBallTargetX
            : kMode2InitialBallTargetX,
        kMode2BallDeadbandPixels);
}

void enterK230StepperControl()
{
    LineTracking::stop();
    SpeedControl::pidEnabled = false;
    SpeedControl::stop();
    TB6612::coast();

    Stepper::stop();
    Stepper::setEnabled(false);
    if (!g_stepperZeroEstablished) {
        Stepper::setPositionPulses(0);
        g_stepperZeroEstablished = true;
    }
    resetBallBalance();
    PidDashboard::setView(PidDashboard::View::K230Monitor);
}

void updateK230TwoStage(std::uint32_t sampleSequence)
{
    updateMode2BallBalance(sampleSequence);
    PidDashboard::update(sampleSequence);
}

void enterK230TwoStage()
{
    g_mode2TargetSwitched = false;
    g_ballPid.setTunings(
        kMode2BalanceKp, kMode2BalanceKi, kMode2BalanceKd);
    enterK230StepperControl();
}

void enterK230FixedTarget()
{
    g_ballPid.setTunings(
        kMode3BalanceKp, kMode3BalanceKi, kMode3BalanceKd);
    enterK230StepperControl();

    GraySensor::init();
    LineTracking::init();
    LineTracking::setTunings(
        kMode3LineKp, kMode3LineKi, kMode3LineKd);
    LineTracking::setBaseRpm(0.0F);
    g_mode3LineRampStartSequence = Encoder::latest().sequence;
    g_mode3LineRampLastSequence =
        g_mode3LineRampStartSequence - 1U;
    g_mode3LineRampComplete = false;
    SpeedControl::pidEnabled = true;
}

void updateMode3LineSpeedRamp(std::uint32_t sampleSequence)
{
    if (g_mode3LineRampComplete ||
        sampleSequence == g_mode3LineRampLastSequence) {
        return;
    }
    g_mode3LineRampLastSequence = sampleSequence;

    const std::uint32_t elapsedSamples =
        sampleSequence - g_mode3LineRampStartSequence;
    if (elapsedSamples >= kMode3LineRampSamples) {
        LineTracking::setBaseRpm(kMode3LineTargetRpm);
        g_mode3LineRampComplete = true;
        return;
    }

    const float targetRpm = kMode3LineTargetRpm *
        static_cast<float>(elapsedSamples) /
        static_cast<float>(kMode3LineRampSamples);
    LineTracking::setBaseRpm(targetRpm);
}

void updateK230StepperDebug(std::uint32_t sampleSequence)
{
    updateMode3LineSpeedRamp(sampleSequence);
    GraySensor::update(sampleSequence);
    LineTracking::update(sampleSequence);
    updateBallBalance(
        sampleSequence, kMode3BallTargetX, kMode3BallDeadbandPixels);
    PidDashboard::update(sampleSequence);
}

void updateK230CapturedTarget(std::uint32_t sampleSequence)
{
    updateMode3LineSpeedRamp(sampleSequence);
    GraySensor::update(sampleSequence);
    LineTracking::update(sampleSequence);
    updateBallBalance(
        sampleSequence, g_mode4BallTargetX, kMode3BallDeadbandPixels);
    PidDashboard::update(sampleSequence);
}

void exitK230StepperControl()
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
    g_mode1Stopped = false;
    const Encoder::Sample sample = Encoder::latest();
    g_mode1PreviousRightCount = sample.rightTotal;
    g_mode1PreviousLeftCount = sample.leftTotal;
    g_mode1WheelCountSum = 0U;
    PidDashboard::startLineTrackingRuntime(sample.sequence);
}

void updateLineTracking(const Encoder::Sample &sample)
{
    const std::uint32_t sampleSequence = sample.sequence;
    if (g_mode1Stopped) {
        PidDashboard::update(sampleSequence);
        return;
    }

    const std::uint32_t distanceMillimeters =
        updateMode1Distance(sample);
    PidDashboard::setLineTrackingDistanceMillimeters(distanceMillimeters);

    if (distanceMillimeters >= kMode1StopDistanceMillimeters) {
        // Clear both wheel-speed targets and latch active braking immediately.
        LineTracking::stop();
        SpeedControl::pidEnabled = false;
        SpeedControl::brake();
        g_mode1Stopped = true;
        PidDashboard::stopLineTrackingRuntime(sampleSequence);
        PidDashboard::update(sampleSequence);
        return;
    }

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
        case AppMode::K230CapturedTarget:
            exitLineTracking();
            exitK230StepperControl();
            break;
        case AppMode::K230TwoStage:
            exitK230StepperControl();
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
        case AppMode::K230CapturedTarget:
            enterK230FixedTarget();
            break;
        case AppMode::K230TwoStage:
            enterK230TwoStage();
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
    initializeButtons(Encoder::latest().sequence);

    g_stepperZeroEstablished = false;
    g_latestBallPosition = {};
    g_hasBallFrame = false;
    g_lastBallFrameSampleSequence = 0U;
    g_mode4BallTargetX = kMode3BallTargetX;
    g_mode4CandidateTargetX = kMode3BallTargetX;
    g_hasMode4CandidateTarget = false;
    resetBallBalance();
    g_modeActive = false;
    g_startupState = StartupState::WaitingForK230;
    g_b21PressCount = 0U;
    PidDashboard::setView(PidDashboard::View::K230Monitor);
}

void runOnce()
{
    const Encoder::Sample sample = Encoder::latest();
    updateButtons(sample.sequence);

    if (g_startupState == StartupState::WaitingForK230) {
        serviceK230Link(sample.sequence);
        PidDashboard::update(sample.sequence);
        if (g_hasBallFrame) {
            Stepper::stop();
            Stepper::setEnabled(false);
            SpeedControl::pidEnabled = false;
            SpeedControl::stop();
            TB6612::coast();
            g_startupState = StartupState::SelectingMode;
            initializeButtons(sample.sequence);
            PidDashboard::setModeSelection(0U);
            PidDashboard::setView(PidDashboard::View::ModeSelection);
        }
        return;
    }

    if (g_startupState == StartupState::SelectingMode) {
        // Keep the UART buffer current while the operator chooses a mode.
        serviceK230Link(sample.sequence);
        g_hasMode4CandidateTarget =
            ballFrameIsFresh(sample.sequence) &&
            g_latestBallPosition.detected;
        if (g_hasMode4CandidateTarget) {
            // This remains the most recent accepted detected frame preceding
            // the eventual USER-button confirmation.
            g_mode4CandidateTargetX = g_latestBallPosition.x;
        }
        if (takeButtonPress(g_b21Button)) {
            g_b21PressCount = g_b21PressCount < 99U
                ? static_cast<std::uint8_t>(g_b21PressCount + 1U)
                : 1U;
            PidDashboard::setModeSelection(g_b21PressCount);
        }
        if (takeButtonPress(g_userButton) && g_b21PressCount != 0U) {
            const std::uint8_t modeNumber = static_cast<std::uint8_t>(
                ((g_b21PressCount - 1U) % 4U) + 1U);
            if (modeNumber == 4U && !g_hasMode4CandidateTarget) {
                // Mode 4 cannot define its fixed target until at least one
                // detected ball frame has arrived before confirmation.
                PidDashboard::update(sample.sequence);
                return;
            }
            if (modeNumber == 1U) {
                selectMode(AppMode::LineTracking);
            } else if (modeNumber == 2U) {
                selectMode(AppMode::K230TwoStage);
            } else if (modeNumber == 3U) {
                selectMode(AppMode::K230StepperDebug);
            } else {
                g_mode4BallTargetX = g_mode4CandidateTargetX;
                selectMode(AppMode::K230CapturedTarget);
            }
            g_startupState = StartupState::Running;
        } else {
            PidDashboard::update(sample.sequence);
        }
        return;
    }

    // Mode 1 deliberately does not read or parse K230 data.
    if (g_currentMode != AppMode::LineTracking) {
        serviceK230Link(sample.sequence);
    }
    switch (g_currentMode) {
        case AppMode::K230StepperDebug:
            updateK230StepperDebug(sample.sequence);
            break;
        case AppMode::K230CapturedTarget:
            updateK230CapturedTarget(sample.sequence);
            break;
        case AppMode::K230TwoStage:
            updateK230TwoStage(sample.sequence);
            break;
        case AppMode::LineTracking:
            updateLineTracking(sample);
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
