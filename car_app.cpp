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
constexpr std::int16_t kOutboundBallTargetX = 450;
constexpr std::int16_t kHoldingBallTargetX = 227;
constexpr std::int16_t kBallDeadbandPixels = 6;
constexpr std::uint32_t kBallFrameTimeoutSamples =
    Encoder::kSampleRateHz * 3U / 10U;
constexpr float kMaximumStepSpeedPps = 2500.0F;
constexpr float kMaximumStepAccelerationPps2 = 40000.0F;
constexpr float kTrajectoryPositionGain = 20.0F;
// Positive motor angle makes X decrease on the assembled mechanism.
constexpr std::int32_t kBalanceMotorPolarity = -1;
constexpr float kBalanceKp = 0.444F;
constexpr float kBalanceKi = 0.005F;
constexpr float kBalanceKd = 0.0F;

enum class BallTargetPhase {
    MoveToOutboundTarget,
    ReturnAndHold,
};

bool g_stepperZeroEstablished = false;
BallTargetPhase g_ballTargetPhase =
    BallTargetPhase::MoveToOutboundTarget;
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
std::uint32_t g_lastTrajectorySampleSequence = 0U;
std::int32_t g_balanceTargetPulses = 0;
float g_balanceVelocityPps = 0.0F;

std::int16_t currentBallTargetX()
{
    return g_ballTargetPhase ==
            BallTargetPhase::MoveToOutboundTarget
        ? kOutboundBallTargetX
        : kHoldingBallTargetX;
}

float clampValue(float value, float minimum, float maximum)
{
    if (value > maximum) {
        return maximum;
    }
    if (value < minimum) {
        return minimum;
    }
    return value;
}

std::int32_t roundToInt(float value)
{
    return static_cast<std::int32_t>(
        value >= 0.0F ? value + 0.5F : value - 0.5F);
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
    g_lastTrajectorySampleSequence = 0U;
    g_balanceTargetPulses = Stepper::positionPulses();
    g_balanceVelocityPps = 0.0F;
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

void updateBallTrajectory(std::uint32_t sampleSequence)
{
    if (!g_ballPidActive ||
        sampleSequence == g_lastTrajectorySampleSequence) {
        return;
    }

    std::uint32_t elapsedSamples =
        sampleSequence - g_lastTrajectorySampleSequence;
    constexpr std::uint32_t kMaximumElapsedSamples = 5U;
    if (elapsedSamples > kMaximumElapsedSamples) {
        elapsedSamples = kMaximumElapsedSamples;
    }
    g_lastTrajectorySampleSequence = sampleSequence;

    const float dtSeconds =
        static_cast<float>(elapsedSamples) /
        static_cast<float>(Encoder::kSampleRateHz);
    const std::int32_t positionError =
        g_balanceTargetPulses - Stepper::positionPulses();
    const float desiredVelocity = clampValue(
        kTrajectoryPositionGain *
            static_cast<float>(positionError),
        -kMaximumStepSpeedPps,
        kMaximumStepSpeedPps);
    const float maximumVelocityChange =
        kMaximumStepAccelerationPps2 * dtSeconds;
    const float velocityChange = clampValue(
        desiredVelocity - g_balanceVelocityPps,
        -maximumVelocityChange,
        maximumVelocityChange);
    g_balanceVelocityPps = clampValue(
        g_balanceVelocityPps + velocityChange,
        -kMaximumStepSpeedPps,
        kMaximumStepSpeedPps);

    if (positionError == 0 &&
        g_balanceVelocityPps == 0.0F) {
        Stepper::stop();
        return;
    }

    std::int32_t signedFrequency =
        roundToInt(g_balanceVelocityPps);
    const std::int32_t minimumFrequency =
        static_cast<std::int32_t>(
            Stepper::kMinimumStepFrequencyHz);
    if (signedFrequency > 0 &&
        signedFrequency < minimumFrequency) {
        signedFrequency = minimumFrequency;
    } else if (signedFrequency < 0 &&
        signedFrequency > -minimumFrequency) {
        signedFrequency = -minimumFrequency;
    }

    if (!Stepper::setVelocity(signedFrequency)) {
        resetBallBalance();
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

    if (g_latestBallPosition.sequence !=
        g_lastControlledBallSequence) {
        float measurement =
            static_cast<float>(g_latestBallPosition.x);
        bool targetSwitched = false;
        if (g_ballTargetPhase ==
                BallTargetPhase::MoveToOutboundTarget &&
            g_latestBallPosition.x >=
                kOutboundBallTargetX -
                    kBallDeadbandPixels) {
            g_ballTargetPhase =
                BallTargetPhase::ReturnAndHold;
            targetSwitched = true;
        }

        const std::int16_t targetX = currentBallTargetX();
        const std::int16_t error =
            static_cast<std::int16_t>(
                targetX - g_latestBallPosition.x);
        if (error >= -kBallDeadbandPixels &&
            error <= kBallDeadbandPixels) {
            measurement = static_cast<float>(targetX);
        }

        float dtSeconds = 1.0F /
            static_cast<float>(Encoder::kSampleRateHz);
        if (g_ballPidActive) {
            std::uint32_t elapsedSamples =
                g_lastBallFrameSampleSequence -
                g_lastBalanceUpdateSampleSequence;
            if (elapsedSamples == 0U) {
                elapsedSamples = 1U;
            } else if (elapsedSamples >
                kBallFrameTimeoutSamples) {
                elapsedSamples = kBallFrameTimeoutSamples;
            }
            dtSeconds = static_cast<float>(elapsedSamples) /
                static_cast<float>(Encoder::kSampleRateHz);
        } else {
            g_ballPid.reset(measurement);
            g_ballPidActive = true;
            g_balanceVelocityPps = 0.0F;
            g_lastTrajectorySampleSequence = sampleSequence;
        }
        if (targetSwitched && g_ballPidActive) {
            g_ballPid.reset(measurement);
        }

        const float pidOutput = g_ballPid.update(
            static_cast<float>(targetX),
            measurement,
            dtSeconds);
        g_balanceTargetPulses =
            kBalanceMotorPolarity * roundToInt(pidOutput);
        if (g_balanceTargetPulses >
            Stepper::kTravelLimitPulses) {
            g_balanceTargetPulses =
                Stepper::kTravelLimitPulses;
        } else if (g_balanceTargetPulses <
            -Stepper::kTravelLimitPulses) {
            g_balanceTargetPulses =
                -Stepper::kTravelLimitPulses;
        }

        g_lastControlledBallSequence =
            g_latestBallPosition.sequence;
        g_lastBalanceUpdateSampleSequence =
            g_lastBallFrameSampleSequence;
    }

    updateBallTrajectory(sampleSequence);
}

void enterK230StepperDebug()
{
    LineTracking::stop();
    SpeedControl::pidEnabled = false;
    SpeedControl::stop();
    TB6612::coast();

    Stepper::stop();
    Stepper::setEnabled(true);
    g_ballTargetPhase =
        BallTargetPhase::MoveToOutboundTarget;
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
