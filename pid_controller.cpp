#include "pid_controller.hpp"

namespace {

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

void orderLimits(float &minimum, float &maximum)
{
    if (minimum > maximum) {
        const float temporary = minimum;
        minimum = maximum;
        maximum = temporary;
    }
}

}  // namespace

namespace Control {

PidController::PidController()
    : config_{0.0F, 0.0F, 0.0F,
              -1.0F, 1.0F, -1.0F, 1.0F,
              DerivativeMode::Measurement},
      terms_{},
      integralState_(0.0F),
      previousError_(0.0F),
      previousMeasurement_(0.0F),
      initialized_(false)
{
}

PidController::PidController(const PidConfig &config)
    : PidController()
{
    configure(config);
}

void PidController::configure(const PidConfig &config)
{
    config_ = config;
    orderLimits(config_.outputMin, config_.outputMax);
    orderLimits(config_.integralMin, config_.integralMax);
    reset();
}

void PidController::setTunings(float kp, float ki, float kd)
{
    config_.kp = kp;
    config_.ki = ki;
    config_.kd = kd;
    reset();
}

void PidController::setOutputLimits(float minimum, float maximum)
{
    orderLimits(minimum, maximum);
    config_.outputMin = minimum;
    config_.outputMax = maximum;
    reset();
}

void PidController::setIntegralLimits(float minimum, float maximum)
{
    orderLimits(minimum, maximum);
    config_.integralMin = minimum;
    config_.integralMax = maximum;
    integralState_ =
        clampValue(integralState_, minimum, maximum);
}

void PidController::setDerivativeMode(DerivativeMode mode)
{
    config_.derivativeMode = mode;
    reset();
}

void PidController::reset()
{
    terms_ = {};
    integralState_ = 0.0F;
    previousError_ = 0.0F;
    previousMeasurement_ = 0.0F;
    initialized_ = false;
}

void PidController::reset(float measurement)
{
    reset();
    terms_.measurement = measurement;
    previousMeasurement_ = measurement;
    initialized_ = true;
}

float PidController::update(
    float setpoint, float measurement, float dtSeconds)
{
    if (dtSeconds <= 0.0F) {
        return terms_.output;
    }

    const float error = setpoint - measurement;
    float derivativeRate = 0.0F;
    if (initialized_) {
        if (config_.derivativeMode == DerivativeMode::Error) {
            derivativeRate =
                (error - previousError_) / dtSeconds;
        } else {
            derivativeRate =
                -(measurement - previousMeasurement_) / dtSeconds;
        }
    }

    const float proportional = config_.kp * error;
    const float derivative = config_.kd * derivativeRate;
    const float candidateIntegral = clampValue(
        integralState_ + error * dtSeconds,
        config_.integralMin, config_.integralMax);
    const float candidateRaw =
        proportional + config_.ki * candidateIntegral + derivative;

    const bool saturatedHigh =
        candidateRaw > config_.outputMax;
    const bool saturatedLow =
        candidateRaw < config_.outputMin;
    const float integralDirection = config_.ki * error;
    const bool drivesBackFromHigh =
        saturatedHigh && integralDirection < 0.0F;
    const bool drivesBackFromLow =
        saturatedLow && integralDirection > 0.0F;

    if ((!saturatedHigh && !saturatedLow) ||
        drivesBackFromHigh || drivesBackFromLow) {
        integralState_ = candidateIntegral;
    }

    const float integral = config_.ki * integralState_;
    const float rawOutput =
        proportional + integral + derivative;
    const float output = clampValue(
        rawOutput, config_.outputMin, config_.outputMax);

    terms_.setpoint = setpoint;
    terms_.measurement = measurement;
    terms_.error = error;
    terms_.proportional = proportional;
    terms_.integral = integral;
    terms_.derivative = derivative;
    terms_.unsaturatedOutput = rawOutput;
    terms_.output = output;
    terms_.saturated =
        rawOutput > config_.outputMax ||
        rawOutput < config_.outputMin;

    previousError_ = error;
    previousMeasurement_ = measurement;
    initialized_ = true;
    return output;
}

const PidConfig &PidController::configuration() const
{
    return config_;
}

const PidTerms &PidController::latest() const
{
    return terms_;
}

}  // namespace Control
