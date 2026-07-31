#ifndef PID_CONTROLLER_HPP
#define PID_CONTROLLER_HPP

#include <cstdint>

namespace Control {

enum class DerivativeMode : std::uint8_t {
    Error,
    Measurement,
};

struct PidConfig {
    float kp;
    float ki;
    float kd;
    float outputMin;
    float outputMax;
    float integralMin;
    float integralMax;
    DerivativeMode derivativeMode;
};

struct PidTerms {
    float setpoint;
    float measurement;
    float error;
    float proportional;
    float integral;
    float derivative;
    float unsaturatedOutput;
    float output;
    bool saturated;
};

/*
 * Reusable positional PID controller.
 *
 * The class has no hardware dependencies and performs no allocation. Call
 * update() at a known interval and pass that interval in seconds. Output and
 * integral limiting use conditional integration to prevent windup.
 */
class PidController {
public:
    PidController();
    explicit PidController(const PidConfig &config);

    void configure(const PidConfig &config);
    void setTunings(float kp, float ki, float kd);
    void setOutputLimits(float minimum, float maximum);
    void setIntegralLimits(float minimum, float maximum);
    void setDerivativeMode(DerivativeMode mode);

    void reset();
    void reset(float measurement);

    float update(float setpoint, float measurement, float dtSeconds);
    // Uses an externally calculated measurement rate. The derivative term is
    // -kd * measurementRate, which avoids derivative kick from setpoint
    // changes and allows the caller to filter a noisy sensor velocity.
    float updateWithMeasurementRate(
        float setpoint,
        float measurement,
        float measurementRate,
        float dtSeconds);

    const PidConfig &configuration() const;
    const PidTerms &latest() const;

private:
    PidConfig config_;
    PidTerms terms_;
    float integralState_;
    float previousError_;
    float previousMeasurement_;
    bool initialized_;

    float updateWithDerivativeRate(
        float setpoint,
        float measurement,
        float derivativeRate,
        float dtSeconds);
};

}  // namespace Control

#endif  // PID_CONTROLLER_HPP
