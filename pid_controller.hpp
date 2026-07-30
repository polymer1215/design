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

    const PidConfig &configuration() const;
    const PidTerms &latest() const;

private:
    PidConfig config_;
    PidTerms terms_;
    float integralState_;
    float previousError_;
    float previousMeasurement_;
    bool initialized_;
};

}  // namespace Control

#endif  // PID_CONTROLLER_HPP
