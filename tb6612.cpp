#include "tb6612.hpp"

#include "ti_msp_dl_config.h"

namespace {

constexpr std::uint32_t kPwmPeriodCounts = 4000U;

enum class Motor {
    LeftA,
    RightB,
};

std::int16_t clampCommand(std::int16_t command)
{
    if (command > TB6612::kMaxCommand) {
        return TB6612::kMaxCommand;
    }
    if (command < -TB6612::kMaxCommand) {
        return -TB6612::kMaxCommand;
    }
    return command;
}

std::uint32_t compareForMagnitude(std::uint16_t magnitude)
{
    if (magnitude == 0U) {
        return kPwmPeriodCounts;
    }

    // EDGE_ALIGN starts high and becomes low at the compare event.
    // A lower compare value therefore produces a larger high-time duty cycle.
    return ((kPwmPeriodCounts + 1U) *
            (TB6612::kMaxCommand - magnitude)) /
           TB6612::kMaxCommand;
}

void writePwm(Motor motor, std::uint16_t magnitude)
{
    const DL_TIMER_CC_INDEX channel =
        (motor == Motor::LeftA) ? GPIO_MOTOR_PWM_C1_IDX
                                : GPIO_MOTOR_PWM_C0_IDX;

    DL_Timer_setCaptureCompareValue(
        MOTOR_PWM_INST, compareForMagnitude(magnitude), channel);
}

void setDirection(Motor motor, bool forward)
{
    if (motor == Motor::LeftA) {
        if (forward) {
            DL_GPIO_setPins(TB6612_PORT, TB6612_AIN1_PIN);
            DL_GPIO_clearPins(TB6612_PORT, TB6612_AIN2_PIN);
        } else {
            DL_GPIO_clearPins(TB6612_PORT, TB6612_AIN1_PIN);
            DL_GPIO_setPins(TB6612_PORT, TB6612_AIN2_PIN);
        }
    } else {
        if (forward) {
            DL_GPIO_setPins(TB6612_PORT, TB6612_BIN1_PIN);
            DL_GPIO_clearPins(TB6612_PORT, TB6612_BIN2_PIN);
        } else {
            DL_GPIO_clearPins(TB6612_PORT, TB6612_BIN1_PIN);
            DL_GPIO_setPins(TB6612_PORT, TB6612_BIN2_PIN);
        }
    }
}

void coastMotor(Motor motor)
{
    writePwm(motor, 0U);

    if (motor == Motor::LeftA) {
        DL_GPIO_clearPins(
            TB6612_PORT, TB6612_AIN1_PIN | TB6612_AIN2_PIN);
    } else {
        DL_GPIO_clearPins(
            TB6612_PORT, TB6612_BIN1_PIN | TB6612_BIN2_PIN);
    }
}

void setMotor(Motor motor, std::int16_t command)
{
    command = clampCommand(command);
    if (command == 0) {
        coastMotor(motor);
        return;
    }

    // Remove drive before changing direction to avoid a bridge transient.
    writePwm(motor, 0U);
    setDirection(motor, command > 0);

    const std::uint16_t magnitude = static_cast<std::uint16_t>(
        (command > 0) ? command : -command);
    writePwm(motor, magnitude);
}

}  // namespace

namespace TB6612 {

void init()
{
    coast();
}

void setRight(std::int16_t command)
{
    setMotor(Motor::RightB, command);
}

void setLeft(std::int16_t command)
{
    setMotor(Motor::LeftA, command);
}

void setSpeeds(std::int16_t leftCommand, std::int16_t rightCommand)
{
    setLeft(leftCommand);
    setRight(rightCommand);
}

void coast()
{
    coastMotor(Motor::LeftA);
    coastMotor(Motor::RightB);
}

void brake()
{
    // Disable drive before changing all four bridge inputs.
    writePwm(Motor::LeftA, 0U);
    writePwm(Motor::RightB, 0U);
    DL_GPIO_setPins(TB6612_PORT,
                    TB6612_AIN1_PIN | TB6612_AIN2_PIN |
                        TB6612_BIN1_PIN | TB6612_BIN2_PIN);
    writePwm(Motor::LeftA, kMaxCommand);
    writePwm(Motor::RightB, kMaxCommand);
}

}  // namespace TB6612
