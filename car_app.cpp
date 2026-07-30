#include "car_app.hpp"

#include "encoder.hpp"
#include "gray_sensor.hpp"
#include "k230_protocol.hpp"
#include "k230_uart.hpp"
#include "line_tracking.hpp"
#include "pid_dashboard.hpp"
#include "speed_controller.hpp"
#include "tb6612.hpp"

namespace CarApp {

namespace {

void serviceK230Link()
{
    constexpr std::size_t kMaximumBytesPerRun = 64U;
    std::size_t count = 0U;
    std::uint8_t data;
    K230Protocol::BallPosition position;

    while (count < kMaximumBytesPerRun && K230Uart::read(data)) {
        K230Uart::write(data);
        K230Protocol::consume(data, position);
        ++count;
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

    TB6612::init();
    PidDashboard::init();
    GraySensor::init();

    SpeedControl::init();
    SpeedControl::setTunings(
        SpeedControl::kDefaultKp,
        SpeedControl::kDefaultKi,
        SpeedControl::kDefaultKd);
    LineTracking::init();
    SpeedControl::pidEnabled = true;
    Encoder::init();
}

void runOnce()
{
    serviceK230Link();

    const Encoder::Sample sample = Encoder::latest();
    GraySensor::update(sample.sequence);
    LineTracking::update(sample.sequence);
    PidDashboard::update(sample.sequence);
}

}  // namespace CarApp
