#include "car_app.hpp"

#include "encoder.hpp"
#include "k230_uart.hpp"
#include "pid_dashboard.hpp"
#include "speed_controller.hpp"
#include "speed_test.hpp"
#include "tb6612.hpp"

namespace CarApp {

void init()
{
    TB6612::init();
    PidDashboard::init();
    K230Uart::init();

    SpeedControl::init();
    SpeedControl::setTunings(
        SpeedControl::kDefaultKp,
        SpeedControl::kDefaultKi,
        SpeedControl::kDefaultKd);
    SpeedTest::init();
    Encoder::init();
}

void runOnce()
{
    K230Uart::serviceEcho();

    const Encoder::Sample sample = Encoder::latest();
    SpeedTest::update(sample.sequence);
    PidDashboard::update(sample.sequence);
}

}  // namespace CarApp
