#include "car_app.hpp"

#include "encoder.hpp"
#include "k230_protocol.hpp"
#include "k230_uart.hpp"
#include "pid_dashboard.hpp"
#include "speed_controller.hpp"
#include "speed_test.hpp"
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
        if (K230Protocol::consume(data, position)) {
            PidDashboard::setBallPosition(position);
        }
        ++count;
    }
}

}  // namespace

void init()
{
    TB6612::init();
    PidDashboard::init();
    K230Uart::init();
    K230Protocol::reset();

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
    serviceK230Link();

    const Encoder::Sample sample = Encoder::latest();
    SpeedTest::update(sample.sequence);
    PidDashboard::update(sample.sequence);
}

}  // namespace CarApp
