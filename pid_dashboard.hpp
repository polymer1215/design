#ifndef PID_DASHBOARD_HPP
#define PID_DASHBOARD_HPP

#include <cstdint>

#include "k230_protocol.hpp"

namespace PidDashboard {

void init();
void setBallPosition(const K230Protocol::BallPosition &position);
void update(std::uint32_t sampleSequence);

}  // namespace PidDashboard

#endif
