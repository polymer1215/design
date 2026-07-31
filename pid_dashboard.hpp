#ifndef PID_DASHBOARD_HPP
#define PID_DASHBOARD_HPP

#include <cstdint>

#include "k230_protocol.hpp"

namespace PidDashboard {

enum class View {
    K230Monitor,
    SpeedControl,
};

void init();
void setView(View view);
View currentView();
void setBallPosition(const K230Protocol::BallPosition &position);
void update(std::uint32_t sampleSequence);

}  // namespace PidDashboard

#endif
