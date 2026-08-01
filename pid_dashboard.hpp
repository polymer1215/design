#ifndef PID_DASHBOARD_HPP
#define PID_DASHBOARD_HPP

#include <cstdint>

#include "k230_protocol.hpp"

namespace PidDashboard {

enum class View {
    K230Monitor,
    ModeSelection,
    LineTrackingRuntime,
    SpeedControl,
};

void init();
void setView(View view);
View currentView();
void setBallPosition(const K230Protocol::BallPosition &position);
void setModeSelection(std::uint8_t b21PressCount);
void startLineTrackingRuntime(std::uint32_t sampleSequence);
void stopLineTrackingRuntime(std::uint32_t sampleSequence);
void update(std::uint32_t sampleSequence);

}  // namespace PidDashboard

#endif
