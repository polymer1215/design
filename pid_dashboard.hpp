#ifndef PID_DASHBOARD_HPP
#define PID_DASHBOARD_HPP

#include <cstdint>

namespace PidDashboard {

void init();
void pushK230Byte(std::uint8_t data);
void update(std::uint32_t sampleSequence);

}  // namespace PidDashboard

#endif
