#ifndef TB6612_HPP
#define TB6612_HPP

#include <cstdint>

namespace TB6612 {

constexpr std::int16_t kMaxCommand = 1000;

// Call after SYSCFG_DL_init(). Both motors remain stopped.
void init();

// Signed commands are clamped to [-1000, 1000].
// Positive means the vehicle's forward direction.
void setRight(std::int16_t command);
void setLeft(std::int16_t command);
void setSpeeds(std::int16_t leftCommand, std::int16_t rightCommand);

// Coast disables both bridges. Brake actively shorts both motor terminals.
void coast();
void brake();

}  // namespace TB6612

#endif  // TB6612_HPP
