#ifndef CAR_APP_HPP
#define CAR_APP_HPP

namespace CarApp {

enum class AppMode {
    K230StepperDebug,
    K230FourSecondFixedTarget,
    K230CapturedTarget,
    K230TwoStage,
    LineTracking,
    WheelPwmTest,
};

void init();
void runOnce();
void selectMode(AppMode mode);
AppMode currentMode();

}  // namespace CarApp

#endif
