#ifndef CAR_APP_HPP
#define CAR_APP_HPP

namespace CarApp {

enum class AppMode {
    K230StepperDebug,
    K230CapturedTarget,
    K230TwoStage,
    LineTracking,
};

void init();
void runOnce();
void selectMode(AppMode mode);
AppMode currentMode();

}  // namespace CarApp

#endif
