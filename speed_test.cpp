#include "speed_test.hpp"

#include "encoder.hpp"
#include "speed_controller.hpp"

namespace {

constexpr float kStartRpm = 50.0F;
constexpr float kStepRpm = 50.0F;
constexpr float kMaximumRpm = 500.0F;
constexpr std::uint32_t kStepTicks = 2U * Encoder::kSampleRateHz;

float g_targetRpm = kStartRpm;
std::uint32_t g_nextStep = kStepTicks;

}  // namespace

namespace SpeedTest {

void init()
{
    g_targetRpm = kStartRpm;
    g_nextStep = kStepTicks;
    SpeedControl::setTargetRpm(g_targetRpm, g_targetRpm);
}

void update(std::uint32_t sampleSequence)
{
    while (g_targetRpm < kMaximumRpm &&
        sampleSequence >= g_nextStep) {
        g_targetRpm += kStepRpm;
        SpeedControl::setTargetRpm(g_targetRpm, g_targetRpm);
        g_nextStep += kStepTicks;
    }
}

}  // namespace SpeedTest
