// Standalone regression test: compile with C++20 and the project's include directory.
#include "recomp_stick.h"
#include <cstdlib>
#include <iostream>
#include <limits>

namespace {
int checks = 0;
void check(bool condition, const char* message) {
    if (!condition) { std::cerr << message << '\n'; std::exit(1); }
    ++checks;
}

struct Capture {
    recomp::StickCalibrationSession session;
    double time = 0.0;
    Capture() {
        session.begin();
        for (int i = 0; i < 120; ++i) raw(.04f, -.02f);
        check(session.stage() == recomp::StickCalibrationSession::Stage::Range, "stable center captured");
        for (int i = 0; i < 240; ++i) {
            float angle = i * (6.28318530718f / 120);
            float x = std::cos(angle), y = std::sin(angle);
            raw(.04f + x * (x < 0 ? .6f : .85f), -.02f + y * (y < 0 ? .8f : .7f));
        }
        raw(.04f, -.02f);
        check(session.stage() == recomp::StickCalibrationSession::Stage::Snapback, "range sweep requires snapback capture");
        check(!session.ready(), "range sweep alone cannot be saved");
    }
    void raw(float x, float y, double dt = .01) {
        time += dt;
        session.sample(x, y, time);
    }
    void normalized(float x, float y, double dt = .01) {
        raw(.04f + x * (x < 0 ? .6f : .85f) * .95f,
            -.02f + y * (y < 0 ? .8f : .7f) * .95f, dt);
    }
    void rest(int samples = 50) {
        for (int i = 0; i < samples; ++i) normalized(0, 0);
    }
    void flick(int direction, float rebound, bool settle = true) {
        rest(30);
        float x = direction == 0 ? 1.f : direction == 1 ? -1.f : 0.f;
        float y = direction == 2 ? 1.f : direction == 3 ? -1.f : 0.f;
        for (int i = 0; i < 10; ++i) normalized(x * .9f, y * .9f);
        normalized(x * .15f, y * .15f);
        // A one-millisecond opposite pulse must be measured, even if the next
        // rendered frame would already see a centered stick.
        normalized(-x * rebound, -y * rebound, .001);
        normalized(x * .04f, y * .04f, .001);
        if (settle) rest();
    }
};
}

int main() {
    Capture capture;
    recomp::StickCalibration result;
    check(!capture.session.finish(result), "cannot save before measuring releases");
    capture.flick(1, .2f);
    check(capture.session.flicks_recorded() == 0, "wrong requested direction is not counted");
    capture.flick(0, .65f);
    check(capture.session.flicks_recorded() == 0, "deliberate hard reversal is not mistaken for snapback");
    for (int i = 0; i < 8; ++i) {
        capture.flick(i / 2, i == 3 ? .27f : .12f, i != 7);
        if (i != 7) check(capture.session.flicks_recorded() == i + 1, "each settled release is counted");
    }
    check(!capture.session.ready(), "last release must settle before saving");
    capture.rest();
    check(capture.session.finish(result), "complete calibration saves");
    check(result.enabled && result.valid() && !capture.session.active(), "saved calibration valid and capture ends");
    check(result.deadzone >= .32f && result.deadzone <= .34f, "worst directional rebound plus five percentage points sets deadzone");
    float padded_x = .04f - .31f * .6f * .95f, padded_y = -.02f;
    result.apply(padded_x, padded_y);
    check(padded_x == 0 && padded_y == 0, "safety padding absorbs rebound stronger than the calibration samples");
    for (auto point : {std::array{.04f - .12f * .6f * .95f, -.02f},
                       std::array{.04f + .27f * .85f * .95f, -.02f},
                       std::array{.04f, -.02f - .12f * .8f * .95f},
                       std::array{.04f, -.02f + .12f * .7f * .95f}}) {
        result.apply(point[0], point[1]);
        check(point[0] == 0 && point[1] == 0, "measured snapback produces no game movement");
    }
    float x = -.56f, y = -.02f;
    result.apply(x, y);
    check(x == -1 && y == 0, "intentional reversal still reaches full speed immediately");
    x = .04f + .6f * .85f * .95f; y = -.02f;
    result.apply(x, y);
    check(x > .35f && x < .5f && y == 0, "walking remains proportional outside the deadzone");
    x = .04f + .70710678f * .85f; y = -.02f + .70710678f * .7f;
    result.apply(x, y);
    check(std::hypot(x, y) >= 1.0f && std::fabs(x - y) < .0001f,
        "full-speed diagonal movement and direction survive the measured deadzone");
    x = .04f + .2f * .85f * .95f; y = -.02f + .2f * .7f * .95f;
    result.apply(x, y);
    check(x == 0 && y == 0, "diagonal rebound inside the measured radius is suppressed");
    x = .04f + .6f * .85f * .95f; y = -.02f;
    result.apply(x, y, .4f);
    check(std::fabs(x - 1.f / 3.f) < .0001f, "manual deadzone is a minimum, not a second stacked deadzone");

    Capture smooth;
    for (int i = 0; i < 8; ++i) smooth.flick(i / 2, .01f);
    recomp::StickCalibration smooth_result;
    check(smooth.session.finish(smooth_result) && smooth_result.deadzone == .10f, "smooth controller gets a 10 percent safety deadzone");
    check(result.deadzone > smooth_result.deadzone, "different controllers retain independent measured deadzones");

    Capture canceled;
    canceled.flick(0, .2f);
    canceled.session.cancel();
    check(!canceled.session.finish(result) && result.deadzone >= .32f, "cancel preserves saved settings");

    auto invalid = result;
    invalid.deadzone = std::numeric_limits<float>::quiet_NaN();
    check(!invalid.valid(), "nonfinite deadzone is rejected");
    invalid.deadzone = -.1f;
    check(!invalid.valid(), "negative deadzone is rejected");
    invalid.deadzone = .9f;
    check(!invalid.valid(), "excessive saved deadzone is rejected");
    recomp::StickCalibration legacy{true, 0, 0, 1, 1, 1, 1};
    x = .2f; y = 0;
    legacy.apply(x, y);
    check(x == .2f && legacy.deadzone == 0, "older calibrations preserve their original response");
    legacy.enabled = false;
    x = .2f; y = -.3f;
    legacy.apply(x, y, 1.0f);
    check(x == 0 && y == 0, "100 percent manual deadzone cannot divide by zero");
    std::cout << checks << " stick calibration checks passed.\n";
}
