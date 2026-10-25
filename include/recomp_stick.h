#pragma once

#include <algorithm>
#include <array>
#include <cmath>

namespace recomp {
inline void apply_stick_deadzone(float& x, float& y, float deadzone) {
    deadzone = std::clamp(deadzone, 0.0f, 1.0f);
    auto adjust = [deadzone](float value) {
        if (deadzone >= 1.0f || std::fabs(value) <= deadzone) return 0.0f;
        return std::copysign((std::fabs(value) - deadzone) / (1.0f - deadzone), value);
    };
    x = adjust(x);
    y = adjust(y);
}

// Ranges are measured from the resting center, independently for each direction.
struct StickCalibration {
    bool enabled = false;
    float center_x = 0.0f;
    float center_y = 0.0f;
    float left = 1.0f;
    float right = 1.0f;
    float up = 1.0f;
    float down = 1.0f;
    // Fraction of the calibrated range, measured from resting noise and snapback.
    float deadzone = 0.0f;

    bool valid() const {
        if (!std::isfinite(center_x) || !std::isfinite(center_y)
            || std::fabs(center_x) > 0.25f || std::fabs(center_y) > 0.25f) return false;
        for (float range : {left, right, up, down}) {
            if (!std::isfinite(range) || range < 0.2f || range > 1.25f) return false;
        }
        return std::isfinite(deadzone) && deadzone >= 0.0f && deadzone <= 0.5f;
    }

    void apply(float& x, float& y, float minimum_deadzone = 0.0f) const {
        if (!enabled) {
            apply_stick_deadzone(x, y, minimum_deadzone);
            return;
        }
        x -= center_x;
        y -= center_y;
        // Ignore small fluctuations around the measured resting center.
        x = std::fabs(x) <= 0.01f ? 0.0f : x / (x < 0.0f ? left : right);
        y = std::fabs(y) <= 0.01f ? 0.0f : y / (y < 0.0f ? down : up);
        float largest = std::max({1.0f, std::fabs(x), std::fabs(y)});
        x /= largest;
        y /= largest;
        if (deadzone == 0.0f) {
            apply_stick_deadzone(x, y, minimum_deadzone);
        } else {
            // A radial deadzone preserves direction and full-speed diagonals.
            float threshold = std::clamp(std::max(deadzone, minimum_deadzone), 0.0f, 1.0f);
            float magnitude = std::hypot(x, y);
            if (threshold >= 1.0f || magnitude <= threshold) {
                x = y = 0.0f;
            } else if (magnitude < 1.0f) {
                float scale = (magnitude - threshold) / ((1.0f - threshold) * magnitude);
                x *= scale;
                y *= scale;
            }
        }
    }
};

// UI-owned capture state. Sample raw controller events and once per frame while idle.
class StickCalibrationSession {
public:
    enum class Stage { Idle, Center, Range, Snapback };

    void begin() { *this = {}; stage_ = Stage::Center; }
    void cancel() { stage_ = Stage::Idle; }
    Stage stage() const { return stage_; }
    bool active() const { return stage_ != Stage::Idle; }
    int flicks_recorded() const { return flicks_; }
    const char* flick_direction() const {
        constexpr std::array names{"right", "left", "up", "down"};
        return names[std::min(flicks_ / 2, 3)];
    }

    int directions_recorded() const {
        return static_cast<int>(std::count_if(directions_.begin(), directions_.end(), [](int count) { return count >= 3; }));
    }

    bool ready() const {
        return stage_ == Stage::Snapback && flicks_ == 8 && ranges_.valid()
            && centered_ && latest_time_ - centered_since_ >= 0.2;
    }

    void sample(float x, float y, double now) {
        if (!active() || !std::isfinite(x) || !std::isfinite(y)
            || !std::isfinite(now) || std::fabs(x) > 1.0f || std::fabs(y) > 1.0f) return;
        // Event delivery can lag a frame heartbeat; keep timers monotonic while
        // still observing every rebound sample.
        now = std::max(now, latest_time_);
        latest_time_ = now;
        if (stage_ == Stage::Center) {
            if (std::fabs(x) > 0.25f || std::fabs(y) > 0.25f) {
                center_samples_ = 0;
                return;
            }
            if (center_samples_ == 0 || x < min_x_ - 0.03f || x > max_x_ + 0.03f
                || y < min_y_ - 0.03f || y > max_y_ + 0.03f) {
                reset_center(x, y, now);
                return;
            }
            min_x_ = std::min(min_x_, x); max_x_ = std::max(max_x_, x);
            min_y_ = std::min(min_y_, y); max_y_ = std::max(max_y_, y);
            if (max_x_ - min_x_ > 0.03f || max_y_ - min_y_ > 0.03f) {
                reset_center(x, y, now);
                return;
            }
            sum_x_ += x; sum_y_ += y; ++center_samples_;
            if (now - center_started_ >= 1.0 && center_samples_ >= 10) {
                ranges_.center_x = static_cast<float>(sum_x_ / center_samples_);
                ranges_.center_y = static_cast<float>(sum_y_ / center_samples_);
                ranges_.left = ranges_.right = ranges_.up = ranges_.down = 0.0f;
                stage_ = Stage::Range;
            }
            return;
        }

        if (stage_ == Stage::Snapback) {
            sample_snapback(x, y, now);
            return;
        }

        float dx = x - ranges_.center_x, dy = y - ranges_.center_y;
        ranges_.left = std::max(ranges_.left, -dx);
        ranges_.right = std::max(ranges_.right, dx);
        ranges_.up = std::max(ranges_.up, dy);
        ranges_.down = std::max(ranges_.down, -dy);
        if (std::hypot(dx, dy) >= 0.2f) {
            constexpr float sector_angle = 0.7853981633974483f;
            int sector = (static_cast<int>(std::lround(std::atan2(dy, dx) / sector_angle)) + 8) % 8;
            directions_[sector] = std::min(directions_[sector] + 1, 3);
        }
        if (directions_recorded() == 8 && ranges_.valid()
            && std::fabs(dx) < 0.1f && std::fabs(dy) < 0.1f) {
            // Measure rebound in exactly the same units used during gameplay.
            ranges_.enabled = true;
            for (float* range : {&ranges_.left, &ranges_.right, &ranges_.up, &ranges_.down}) {
                *range = std::max(0.2f, *range * 0.95f);
            }
            float noise_x = std::max(std::fabs(min_x_ - ranges_.center_x) / ranges_.left,
                std::fabs(max_x_ - ranges_.center_x) / ranges_.right);
            float noise_y = std::max(std::fabs(min_y_ - ranges_.center_y) / ranges_.down,
                std::fabs(max_y_ - ranges_.center_y) / ranges_.up);
            noise_peak_ = std::hypot(noise_x, noise_y);
            stage_ = Stage::Snapback;
        }
    }

    bool finish(StickCalibration& out) {
        if (!ready()) return false;
        out = ranges_;
        // Add five percentage points for harder releases than those captured,
        // with a 10% floor, and round up to avoid shaving off the safety margin.
        out.deadzone = std::clamp(std::ceil((noise_peak_ + 0.05f) * 100.0f) / 100.0f, 0.10f, 0.5f);
        cancel();
        return true;
    }

private:
    Stage stage_ = Stage::Idle;
    StickCalibration ranges_;
    std::array<int, 8> directions_{};
    double center_started_ = 0.0, sum_x_ = 0.0, sum_y_ = 0.0;
    int center_samples_ = 0;
    float min_x_ = 0.0f, max_x_ = 0.0f, min_y_ = 0.0f, max_y_ = 0.0f;
    double latest_time_ = 0.0, centered_since_ = 0.0, held_since_ = -1.0, released_at_ = -1.0;
    bool centered_ = false, waiting_for_center_ = true, armed_ = false;
    int flicks_ = 0;
    float noise_peak_ = 0.0f, release_peak_ = 0.0f;

    void sample_snapback(float x, float y, double now) {
        ranges_.apply(x, y);
        bool near_center = std::hypot(x, y) < 0.1f;
        if (near_center && !centered_) centered_since_ = now;
        centered_ = near_center;
        if (flicks_ == 8) return;
        if (waiting_for_center_) {
            if (centered_ && now - centered_since_ >= 0.2) waiting_for_center_ = false;
            return;
        }

        int direction = flicks_ / 2;
        float along = direction < 2 ? x : y;
        float across = direction < 2 ? y : x;
        if (direction == 1 || direction == 3) along = -along;
        if (!armed_) {
            if (along >= 0.65f && std::fabs(across) < 0.25f) {
                if (held_since_ < 0.0) held_since_ = now;
                armed_ = now - held_since_ >= 0.05;
            } else {
                held_since_ = -1.0;
            }
            return;
        }
        if (released_at_ < 0.0) {
            if (along > 0.25f) return;
            released_at_ = now;
        }

        // The initial return toward center is intentional. Only opposite travel
        // and sideways movement near/past center contribute to the rebound peak.
        release_peak_ = std::max(release_peak_, -along);
        if (along <= 0.05f) release_peak_ = std::max(release_peak_, std::hypot(along, across));
        bool invalid_release = std::hypot(x, y) > 0.45f
            || now - released_at_ > 1.0;
        bool settled = now - released_at_ >= 0.35 && centered_ && now - centered_since_ >= 0.2;
        if (!invalid_release && !settled) return;
        if (!invalid_release) {
            noise_peak_ = std::max(noise_peak_, release_peak_);
            ++flicks_;
        }
        waiting_for_center_ = true;
        armed_ = false;
        held_since_ = released_at_ = -1.0;
        release_peak_ = 0.0f;
    }

    void reset_center(float x, float y, double now) {
        center_started_ = now;
        sum_x_ = x; sum_y_ = y; center_samples_ = 1;
        min_x_ = max_x_ = x; min_y_ = max_y_ = y;
    }
};
}
