#pragma once

#include <algorithm>
#include <array>
#include <cmath>

namespace recomp {
// Ranges are measured from the resting center, independently for each direction.
struct StickCalibration {
    bool enabled = false;
    float center_x = 0.0f;
    float center_y = 0.0f;
    float left = 1.0f;
    float right = 1.0f;
    float up = 1.0f;
    float down = 1.0f;

    bool valid() const {
        if (!std::isfinite(center_x) || !std::isfinite(center_y)
            || std::fabs(center_x) > 0.25f || std::fabs(center_y) > 0.25f) return false;
        for (float range : {left, right, up, down}) {
            if (!std::isfinite(range) || range < 0.2f || range > 1.25f) return false;
        }
        return true;
    }

    void apply(float& x, float& y) const {
        if (!enabled) return;
        x -= center_x;
        y -= center_y;
        // Ignore small fluctuations around the measured resting center.
        x = std::fabs(x) <= 0.01f ? 0.0f : x / (x < 0.0f ? left : right);
        y = std::fabs(y) <= 0.01f ? 0.0f : y / (y < 0.0f ? down : up);
        float largest = std::max({1.0f, std::fabs(x), std::fabs(y)});
        x /= largest;
        y /= largest;
    }
};

// UI-owned capture state. Call sample once per frame, using unadjusted stick input.
class StickCalibrationSession {
public:
    enum class Stage { Idle, Center, Range };

    void begin() { *this = {}; stage_ = Stage::Center; }
    void cancel() { stage_ = Stage::Idle; }
    Stage stage() const { return stage_; }
    bool active() const { return stage_ != Stage::Idle; }

    int directions_recorded() const {
        return static_cast<int>(std::count_if(directions_.begin(), directions_.end(), [](int count) { return count >= 3; }));
    }

    bool ready() const {
        return stage_ == Stage::Range && directions_recorded() == 8
            && ranges_.valid() && ranges_.left >= 0.2f && ranges_.right >= 0.2f
            && ranges_.up >= 0.2f && ranges_.down >= 0.2f
            && std::fabs(latest_x_ - ranges_.center_x) < 0.1f
            && std::fabs(latest_y_ - ranges_.center_y) < 0.1f;
    }

    void sample(float x, float y, double now) {
        if (!active() || !std::isfinite(x) || !std::isfinite(y)
            || std::fabs(x) > 1.0f || std::fabs(y) > 1.0f) return;
        latest_x_ = x;
        latest_y_ = y;
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
    }

    bool finish(StickCalibration& out) {
        if (!ready()) return false;
        out = ranges_;
        out.enabled = true;
        // Reach full speed slightly before the measured edge to allow for jitter.
        out.left = std::max(0.2f, out.left * 0.95f);
        out.right = std::max(0.2f, out.right * 0.95f);
        out.up = std::max(0.2f, out.up * 0.95f);
        out.down = std::max(0.2f, out.down * 0.95f);
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
    float latest_x_ = 0.0f, latest_y_ = 0.0f;

    void reset_center(float x, float y, double now) {
        center_started_ = now;
        sum_x_ = x; sum_y_ = y; center_samples_ = 1;
        min_x_ = max_x_ = x; min_y_ = max_y_ = y;
    }
};
}
