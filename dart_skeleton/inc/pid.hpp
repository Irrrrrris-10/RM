#pragma once
#include <limits>

namespace dart {
// Gains act on physical units; dt is seconds. Integral stores its output contribution.
class Pid {
 public:
  struct Config {
    float kp = 0, ki = 0, kd = 0;
    float integral_limit = 0;
    float output_limit = 0;
    float dt = 0.002f;
  };
  constexpr Pid() = default;
  constexpr explicit Pid(Config config) { configure(config); }
  constexpr bool configure(Config config) {
    reset();
    valid_ = finite(config.kp) && finite(config.ki) && finite(config.kd) &&
      finite(config.integral_limit) && finite(config.output_limit) && finite(config.dt) &&
      config.kp >= 0 && config.ki >= 0 && config.kd >= 0 &&
      config.integral_limit >= 0 && config.output_limit >= 0 && config.dt > 0;
    config_ = valid_ ? config : Config{};
    return valid_;
  }
  constexpr void reset() {
    integral_ = 0; output_ = 0; previous_measurement_ = 0; primed_ = false;
  }
  constexpr float update(float target, float measurement, bool enabled = true) {
    if (!enabled || !valid_ || !finite(target) || !finite(measurement)) { reset(); return 0; }
    // Use wider intermediates to avoid float overflow before checking the result.
    const double error = double(target)-measurement;
    const double p = config_.kp*error;
    const double d = primed_ ? -double(config_.kd)*(double(measurement)-previous_measurement_)/config_.dt : 0;
    const double step = double(config_.ki)*error*config_.dt;
    const double candidate = clamp(double(integral_)+step, config_.integral_limit);
    const double sum = p+candidate+d;
    if (!finite(p) || !finite(d) || !finite(step) || !finite(sum)) { reset(); return 0; }
    // Reject integration that drives saturation further; permit unwinding.
    if (!((sum > config_.output_limit && step > 0) ||
          (sum < -config_.output_limit && step < 0))) integral_ = float(candidate);
    output_ = float(clamp(p+integral_+d, config_.output_limit));
    previous_measurement_ = measurement; primed_ = true;
    return output_;
  }
  constexpr bool valid() const { return valid_; }
  constexpr float output() const { return output_; }
  constexpr float integral() const { return integral_; }
 private:
  static constexpr bool finite(double value) {
    return value == value && value <= std::numeric_limits<double>::max() &&
      value >= -std::numeric_limits<double>::max();
  }
  static constexpr double clamp(double value, double limit) {
    return value > limit ? limit : value < -limit ? -limit : value;
  }
  Config config_{};
  bool valid_ = false, primed_ = false;
  float integral_ = 0, output_ = 0, previous_measurement_ = 0;
};
}
