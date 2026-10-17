#include "SineRef.h"

#include <cmath>

namespace {
constexpr float k2Pi = 6.283185307179586f;
}

SineRef::SineRef(float amp, float freq, float offset, float phase, float dt)
    : amp_(amp), freq_(freq), offset_(offset), phase_(phase), dt_(dt),
      t_(0.0f), value_(0.0f), deriv_(0.0f) {
    reset();
}

void SineRef::setParams(float amp, float freq, float offset, float phase) {
    amp_ = amp; freq_ = freq; offset_ = offset; phase_ = phase;
}

void SineRef::setDt(float dt) {
    dt_ = dt;
}

void SineRef::reset() {
    t_ = 0.0f;
    value_ = offset_ + amp_ * sinf(phase_);
    deriv_ = amp_ * k2Pi * freq_ * cosf(phase_);
}

float SineRef::update() {
    const float w = k2Pi * freq_;
    const float arg = w * t_ + phase_;

    value_ = offset_ + amp_ * sinf(arg);
    deriv_ = amp_ * w * cosf(arg);

    t_ += dt_;
    // 防止长时间运行 t_ 过大导致浮点精度下降/溢出(约每 1000s 回绕一次)
    if (t_ > 1.0e6f) {
        t_ = 0.0f;
    }
    return value_;
}

float SineRef::value() const { return value_; }
float SineRef::derivative() const { return deriv_; }
