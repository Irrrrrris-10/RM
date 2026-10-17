#ifndef SINEREF_H
#define SINEREF_H

// 正弦参考发生器
//
//   value(t)      = offset + amp * sin(2π*freq*t + phase)
//   derivative(t) =          amp * 2π*freq * cos(2π*freq*t + phase)
//
// 每个控制周期调用一次 update() 推进时间并返回当前参考值。
// derivative() 给出参考的导数, 可用作速度前馈。
class SineRef {
public:
    SineRef(float amp = 1.0f, float freq = 1.0f, float offset = 0.0f,
            float phase = 0.0f, float dt = 0.001f);

    void setParams(float amp, float freq, float offset, float phase);
    void setDt(float dt);
    void reset();            // 时间清零, 从 t=0 重新开始

    float update();          // 推进一个周期, 返回当前参考值
    float value() const;     // 当前参考值(不推进时间)
    float derivative() const;// 当前参考的导数

private:
    float amp_;
    float freq_;
    float offset_;
    float phase_;
    float dt_;
    float t_;
    float value_;
    float deriv_;
};

#endif // SINEREF_H
