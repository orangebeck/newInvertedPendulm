#include "pend_ctrl.h"
#include <math.h>
#include <float.h>
#include <stddef.h>

#ifndef CLAMP
#define CLAMP(x, lo, hi) ((x) < (lo) ? (lo) : ((x) > (hi) ? (hi) : (x)))
#endif

#define PI 3.14159265358979323846
typedef struct {
    double x1, x2;
    double y1, y2;
} BiquadState;

// 设计 2 阶巴特沃斯低通（RBJ 配方，Q=1/sqrt(2)） 
// 输入: fc(Hz), fs(Hz) 
// 输出: 归一化后的系数 (a0==1)，即 b0,b1,b2,a1,a2 
// 返回: 0 成功；非 0 表示参数非法
int design_butterworth_lowpass_biquad(double fc, double fs, double *b0,
                                      double *b1, double *b2, double *a1, double *a2)
{
    if (fc <= 0.0 || fs <= 0.0 || fc >= fs * 0.5)
    {
        return -1; // bad params: 需 0 < fc < fs/2
    }
    // 与 Python 一致：w0 = 2*pi*(fc/fs)
    double w0 = 2.0 * PI * (fc / fs);
    double cosw0 = cos(w0);
    double sinw0 = sin(w0);
    // Q = 1/sqrt(2)（Butterworth 二阶）
    double Q = 1.0 / sqrt(2.0);
    double alpha = sinw0 / (2.0 * Q);
    // 未归一化系数
    double _b0 = (1.0 - cosw0) * 0.5;
    double _b1 = (1.0 - cosw0);
    double _b2 = (1.0 - cosw0) * 0.5;
    double _a0 = 1.0 + alpha;
    double _a1 = -2.0 * cosw0;
    double _a2 = 1.0 - alpha;
    // 归一化到 a0 == 1
    double inv_a0 = 1.0 / _a0;
    *b0 = _b0 * inv_a0;
    *b1 = _b1 * inv_a0;
    *b2 = _b2 * inv_a0;
    *a1 = _a1 * inv_a0;
    *a2 = _a2 * inv_a0;

    return 0;
}

// （可选）单节初始化
static inline void biquad_init(BiquadState* st) {
    st->x1 = st->x2 = st->y1 = st->y2 = 0.0;
}

// ================= 新增：两级串联的实时版 =================

// 二级串联的状态（等价于你原来的“两次前向”）
typedef struct {
    BiquadState s1;  // 第1级
    BiquadState s2;  // 第2级
} BiquadCascadeState;

// 初始化两级串联状态
static inline void biquad_cascade_init(BiquadCascadeState* cs) {
    biquad_init(&cs->s1);
    biquad_init(&cs->s2);
}

static inline double biquad_step(BiquadState* st, double x,
                                 double b0, double b1, double b2,
                                 double a1, double a2)
{
    // 计算当前输出
    double y = b0 * x
             + b1 * st->x1
             + b2 * st->x2
             - a1 * st->y1
             - a2 * st->y2;

    // 更新状态
    st->x2 = st->x1;
    st->x1 = x;
    st->y2 = st->y1;
    st->y1 = y;

    return y;
}

// 实时一步：相当于“先过一级，再过一级”，无 n、无 malloc
static inline double biquad_cascade_step(BiquadCascadeState* cs, double x,
                                         double b0, double b1, double b2,
                                         double a1, double a2)
{
    double y1 = biquad_step(&cs->s1, x,  b0,b1,b2, a1,a2);
    double y2 = biquad_step(&cs->s2, y1, b0,b1,b2, a1,a2);
    return y2;
}

// 双二阶巴特沃斯滤波器结构体
typedef struct {
    double x1, x2, y1, y2;     // 滤波器状态变量
    double a0, a1, a2, b1, b2; // 滤波器系数
} ZeroPhaseFilter;

// 计算巴特沃斯滤波器的系数
void butterworth_coeff(double cutoff_freq, double sample_rate, ZeroPhaseFilter* filter) {
    double W0 = 2 * PI * cutoff_freq / sample_rate;  // 截止频率的归一化
    double cosW0 = cos(W0);
    double sinW0 = sin(W0);
    double alpha = sinW0 / 2.0;

    // 根据二阶低通巴特沃斯滤波器设计计算系数
    double b0 = (1 - cosW0) / 2.0;
    double b1 = 1 - cosW0;
    double b2 = (1 - cosW0) / 2.0;
    double a0 = 1 + alpha;
    double a1 = -2 * cosW0;
    double a2 = 1 - alpha;

    // 保存计算得到的系数
    filter->a0 = b0 / a0;
    filter->a1 = b1 / a0;
    filter->a2 = b2 / a0;
    filter->b1 = a1 / a0;
    filter->b2 = a2 / a0;

    // 初始化滤波器的状态为零
    filter->x1 = 0.0;
    filter->x2 = 0.0;
    filter->y1 = 0.0;
    filter->y2 = 0.0;
}

// 实现双二阶巴特沃斯滤波器（前向+反向滤波器）
double zero_phase_filter(ZeroPhaseFilter* filter, double input) {
    // 前向滤波
    double output = filter->a0 * input + filter->a1 * filter->x1 + filter->a2 * filter->x2
                    - filter->b1 * filter->y1 - filter->b2 * filter->y2;

    // 更新状态变量
    filter->x2 = filter->x1;
    filter->x1 = input;
    filter->y2 = filter->y1;
    filter->y1 = output;

    return output;
}

// 反向滤波（为消除相位延迟，进行反向滤波）
double reverse_zero_phase_filter(ZeroPhaseFilter* filter, double input) {
    // 进行反向滤波（前向滤波后再反向滤波）
    return zero_phase_filter(filter, input);
}

ZeroPhaseFilter filter;
const double fs = 20.0;
const double fc = 1.0;
double b0,b1,b2,a1,a2;
BiquadCascadeState lp2;   // 静态或全局


static inline double lpf_step(double x, double state, double dt, double T)
{
    if (T <= 0.0) return x;
    const double a = dt / (T + dt);
    return state + a * (x - state);
}

/* ----- 默认参数（基于你的条件：20 Hz、Tn≈8s、G=3600mm/rad、α∈[0,0.3mrad]） ----- */
void Ctrl_GetDefaultParams(double dt, CtrlParams* p)
{
    p->alpha0_rad     = 0.15;  // 0.15 mrad
    p->G_mm_per_rad   = 3.80;

    /* 起步增益：稳妥为主（可现场小步调） */
    p->Kp   = 6e-1;   // mrad/mm
    p->Kd   = 3.0;   // mrad/(mm/s)
    p->Ki   = 2e-2;   // mrad/(mm*s)
    p->beta = 0.4;

    /* 滤波：20Hz 下建议慢一些，防激振 */
    p->tau_d = 0.10;  // s
    p->Tsp1  = 1.0;   // s
    p->Tsp2  = 1.0;   // s

    /* 物理限制与抗饱和 */
    p->alpha_min  = 0.0;        // 若允许负方向，改为 -0.3e-3
    p->alpha_max  = 0.3;     // 0.3 mrad
    p->slew_rate  = 0.06;     // mrad/s（可设 0 关闭）
    p->Kaw        = 0.2;        // ~ (5~10)*Ki 比例量级

    /* 捕获→跟踪（按需要用；否则不触发也没关系） */
    p->th_abs_mm   = 0.10;      // mm
    p->th_dabs_mms = 0.20;      // mm/s
    p->dwell_s     = 0.50;      // s

    // 设置滤波器的参数
    double cutoff_freq = 2.0;      // 截止频率 0.5 Hz
    double sample_rate = 20.0;     // 采样频率 10 Hz

    butterworth_coeff(cutoff_freq, sample_rate, &filter);

    if (design_butterworth_lowpass_biquad(fc, fs, &b0,&b1,&b2,&a1,&a2) != 0) {
        printf("bad params\n");
        return 1;
    }
    biquad_cascade_init(&lp2);

    (void)dt; // 当前实现里 tau/滤波常数已直接设定
}

void Ctrl_Init(CtrlState* s)
{
    s->sp1 = s->sp2 = 0.0;
    s->y_prev = 0.0;
    s->dy_f = 0.0;
    s->I_state = 0.0;
    s->alpha_prev = 0.0;
    s->mode = CTRL_TRACK;   // 默认上电先捕获/抑振
    s->dwell_acc = 0.0;
    s->last_P = s->last_D = s->last_I = s->last_FF = s->last_u_unsat = 0.0;
    s->initialized = false;


}

double Controller_Step(double before_filter_mm,
                       const DeviceInfo* info,
                       const CtrlParams* p,
                       CtrlState* s,
                       double XMT_OFFSET_rad,
                       double positive_nagtive_mode)
{
    const double dt = info->dt > 0.0 ? info->dt : 1e-3;

    /* 第一次调用：把设定值滤波器与状态初始化到当前值，避免瞬冲 */
    if (!s->initialized) {
        s->sp1 = s->sp2 = info->target;
        s->y_prev = before_filter_mm;
        s->dy_f = 0.0;
        s->alpha_prev = p->alpha0_rad; // 从基准角开始
        s->initialized = true;
    }

    /* 1) 以 foundation_zero 为“基准位置”做增量（mm） */
    const double y_mm   = before_filter_mm - info->foundation_zero; // 测量相对位移
    const double r_raw  = info->target      - info->foundation_zero; // 设定相对位移

    /* 2) 设定值两级LPF（避免激振）：对绝对目标做滤波，再减去基准得到增量 */
    s->sp1 = lpf_step(info->target, s->sp1, dt, p->Tsp1);
    s->sp2 = lpf_step(s->sp1,       s->sp2, dt, p->Tsp2);
    const double r_mm = s->sp2 - info->foundation_zero;

    /* 4) 误差构造：P 用加权设定值，I 用完整误差，D 只看测量速度 */
    const double eP = positive_nagtive_mode*(p->beta *( r_mm - y_mm));  // mm
    const double eI = positive_nagtive_mode*(r_mm - y_mm);            // mm

    /* 3) D-on-measurement：对测量做速度估计（低通微分） */
    if((eP < 0.0005 && eP > -0.0005) && (s->dy_f < 0.0015 && s->dy_f > -0.0015))  //现在波动ep和测量之存在很大偏差
    {
        const double dy_raw = (before_filter_mm - s->y_prev) / dt;
        s->y_prev = before_filter_mm;
        s->dy_f = (p->tau_d > 0.0) ? lpf_step(dy_raw, s->dy_f, dt, p->tau_d) : dy_raw; 
        //在10um波动范围内使用长周期滤波平滑D的波动,直接将p->tau_d设置为10.0
        // s->dy_f = (p->tau_d > 0.0) ? lpf_step(dy_raw, s->dy_f, dt, 10.0) : dy_raw; 
    }else
    {
        const double dy_raw = (before_filter_mm - s->y_prev) / dt;
        s->y_prev = before_filter_mm;
        s->dy_f = (p->tau_d > 0.0) ? lpf_step(dy_raw, s->dy_f, dt, p->tau_d) : dy_raw;
    }

    /* 5) 前馈：α_ff = α0 + (1/G) * r_mm */
    const double invG = 1.0 / p->G_mm_per_rad;
    const double alpha_ff = p->alpha0_rad + positive_nagtive_mode * invG * r_mm;

    double D = 0.0;
    /* 6) PD + 慢 I（单位直接输出 rad） */
    if((eP < 0.0005 && eP > -0.0005) && (s->dy_f < 0.0015 && s->dy_f > -0.0015))
    {
        // D = 0.0;   // rad（抑制速度）
        //如果本if判断里面不能将波动进行抑制就将 114行注释，116行解注释，同时96行注释98行接触注释
        D = - positive_nagtive_mode * p->Kd * s->dy_f;   // rad（抑制速度）
    }else
    {
        D = - positive_nagtive_mode * p->Kd * s->dy_f;   // rad（抑制速度）
    }
    const double P = p->Kp * eP;          // rad

    /* 积分器（只在 TRACK 模式下积累） */
    double I = 0.0;
    if (s->mode == CTRL_TRACK ) {
        s->I_state += p->Ki * eI * dt;   // rad
        I = s->I_state;
    }

    /* 7) 未饱和输出 */
    // double filtered = zero_phase_filter(&filter, D);

    //     // 反向滤波
    // D = reverse_zero_phase_filter(&filter, filtered);
    if((eP < 0.001 && eP > -0.001) )
    {
        biquad_cascade_step(&lp2, D, b0,b1,b2, a1,a2);
    }else
    {
        biquad_cascade_step(&lp2, D, b0,b1,b2, a1,a2);
    }
    
    const double u_unsat = alpha_ff + P + D + I ;
    // printf("dy_f = %f, b*rmm = %f, ymm = %f, eP = %f,D = %f , ", s->dy_f, p->beta * r_mm ,y_mm, eP, D);
    /* 8) 限幅 + 斜率限幅 */
    double alpha_cmd = CLAMP(u_unsat, p->alpha_min, p->alpha_max);


    if (p->slew_rate > 0.0) {
        const double dmax = p->slew_rate * dt;
        const double lo = s->alpha_prev - dmax;
        const double hi = s->alpha_prev + dmax;
        alpha_cmd = CLAMP(alpha_cmd, lo, hi);
    }


    /* 9) 抗积分饱和（back-calculation） */
    if (s->mode == CTRL_TRACK && p->Ki > 0.0 && p->Kaw > 0.0) {
        const double aw = p->Kaw * (alpha_cmd - u_unsat); // rad
        s->I_state += aw * dt;
        I = s->I_state;
    }

    /* 10) 捕获→跟踪条件（若不需要，可把阈值设很宽或直接将 mode=TRACK） */
    const bool calm = (fabs(eI) < p->th_abs_mm) && (fabs(s->dy_f) < p->th_dabs_mms);
    s->dwell_acc = calm ? (s->dwell_acc + dt) : 0.0;
    if (s->mode == CTRL_CAPTURE && s->dwell_acc >= p->dwell_s) {
        s->mode = CTRL_TRACK;
        /* 无扰切换：I = alpha_cmd - (P + D + alpha_ff + XMT_OFFSET) */
        s->I_state = alpha_cmd - (P + D + alpha_ff );
    }

    /* 记录与返回 */
    s->last_P = P; s->last_D = D; s->last_I = I; s->last_FF = alpha_ff; s->last_u_unsat = u_unsat;
    s->alpha_prev = alpha_cmd;
    return alpha_cmd;
}