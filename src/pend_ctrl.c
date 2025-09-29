#include "pend_ctrl.h"
#include <math.h>
#include <float.h>
#include <stddef.h>

#ifndef CLAMP
#define CLAMP(x, lo, hi) ((x) < (lo) ? (lo) : ((x) > (hi) ? (hi) : (x)))
#endif

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
    const double dy_raw = (before_filter_mm - s->y_prev) / dt;
    s->y_prev = before_filter_mm;
    static double change_filter = 0;
    bool change_filter_calm = (fabs(eI) < 0.01);
    change_filter = change_filter_calm ? (change_filter + dt) : 0.0;

    if(change_filter > 8.0)
    {
        s->dy_f = (p->tau_d > 0.0) ? lpf_step(dy_raw, s->dy_f, dt, 0.0) : dy_raw;
    }else
    {
        s->dy_f = (p->tau_d > 0.0) ? lpf_step(dy_raw, s->dy_f, dt, p->tau_d) : dy_raw;
    }

    
    /* 5) 前馈：α_ff = α0 + (1/G) * r_mm */
    const double invG = 1.0 / p->G_mm_per_rad;
    const double alpha_ff = p->alpha0_rad + positive_nagtive_mode * invG * r_mm;

    /* 6) PD + 慢 I（单位直接输出 rad） */
    double D = 0.0;
    if(change_filter > 8.0)
    {
        //不对dy进行滤波的情况下可能会导致dy过大，所以需要乘一个系数降低D的幅值
        D = - positive_nagtive_mode * p->Kd * s->dy_f * 0.5;   // rad（抑制速度）
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
    const double u_unsat = alpha_ff + P + D + I ;

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