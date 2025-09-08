#ifndef PEND_CTRL_H
#define PEND_CTRL_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 你的 info 结构（可与工程里的保持一致） */
typedef struct {
    double foundation_zero;  // mm：平台 0.15 mrad 时的摆位置（基准点）
    double target;           // mm：期望摆位置（绝对值，非增量）
    double dt;               // s：控制周期（你给的是 20 Hz → 0.05s）
    double hangLenth;        // 可忽略（留做兼容，不参与控制换算）
    double amplify;          // 可忽略（留做兼容，不参与控制换算）
} DeviceInfo;

/* 控制器参数（可根据需要微调） */
typedef struct {
    /* 映射与前馈 */
    double alpha0_rad;        // 基准角：0.15 mrad → 0.00015 rad
    double G_mm_per_rad;      // 3600 mm/rad (0.1mrad → 0.36mm)
    /* 增益：单位基于输入=mm，输出=rad */
    double Kp;                // rad/mm
    double Kd;                // rad/(mm/s)
    double Ki;                // rad/(mm*s)
    double beta;              // 设定值加权（P用）
    /* 滤波 */
    double tau_d;             // s：D 低通时间常数（对测量微分）
    double Tsp1;              // s：设定值预滤波一级
    double Tsp2;              // s：设定值预滤波二级
    /* 执行器与抗饱和 */
    double alpha_min;         // rad：最小平台角（如 0 或 -0.0003）
    double alpha_max;         // rad：最大平台角（如 +0.0003）
    double slew_rate;         // rad/s：斜率限幅（<=0 关闭）
    double Kaw;               // 抗积分饱和回灌
    /* 捕获→跟踪（如果不需要，可不触发） */
    double th_abs_mm;         // mm：进入TRACK的角度误差阈值
    double th_dabs_mms;       // mm/s：进入TRACK的速度阈值
    double dwell_s;           // s：阈值满足的保持时间
} CtrlParams;

/* 控制器状态 */
typedef enum { CTRL_CAPTURE = 0, CTRL_TRACK = 1 } CtrlMode;

typedef struct {
    /* 滤波状态 */
    double sp1, sp2;          // 设定值两级LPF
    double y_prev;            // 上周期测量（mm）
    double dy_f;              // 低通后的速度估计（mm/s）
    /* 积分与输出 */
    double I_state;           // mm→rad 的积分状态（单位=rad）
    double alpha_prev;        // 上周期输出（rad）
    /* 模式与计时 */
    CtrlMode mode;
    double dwell_acc;         // 满足阈值的累计时间（s）
    /* 监视量（可选） */
    double last_P, last_D, last_I, last_FF, last_u_unsat;
    bool   initialized;
} CtrlState;

/* 初始化参数的一个“即插即用”默认值（基于你的装置） */
void Ctrl_GetDefaultParams(double dt, CtrlParams* p);

/* 初始化状态 */
void Ctrl_Init(CtrlState* s);

/* 单步更新：
   输入：
     before_filter_mm : 传感器原始位置（mm）
     info              : 包含 foundation_zero / target / dt
     p                 : 控制器参数
     XMT_OFFSET_rad    : 执行机构安装零位偏置（rad），若无则传 0
     positive_nagtive_mode : 正向增益，方向增益  
   输出：
     平台角 α_cmd（rad），已限幅/斜率限幅
*/
double Controller_Step(double before_filter_mm,
                       const DeviceInfo* info,
                       const CtrlParams* p,
                       CtrlState* s,
                       double XMT_OFFSET_rad,
                       double positive_nagtive_mode);

#ifdef __cplusplus
}
#endif

#endif /* PEND_CTRL_H */