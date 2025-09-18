#include <math.h>
#include <float.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define MAX_LINE 1024

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

int main(int argc, char *argv[])
{
    if (argc < 2) {
        fprintf(stderr, "用法: %s file.csv\n", argv[0]);
        return 1;
    }

    const char *filename = argv[1];
    FILE *fp = fopen(filename, "r");
    if (!fp) {
        perror("无法打开文件");
        return 1;
    }

    char line[MAX_LINE];
    int row = 0;
    size_t count4 = 0, capacity4 = 100;
    size_t count9 = 0, capacity9 = 100;
    double *values = malloc(capacity4 * sizeof(double));
    double *D_origin = malloc(capacity9 * sizeof(double));
    if (!values) {
        perror("malloc 失败");
        fclose(fp);
        return 1;
    }

    while (fgets(line, sizeof(line), fp)) {
        row++;

        // 跳过表头
        if (row == 1) continue;

        // 分割列
        char *token;
        char *rest = line;
        int col = 0;
        while ((token = strtok_r(rest, ",\n\r", &rest))) {
            col++;
            if (col == 4) {  // 第三列
                double val = atof(token);

                // 动态扩容
                if (count4 >= capacity4) {
                    capacity4*= 2;
                    double *tmp = realloc(values, capacity4 * sizeof(double));
                    if (!tmp) {
                        perror("realloc 失败");
                        free(values);
                        fclose(fp);
                        return 1;
                    }
                    values = tmp;
                }

                values[count4++] = val;
            }
            if (col == 9) {  // 第三列
                double val = atof(token);

                // 动态扩容
                if (count9 >= capacity9) {
                    capacity9 *= 2;
                    double *tmp = realloc(D_origin, capacity9 * sizeof(double));
                    if (!tmp) {
                        perror("realloc 失败");
                        free(D_origin);
                        fclose(fp);
                        return 1;
                    }
                    D_origin = tmp;
                }

                D_origin[count9++] = val;
            }
        }
    }

    fclose(fp);

    // 打印读取结果
    printf("Time,D,D0\n");


    const double fs = 20.0;
    const double fc = 2.0;
    double b0,b1,b2,a1,a2;
    BiquadCascadeState lp2;   // 静态或全局

    if (design_butterworth_lowpass_biquad(fc, fs, &b0,&b1,&b2,&a1,&a2) != 0) {
        printf("bad params\n");
        return 1;
    }
    biquad_cascade_init(&lp2);

    for (size_t i = 0; i < count4; i++) {
        double D = 1.0 * 0.07 * values[i]; 
        D = biquad_cascade_step(&lp2, D, b0,b1,b2, a1,a2);
        printf("%f,%f,%f\n", (double)i * 0.05, D,D_origin[i]);
    }


    free(values);
    return 0;
}