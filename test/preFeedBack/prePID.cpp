#include <iostream>
#include <functional>
 
class PIDWithFeedforward {
private:
    double Kp, Ki, Kd;          // PID 参数
    double dt;                  // 采样周期
    double prevError;           // 上一次误差
    double integral;            // 积分累计
    std::function<double(double, double)> feedforward; // 前馈函数
 
public:
    // 构造函数
    PIDWithFeedforward(double Kp, double Ki, double Kd, double dt,
                       std::function<double(double, double)> feedforwardFunc)
        : Kp(Kp), Ki(Ki), Kd(Kd), dt(dt), prevError(0), integral(0), feedforward(feedforwardFunc) {}
 
    // 计算控制输出
    double compute(double setpoint, double output, double disturbance = 0.0) {
        // 计算误差
        double error = setpoint - output;
 
        // 比例项
        double P = Kp * error;
 
        // 积分项
        integral += error * dt;
        double I = Ki * integral;
 
        // 微分项
        double derivative = (error - prevError) / dt;
        double D = Kd * derivative;
 
        // 前馈补偿
        double FF = feedforward(setpoint, disturbance);
 
        // 更新上一次误差
        prevError = error;
 
        // 控制器输出 = PID 输出 + 前馈补偿
        return P + I + D + FF;
    }
};
 
int main() {
    // 定义前馈补偿函数
    auto feedforwardFunc = [](double setpoint, double disturbance) -> double {
        // 假设前馈补偿为设定值的一部分（可根据实际需求调整）
        double feedforwardGain = 1.0; // 前馈增益
        return feedforwardGain * setpoint - disturbance;
    };
 
    // 初始化 PID 控制器
    double Kp = 2.0, Ki = 0.5, Kd = 1.0, dt = 0.01; // PID 参数和采样周期
    PIDWithFeedforward pid(Kp, Ki, Kd, dt, feedforwardFunc);
 
    // 模拟控制过程
    double setpoint = 100.0; // 目标值
    double output = 0.0;     // 系统初始输出
    double disturbance = 5.0; // 外部扰动
 
    for (int i = 0; i < 100; ++i) {
        // 获取控制器输出
        double controlSignal = pid.compute(setpoint, output, disturbance);
 
        // 模拟系统动态（简单一阶系统）
        output += controlSignal * dt; // 更新系统输出
 
        // 打印结果
        std::cout << "Time: " << i * dt
                  << "s, Output: " << output
                  << ", Control Signal: " << controlSignal << std::endl;
    }
 
    return 0;
}