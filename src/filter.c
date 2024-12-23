#include "filter.h"

// 0.25fir
float firCoeff[FILTER_ORDER + 1] = {-0.00387132316747470, 1.75845919462194e-18, 0.0320877994100304, 0.116708621643743, 0.220701186106900, 0.268747432013603, 0.220701186106900, 0.116708621643743, 0.0320877994100304, 1.75845919462194e-18, -0.00387132316747470};

float inputBuffer[FILTER_ORDER + 1] = {0}; // 输入缓存，初始化为0

int currentIndex = 0;   // 环形缓冲区的当前索引

float firFilterProcess(float newInput)
{
    // 将新输入存储在当前索引的位置
    inputBuffer[currentIndex] = newInput;

    float output = 0.0; // 初始化输出值
    int index = currentIndex;

    // 计算滤波器输出
    for (int i = 0; i <= FILTER_ORDER; i++)
    {
        output += firCoeff[i] * inputBuffer[index];
        index--;
        if (index < 0)
        {
            index = FILTER_ORDER; // 如果索引超出范围，循环它
        }
    }

    // 更新当前索引，以便下次使用
    currentIndex++;
    if (currentIndex > FILTER_ORDER)
    {
        currentIndex = 0; // 保持索引在合理范围内
    }

    return output; // 返回计算出的输出值
}