#include <cmath>

/**
 * @brief 冒泡排序，把数组按升序就地排列
 */
void bubble_sort(float arr[], int n) {
    for (int i = 0; i < n - 1; i++) {
        for (int j = 0; j < n - i - 1; j++) {
            if (arr[j] > arr[j + 1]) {
                float temp = arr[j];
                arr[j] = arr[j + 1];
                arr[j + 1] = temp;
            }
        }
    }
}

/**
 * @brief 中值滤波：把新采样压入长度为 size 的滑动窗口，再取中位数作为输出，用于抑制脉冲噪声
 * @return       当前窗口内 size 个采样值的中值
 */
float median_filter(float input, float* buffer, int size) {
    for (int i = size - 1; i > 0; i--) {
        buffer[i] = buffer[i - 1];
    }
    buffer[0] = input;

    float temp_buf[size];
    for (int i = 0; i < size; i++) {
        temp_buf[i] = buffer[i];
    }

    bubble_sort(temp_buf, size);
    return temp_buf[size / 2];
}

/**
 * @brief          : 状态切换处理函数
 * @param mode      mode = 1：长按约 0.8 秒翻转一次
 * @param mode      mode = 2：上升沿触发模式（边沿检测）翻转一次
 */
void StateChange(bool input, bool* lastInput, bool* state, int mode)
{
    // 用于根据输入信号的持续方式，控制某个状态变量的翻转
    // static 变量只在第一次调用时初始化，之后每次调用都保持其值
    static int internalTimer = 0;    // 内部计时器，记录输入持续的时间（按调用周期计数）
    static bool long_triggered = 0;  // 长按触发标记，防止同一次长按导致状态重复翻转
    const int LONG_THRESHOLD = 800;  // 长按阈值：按调用周期数计，具体时长取决于该函数被调用的频率
    // 指针判空，防止非法访问
    if (!lastInput || !state) return;

    // mode = 1：长按触发模式
    if(mode == 1)
    {
        // 输入持续为高电平
        if(input)
        {
            internalTimer++;
            // 长按模式：累计时长达到阈值时翻转一次状态
            if (internalTimer >= LONG_THRESHOLD && !long_triggered) 
            {
                *state = !*state;        // 翻转状态
                long_triggered = true;   // 标记已触发，防止本次长按再次翻转
                internalTimer = 0;       // 计时器清零（松开重新计时）
            }
        }
        else
        {
            // 输入松开后清除触发标记，允许下一次长按重新触发
            long_triggered = false;
        }
    }
    // mode = 2：上升沿触发模式（边沿检测）
    else if(mode == 2)
    {
        // 检测到输入由 0 变为 1 的瞬间
        if (input && !(*lastInput)) 
        {
            if (internalTimer > -1) 
            {
                *state = !*state;    // 翻转状态
                internalTimer = 0;   // 计时器清零
            }
        }
    }

    // 保存本次输入值，供下一次边沿检测使用
    *lastInput = input;
}

/**
 * @brief 一阶滤波，对输入信号进行平滑处理
 */
void first_order_filter_calc(float x, float *y, float *last_y, float alpha)
{
    *y = alpha * x + (1-alpha) * *last_y;
    *last_y = *y;
}

float Max_Output(float num,float max)
{
	if(num>=max) return max;
	else if(num<=-max) return -max;
	else return num;
}

float Float_Math_Constrain(float *x, float Min, float Max)
{
    if (*x < Min)
    {
        *x = Min;
    }
    else if (*x > Max)
    {
        *x = Max;
    }
    return (*x);
}

/**
 * @brief 计算电机功率
 */
float power_calculate(float K_0, float K_1, float K_2, float A, float Current, float Omega)
{
    return (K_0 * Current * Omega + K_1 * Omega * Omega + K_2 * Current * Current + A);
}

float Float_Math_Abs(float x)
{
	return ((x > 0) ? x : -x);
}

/**
 * @brief 角度归一化，将角度转换为 [-π, π) 范围
 */
void normalizeAngle(float *angle) 
{
    float normalized = std::fmod(*angle, 2.0 * M_PI);
  
    if (normalized < 0) {
        normalized += 2.0 * M_PI;
    }
    
    if (normalized >= M_PI) {
        normalized -= 2.0 * M_PI;
    }
    
    *angle = normalized;
}

/**
 * @brief 对角度进行模归一化[-modulus/2, +modulus/2]
 * @param x 角度
 * @param modulus 模
 * @return float 归一化后的角度
 */
float Math_Modulus_Normalization(float x, float modulus)
{
    float tmp;

    tmp = fmodf(x + modulus / 2.0f, modulus);

    if (tmp < 0.0f) 
    {
        tmp += modulus;
    }

    return (tmp - modulus / 2.0f);
}

/**
 * @brief Get_target_angle 获取目标角度
 * 
 * @param x 目标点x坐标
 * @param y 目标点y坐标
 * @param Now_angle 当前角度
 * @param vel 角度变化率
 * @return float 目标角度
 */ 
float Get_target_angle(float x, float y, float Now_angle, float *vel)
{
    static float target_angle;
    target_angle = atan2f(y, x);

    float tmp_delta_angle = Math_Modulus_Normalization(target_angle - Now_angle, 2.0f * M_PI);

    if (-M_PI / 2.0f <= tmp_delta_angle && tmp_delta_angle <= M_PI / 2.0f)
    {
        target_angle = tmp_delta_angle + Now_angle;
    }
    else
    {
        target_angle = Math_Modulus_Normalization(tmp_delta_angle + M_PI, 2.0f * M_PI) + Now_angle;
        *vel *= -1.0f;
    }

    return target_angle;
}
