#ifndef ALG_SLOPE_H
#define ALG_SLOPE_H

/* Includes ------------------------------------------------------------------*/

/* Exported macros -----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

/**
 * @brief 规划优先类型, 分为目标值优先和真实值优先
 * 目标值优先, 即硬规划
 * 真实值优先, 即当前真实值夹在当前规划值和目标值之间, 当前规划值转为当前真实值
 *
 */
enum Enum_Slope_First
{
    Slope_First_REAL = 0,
    Slope_First_TARGET,
};

/**
 * @brief Reusable, 斜坡函数本体
 *
 */
class Class_Slope
{
public:
    void Init(float __Increase_Value, float __Decrease_Value, Enum_Slope_First __Slope_First = Slope_First_REAL);

    inline float Get_Out();

    inline void Set_Now_Real(float __Now_Real);

    inline void Set_Increase_Value(float __Increase_Value);

    inline void Set_Decrease_Value(float __Decrease_Value);

    inline void Set_Target(float __Target);

    // 新增：设置最大允许目标超前量（重点优化加速）
    inline void Set_Max_Target_Up_Delta(float __Max_Target_Up_Delta);

    // 新增：设置最大允许目标落后量（减速时可单独控制）
    inline void Set_Max_Target_Down_Delta(float __Max_Target_Down_Delta);

    // 新增：设置加速自适应系数，越大则误差大时加速越明显
    inline void Set_Adaptive_Gain(float __Adaptive_Gain);

    // 新增：设置单周期最大加速倍数限制
    inline void Set_Max_Adaptive_Ratio(float __Max_Adaptive_Ratio);

    // 新增：清零内部状态
    inline void Reset(float __Value = 0.0f);

    void TIM_Calculate_PeriodElapsedCallback();

protected:
    // 输出值
    float Out = 0.0f;

    // 规划优先类型
    Enum_Slope_First Slope_First = Slope_First_REAL;

    // 当前规划值
    float Now_Planning = 0.0f;

    // 当前真实值
    float Now_Real = 0.0f;

    // 绝对值增量, 一次计算周期改变值
    float Increase_Value = 0.0f;

    // 绝对值减量, 一次计算周期改变值
    float Decrease_Value = 0.0f;

    // 原始目标值（上层给的目标）
    float Raw_Target = 0.0f;

    // 实际参与斜坡运算的目标值（经过限差）
    float Target = 0.0f;

    // 最大允许目标高于当前真实值的幅度
    float Max_Target_Up_Delta = 0.0f;

    // 最大允许目标低于当前真实值的幅度
    float Max_Target_Down_Delta = 0.0f;

    // 自适应增益
    float Adaptive_Gain = 0.20f;

    // 最大步长倍率，防止误差大时冲得太猛
    float Max_Adaptive_Ratio = 5.0f;

    // 分段跟随相关参数
    float Latched_Target = 0.0f;      // 新增：锁死分段过程中的目标值，防抖抗干扰
    float Stage1_Ratio = 0.6f;        // 阶段一比例 
    int Pause_Wait_Ticks = 800;  // 如果是1ms进一次中断，500就是0.5秒！
    
    int Pause_Counter = 0;       
    int Current_State = 0;
};


class TrajectoryPlanner {
public:
    float start_pos, end_pos;
    float start_vel, end_vel; // 新增：起始速度和目标速度
    float T, t_curr;
    bool is_planning = false;

    // 系数缓存，避免每帧重复高幂次除法
    float a0, a1, a2, a3, a4, a5;

    // 初始化轨迹：支持起始速度和目标速度
    void MoveTo(float start, float end, float v_start, float v_end, float duration) {
        if (duration <= 0.0f) return;

        start_pos = start;
        end_pos = end;
        start_vel = v_start;
        end_vel = v_end;
        T = duration;
        t_curr = 0.0f;
        is_planning = true;

        // --- 五次多项式系数推导 (假设起始/终点加速度为0) ---
        // s(t) = a0 + a1*t + a2*t^2 + a3*t^3 + a4*t^4 + a5*t^5
        a0 = start;
        a1 = v_start;
        a2 = 0; // 初始加速度设为0，如果需要也可以参数化

        float T2 = T * T;
        float T3 = T2 * T;
        float T4 = T3 * T;
        float T5 = T4 * T;

        // 根据边界条件解线性方程组得到的解析解
        a3 = (20.0f * (end - start) - (8.0f * end_vel + 12.0f * start_vel) * T) / (2.0f * T3);
        a4 = (30.0f * (start - end) + (14.0f * end_vel + 16.0f * start_vel) * T) / (2.0f * T4);
        a5 = (12.0f * (end - start) - 6.0f * (end_vel + start_vel) * T) / (2.0f * T5);
    }

    void Step(float dt, float &ref_pos, float &ref_vel) {
        if (!is_planning) {
            ref_pos = end_pos;
            ref_vel = end_vel;
            return;
        }

        t_curr += dt;
        if (t_curr >= T) {
            t_curr = T;
            is_planning = false;
        }

        float t = t_curr;
        float t2 = t * t;
        float t3 = t2 * t;
        float t4 = t3 * t;
        float t5 = t4 * t;

        // 计算位置、速度
        ref_pos = a0 + a1 * t + a2 * t2 + a3 * t3 + a4 * t4 + a5 * t5;
        ref_vel = a1 + 2.0f * a2 * t + 3.0f * a3 * t2 + 4.0f * a4 * t3 + 5.0f * a5 * t4;
    }
};

/* Exported variables --------------------------------------------------------*/

/* Exported function declarations --------------------------------------------*/

/**
 * @brief 获取输出值
 *
 * @return 输出值
 */
inline float Class_Slope::Get_Out()
{
    return (Out);
}

/**
 * @brief 设定当前真实值
 *
 * @param __Now_Real 当前真实值
 */
inline void Class_Slope::Set_Now_Real(float __Now_Real)
{
    Now_Real = __Now_Real;
}

/**
 * @brief 设定绝对值增量, 一次计算周期改变值
 *
 * @param __Increase_Value 绝对值增量, 一次计算周期改变值
 */
inline void Class_Slope::Set_Increase_Value(float __Increase_Value)
{
    Increase_Value = __Increase_Value;
}

/**
 * @brief 设定绝对值减量, 一次计算周期改变值
 *
 * @param __Decrease_Value 绝对值减量, 一次计算周期改变值
 */
inline void Class_Slope::Set_Decrease_Value(float __Decrease_Value)
{
    Decrease_Value = __Decrease_Value;
}

/**
 * @brief 设定目标值（原始目标）
 *
 * @param __Target 目标值
 */
inline void Class_Slope::Set_Target(float __Target)
{
    Raw_Target = __Target;
}

/**
 * @brief 设置最大允许目标高于当前真实值的幅度
 *
 * @param __Max_Target_Up_Delta 最大允许超前量
 */
inline void Class_Slope::Set_Max_Target_Up_Delta(float __Max_Target_Up_Delta)
{
    Max_Target_Up_Delta = __Max_Target_Up_Delta;
}

/**
 * @brief 设置最大允许目标低于当前真实值的幅度
 *
 * @param __Max_Target_Down_Delta 最大允许落后量
 */
inline void Class_Slope::Set_Max_Target_Down_Delta(float __Max_Target_Down_Delta)
{
    Max_Target_Down_Delta = __Max_Target_Down_Delta;
}

/**
 * @brief 设置自适应增益
 *
 * @param __Adaptive_Gain 自适应增益
 */
inline void Class_Slope::Set_Adaptive_Gain(float __Adaptive_Gain)
{
    Adaptive_Gain = __Adaptive_Gain;
}

/**
 * @brief 设置最大步长倍率
 *
 * @param __Max_Adaptive_Ratio 最大步长倍率
 */
inline void Class_Slope::Set_Max_Adaptive_Ratio(float __Max_Adaptive_Ratio)
{
    Max_Adaptive_Ratio = __Max_Adaptive_Ratio;
}

/**
 * @brief 重置内部状态
 *
 * @param __Value 重置值
 */
inline void Class_Slope::Reset(float __Value)
{
    Out = __Value;
    Now_Planning = __Value;
    Now_Real = __Value;
    Raw_Target = __Value;
    Target = __Value;
}

#endif

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/