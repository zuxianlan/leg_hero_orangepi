/* Includes ------------------------------------------------------------------*/

#include "slope.h"

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function declarations ---------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/

/**
 * @brief 求绝对值
 *
 * @tparam Type 类型
 * @param x 传入数据
 * @return Type x的绝对值
 */
template<typename Type>
Type Math_Abs(Type x)
{
    return ((x > 0) ? x : -x);
}

/**
 * @brief 初始化
 *
 * @param __Increase_Value 增长基础幅度
 * @param __Decrease_Value 降低基础幅度
 */
void Class_Slope::Init(float __Increase_Value, float __Decrease_Value, Enum_Slope_First __Slope_First)
{
    Increase_Value = __Increase_Value;
    Decrease_Value = __Decrease_Value;
    Slope_First = __Slope_First;

    // 保底，避免出现0导致完全不动
    if (Adaptive_Gain <= 0.0f)
    {
        Adaptive_Gain = 0.20f;
    }

    if (Max_Adaptive_Ratio < 1.0f)
    {
        Max_Adaptive_Ratio = 1.0f;
    }
}

/**
 * @brief 斜坡函数调整值, 计算周期取决于调用者
 *
 */
void Class_Slope::TIM_Calculate_PeriodElapsedCallback()
{
    // =========================================================
    // --- 0. 状态机：处理分段加速与平台期等待 ---
    // =========================================================
    
    // 1. 刹车/归零逻辑：只要目标接近0，立刻打破所有状态，快速回零
    if (Math_Abs(Raw_Target) < 1e-3f)
    {
        Current_State = 0;
        Latched_Target = 0.0f;
    }
    // 2. 起步触发：只有在【正常跟随】状态下，且目标与当前速度的【差距足够大】时，才触发分段！
    else if (Current_State == 0)
    {
        // 这里的 1.0f 是触发阶梯动作的阈值。
        // 如果你当前速度是 0，目标是 2.0，差距为 2.0 > 1.0，就会完美触发分段。
        // 如果你已经在 2.0 稳住了，目标还是 2.0，差距为 0，就不会再“掉坑”了！
        if (Math_Abs(Raw_Target - Now_Planning) > 1.0f) 
        {
            Latched_Target = Raw_Target; // 锁死当前目标
            Current_State = 1;           // 进入阶段1
            Pause_Counter = 0;
        }
        else
        {
            // 差距很小（比如目标微调，或者已经到达稳态），直接正常顺滑跟随
            Latched_Target = Raw_Target; 
        }
    }
    // 3. 剧烈变向打断：分段正在进行中，但目标指令又发生了剧烈改变
    else if (Current_State != 0 && Math_Abs(Raw_Target - Latched_Target) > 1.0f) 
    {
        Latched_Target = Raw_Target;
        
        float Stage1_Target = Latched_Target * Stage1_Ratio;
        if ((Now_Planning * Latched_Target > 0.0f) && (Math_Abs(Now_Planning) >= Math_Abs(Stage1_Target) - 0.1f)) {
            Current_State = 3;
        } else {
            Current_State = 1;
            Pause_Counter = 0;
        }
    }

    // 根据状态机，给出当前的临时目标 (下面 switch 里的代码不用动)
    float Current_Stage_Target = Latched_Target;

    switch (Current_State)
    {
        case 1: // 【状态1：阶段一加速】
            Current_Stage_Target = Latched_Target * Stage1_Ratio;
            if (Math_Abs(Now_Planning - Current_Stage_Target) <= (Increase_Value + 0.1f))
            {
                Current_State = 2; // 到达阶段1，进入等待
                Pause_Counter = 0; 
            }
            break;

        case 2: // 【状态2：平台期等待】
            Current_Stage_Target = Now_Planning; // 锁死规划值，画出水平线
            Pause_Counter++;
            if (Pause_Counter >= Pause_Wait_Ticks) // 必须等够时间！
            {
                Current_State = 3; // 倒计时结束，进入阶段2
            }
            break;

        case 3: // 【状态3：阶段二加速向最终目标】
            Current_Stage_Target = Latched_Target;
            if (Math_Abs(Now_Planning - Current_Stage_Target) <= (Increase_Value + 0.1f))
            {
                // 分段起步全流程结束，回归状态0（正常跟随 Raw_Target）
                Current_State = 0; 
            }
            break;

        case 0: // 【状态0：正常巡航/跟随】
        default:
            if (Math_Abs(Raw_Target - Latched_Target) > 0.05f) 
            {
                Latched_Target = Raw_Target; 
            }
            Current_Stage_Target = Latched_Target; 
            break;
    }
    // =========================================================

    // 1. 先根据当前真实值，对【临时目标】做限差处理
    Target = Raw_Target;

    if (Max_Target_Up_Delta > 0.0f)
    {
        if (Target > Now_Real + Max_Target_Up_Delta)
        {
            Target = Now_Real + Max_Target_Up_Delta;
        }
    }

    if (Max_Target_Down_Delta > 0.0f)
    {
        if (Target < Now_Real - Max_Target_Down_Delta)
        {
            Target = Now_Real - Max_Target_Down_Delta;
        }
    }

    // 2. 真实值优先逻辑
    if (Slope_First == Slope_First_REAL)
    {
        if ((Target >= Now_Real && Now_Real >= Now_Planning) ||
            (Target <= Now_Real && Now_Real <= Now_Planning))
        {
            Out = Now_Real;
            Now_Planning = Out;
        }
    }

    // 3. 计算当前误差
    float Error = Target - Now_Planning;
    float Abs_Error = Math_Abs(Error);

    // 死区防抖
    if (Abs_Error <= 1e-6f)
    {
        Out = Target;
        Now_Planning = Out;
        return;
    }

    float Step = 0.0f;

    // 4. 按方向分别处理
    if (Error > 0.0f)
    {
        // 加速方向：基础步长 + 自适应误差项
        Step = Increase_Value + Adaptive_Gain * Abs_Error;

        // 限制最大步长，防止误差太大时冲得过猛
        if (Step > Increase_Value * Max_Adaptive_Ratio)
        {
            Step = Increase_Value * Max_Adaptive_Ratio;
        }

        // 防止越过目标
        if (Step > Abs_Error)
        {
            Step = Abs_Error;
        }

        Out = Now_Planning + Step;
    }
    else
    {
        // 减速方向：一般允许更果断
        Step = Decrease_Value + Adaptive_Gain * Abs_Error;

        if (Step > Decrease_Value * Max_Adaptive_Ratio)
        {
            Step = Decrease_Value * Max_Adaptive_Ratio;
        }

        if (Step > Abs_Error)
        {
            Step = Abs_Error;
        }

        Out = Now_Planning - Step;
    }

    // 5. 更新内部规划值
    Now_Planning = Out;
}