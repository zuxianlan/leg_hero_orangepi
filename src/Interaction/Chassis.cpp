/**
  ******************************************************************************
  * @file           : Chassis.cpp
  * @author         : Chen Haoran
  * @brief          : 轮腿底盘控制实现。负责电机反馈更新、腿 VMC/MPC/LQR 计算、
  *                    轮速观测与打滑抑制，以及零力、检查、上台阶等模式控制；
  *                    通过 INS/mc_02 指针读取传感器，输出轮力矩 T_L/T_R 和腿力/腿矩指令。
  * @attention      : None
  * @date           : 2025/11/4
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "Chassis.h"
#include <ranges>
#include "Algorithm/VMC/lqr_k_calc.h"
#include "user_lib.h"
/* Define --------------------------------------------------------------------*/

/* Enum ----------------------------------------------------------------------*/

/* Variable && Struct --------------------------------------------------------*/

/* Function Declaration ------------------------------------------------------*/

/* Function ------------------------------------------------------------------*/
float P[40][6] = 
{ 
	{-0.9114, -17.336, 4.5823, 17.454, 6.1675, -7.2869},
    {-1.4125, -18.315, 6.2039, 19.623, 3.9653, -8.7543},
    {-8.7714, 18.754, -12.309, -18.845, 4.3306, 14.373},
    {-2.3138, 5.4872, -3.9126, -5.0018, 1.2468, 4.5794},
    {-3.7196, -74.88, 17.691, 50.724, 19.991, -25.904},
    {-0.2118, -6.3291, 0.82563, 0.87253, 3.2788, -1.7858},
    {-2.1176, 0.082412, -37.85, 4.5016, 29.005, 30.37},
    {0.025268, -1.0123, -2.6438, 1.8442, -1.3215, 1.3436},
    {-46.761, 26.77, 68.96, 57.526, -69.581, -49.709},
    {-5.2271, 2.9101, 10.285, 5.2689, -9.4969, -8.932},
    {-0.9114, 4.5823, -17.336, -7.2869, 6.1675, 17.454},
    {-1.4125, 6.2039, -18.315, -8.7543, 3.9653, 19.623},
    {8.7714, 12.309, -18.754, -14.373, -4.3306, 18.845},
    {2.3138, 3.9126, -5.4872, -4.5794, -1.2468, 5.0018},
    {-2.1176, -37.85, 0.082412, 30.37, 29.005, 4.5016},
    {0.025268, -2.6438, -1.0123, 1.3436, -1.3215, 1.8442},
    {-3.7196, 17.691, -74.88, -25.904, 19.991, 50.724},
    {-0.2118, 0.82563, -6.3291, -1.7858, 3.2788, 0.87253},
    {-46.761, 68.96, 26.77, -49.709, -69.581, 57.526},
    {-5.2271, 10.285, 2.9101, -8.932, -9.4969, 5.2689},
    {19.974, 38.725, -75.991, -123.17, 83.706, 58.191},
    {24.558, 35.834, -94.78, -134.3, 115.03, 71.896},
    {-6.2838, -76.229, -19.68, 134.15, -72.052, 31.923},
    {-1.5012, -24.271, -4.5508, 40.743, -23.401, 6.5058},
    {61.078, 135.33, -52.778, -256.22, 204.99, 38.185},
    {6.8868, 6.6418, -6.4843, -7.3449, 3.8276, 6.3082},
    {12.623, -45.549, -76.566, 44.223, -164.84, 35.273},
    {0.56421, 3.0834, 0.53641, -11.23, -0.46667, -8.8167},
    {-17.042, -862.17, 48.374, 854.23, 270.18, -45.067},
    {1.9139, -64.232, -9.383, 42.744, 46.631, 9.854},
    {19.974, -75.991, 38.725, 58.191, 83.706, -123.17},
    {24.558, -94.78, 35.834, 71.896, 115.03, -134.3},
    {6.2838, 19.68, 76.229, -31.923, 72.052, -134.15},
    {1.5012, 4.5508, 24.271, -6.5058, 23.401, -40.743},
    {12.623, -76.566, -45.549, 35.273, -164.84, 44.223},
    {0.56421, 0.53641, 3.0834, -8.8167, -0.46667, -11.23},
    {61.078, -52.778, 135.33, 38.185, 204.99, -256.22},
    {6.8868, -6.4843, 6.6418, 6.3082, 3.8276, -7.3449},
    {-17.042, 48.374, -862.17, -45.067, 270.18, 854.23},
    {1.9139, -9.383, -64.232, 9.854, 46.631, 42.744}
};

void class_Chassis::Init()
{
    /******************* pid *******************/
    PID_Omega.Init(5.2f, 0.01f, 0.0001f, 0.0f, 0.0f, 5.0f, 0.002f, 0.0f, 0.0f, 0.0f, 0.0f, PID_D_First_DISABLE);

    PID_tp.Init(90.0f, 0.0f, 0.0f, 0.00f, 0.0f, 5.0f, 0.002f, 0.0f, 0.0f, 0.0f, 0.0f, PID_D_First_ENABLE);

    PID_tp_omega.Init(1.5f, 0.0f, 0.0f, 0.00f, 0.0f, 10.0f, 0.002f, 0.0f, 0.0f, 0.0f, 0.0f, PID_D_First_ENABLE);

    /******************* kf *******************/
    kf.KF_Init();
    /******************* mpc *******************/
    MPC_body.MPC_Init();

    /******************* 10 状态 MPC *******************/
    // dt 必须等于 TIM_4ms 的实际周期（0.004 s）
    mpc_robot = std::make_unique<Robot_MPC>();
    mpc_robot->Init(0.004);

}

/*
 * @brief 更新反馈信息
 */
void class_Chassis::feedback_update()
{
    normalizeAngle(&mc_02->Rx_Data.Motor_joint[0].Position);
    normalizeAngle(&mc_02->Rx_Data.Motor_joint[1].Position);
    normalizeAngle(&mc_02->Rx_Data.Motor_joint[2].Position);
    normalizeAngle(&mc_02->Rx_Data.Motor_joint[3].Position);

    left_leg.Set_phi1(static_cast<float>(std::numbers::pi) + mc_02->Rx_Data.Motor_joint[1].Position);
    left_leg.Set_dphi1(mc_02->Rx_Data.Motor_joint[1].omega);
    left_leg.Set_phi4(mc_02->Rx_Data.Motor_joint[0].Position);
    left_leg.Set_dphi4(mc_02->Rx_Data.Motor_joint[0].omega);
    left_leg.VMC_Calc_1();

    right_leg.Set_phi1(static_cast<float>(std::numbers::pi) - mc_02->Rx_Data.Motor_joint[2].Position);
    right_leg.Set_dphi1(-mc_02->Rx_Data.Motor_joint[2].omega);
    right_leg.Set_phi4(-mc_02->Rx_Data.Motor_joint[3].Position);
    right_leg.Set_dphi4(-mc_02->Rx_Data.Motor_joint[3].omega);
    right_leg.VMC_Calc_1();

    theta_err = left_leg.Get_theta() - right_leg.Get_theta();
    dtheta_err = left_leg.Get_dtheta() - right_leg.Get_dtheta();
    // ========== 当前身体高度与高度变化率（考虑roll角补偿） ==========
    Now_height = (left_leg.Get_L0() * cosf(INS->roll) + right_leg.Get_L0() * cosf(INS->roll)) / 2;
    Now_dheight = (left_leg.Get_dL0() * cosf(INS->roll) + right_leg.Get_dL0() * cosf(INS->roll)) / 2;

    Fitting_K_Calc(Fitting_K, P, left_leg.Get_L0(), right_leg.Get_L0());

    err[0] = (X_filter - Target_X);                                      // 水平位移误差
    err[1] = (Velocity_filter - Target_Velocity);                        // 水平速度误差
    err[2] = 0;                                                          // 偏航角误差（当前禁用）
    err[3] = (INS->gyro[2] - Target_Omega);                              // 偏航角速度误差
    err[4] = Max_Output((left_leg.Get_theta() - Target_Theta), 0.5);     // 左腿角度误差（限幅±0.5）
    err[5] = (left_leg.Get_dtheta() - 0);                                // 左腿角速度误差
    err[6] = Max_Output((right_leg.Get_theta() - Target_Theta), 0.5);    // 右腿角度误差（限幅±0.5）
    err[7] = (right_leg.Get_dtheta() - 0);                               // 右腿角速度误差
    err[8] = Max_Output((INS->pitch - 0), 1.0);                          // 俯仰角误差（限幅±1.0）
    err[9] = (INS->gyro[0] - 0);                                         // 俯仰角速度误差
}

void class_Chassis::Zero_Force_Control()
{
    PID_Omega.Set_Integral_Error(0);
    PID_tp.Set_Integral_Error(0);

    left_leg.Set_Tp(0);
    left_leg.Set_F0(0);
    left_leg.VMC_Calc_1();
    left_leg.VMC_Calc_2();
    T_L = 0;
    Tp_L = 0;

    right_leg.Set_Tp(0);
    right_leg.Set_F0(0);
    right_leg.VMC_Calc_1();
    right_leg.VMC_Calc_2();
    T_R = 0;
    Tp_R = 0;

    Record_MPC_History(T_L, T_R, left_leg.Get_Tp(), right_leg.Get_Tp());

    X_filter = 0;
    last_Target_X = 0;
}


void class_Chassis::Follow_Gimbal_Control()
{
    PID_tp.Set_Target(0);
    PID_tp.Set_Now(theta_err);
    PID_tp.TIM_Calculate_PeriodElapsedCallback();

    PID_tp_omega.Set_Target(PID_tp.Get_Out());
    PID_tp_omega.Set_Now(dtheta_err);
    PID_tp_omega.TIM_Calculate_PeriodElapsedCallback();

    // ===== MPC身体姿态计算 =====
    MPC_Body_Calc();

    // ===== 根据底盘状态机模式设置控制参数 =====
    if (chassis_fsm_mode == ABOVE_GROUND)
    {
        // 腾空状态：关闭摩擦补偿，清零角速度PID输出
        Target_Theta = 0.0;
        M = 180.0;
        F_mc = 0.0;
        PID_Omega.Set_Out(0.0);
    }
    else
    {
        // 着地状态：根据滤波速度与陀螺仪Z轴角速度计算摩擦补偿力
        Target_Theta = 0.0;
        M = 180.0;
        F_mc = Velocity_filter * INS->gyro[2] * 28.0f;
    }
    
    Float_Math_Constrain(&F_mc, 0, 100.0f);
    // 根据左右腿当前长度计算虚拟弹簧力（腿越长弹簧力越大）
    spring_forcel = 100 - 10*(1 - (left_leg.Get_L0()-0.1)/0.28);
    spring_forcer = 100 - 10*(1 - (right_leg.Get_L0()-0.1)/0.28);

    // 同时满足目标速度和实际速度均很小时，进入停止状态
    if (fabs(Target_Velocity) < 0.05f && //目标速度小于此阈值
        fabs(Velocity_filter) < 0.5f)    //返回转速小于此阈值
        {stop_flag = true;} 
    else if (fabs(Target_Velocity) > 0.01f) //目标速度大于此阈值
        {stop_flag = false;}
    
    // ===== 根据停止标志设置LQR目标状态 =====
    if (stop_flag) {
    if (!last_stop_flag) {
        // 刚进入停止状态：清零滤波器与目标位置，记录当前位置
        X_filter = 0.0;
        Target_X = X_filter;
        last_Target_X = X_filter;
    } else {
        // 持续停止状态：保持目标位置不变，施加小角度使机器人微微前倾以维持稳定
        Target_X = last_Target_X; 
        Target_Theta = 0.04;
        
    }
    } else {
    // 运动状态：清零所有滤波和目标值
    X_filter = 0.0;
    Target_Theta = 0.0;
    Target_X = X_filter;
    last_Target_X = X_filter;  
    }
    LQR_Calc();

    // ===== 10 状态 MPC 接管（若 4ms 任务已算出有效解）=====
    // 条件与 LQR 的正常分支一致：不在腾空、不在小陀螺
    // 轮矩仍乘打滑系数；髋矩直接使用 MPC 结果
    // 且要求解是「新鲜」的（8 ms 内），防止 4 ms 线程卡住时一直复用旧解
    mpc_applied = (mpc_enable && MPC_Output_Fresh() && !spin_mode && chassis_fsm_mode != ABOVE_GROUND);
    if (mpc_applied)
    {
        T[0] = T_mpc[0] * slip_factor_l;
        T[1] = T_mpc[1] * slip_factor_r;
        T[2] = T_mpc[2];
        T[3] = T_mpc[3];
    }

    // ===== 左腿控制量计算 =====
    T_L = T[0];
    left_leg.Set_Tp(T[2]- PID_tp_omega.Get_Out()); 
    left_leg.Set_F0(MPC_body.u_out(0) - spring_forcel + M / cosf(left_leg.Get_theta()) + Jump_force - F_mc + 5.0f);
    left_leg.VMC_Calc_2();

    // ===== 右腿控制量计算 =====
    T_R = T[1];
    right_leg.Set_Tp(T[3]+ PID_tp_omega.Get_Out());  
    right_leg.Set_F0(-MPC_body.u_out(1) + spring_forcer - M / cosf(right_leg.Get_theta()) - Jump_force - F_mc - 0.0f);
    right_leg.VMC_Calc_2();
    
    // ===== 记录本拍实际生效的通道命令，供下一拍 Δu 约束当起点 =====
    // 轮矩反除 slip_factor：T[0..1] 已被打滑系数缩放过，反除回来才是决策变量量纲；
    //   否则打滑时 u_last 只有 0.35 倍，Δu 会把力矩一拍一拍越拽越低。
    // 髋矩记 T[2]/T[3]（不含 PID trim）：trim 属于另一个反馈环，不该算进 MPC 的历史。
    {
        const bool slip_on = (!spin_mode && chassis_fsm_mode != ABOVE_GROUND);
        Record_MPC_History(T[0] / (slip_on ? slip_factor_l : 1.0f),
                           T[1] / (slip_on ? slip_factor_r : 1.0f),
                           T[2], T[3]);
    }

    last_Target_X = Target_X;
    last_stop_flag = stop_flag;
}

void class_Chassis::Check_in_Control()
{
    /******************* left_leg *******************/

    static int mod = 0;
    // mod++;

    // 左轮扭矩 = 目标速度反向（速度控制模式，无髋关节参与）
    T_L = -Target_Velocity;
    left_leg.Set_Tp(0);  // 髋关节扭矩清零，纯轮式运动

    // 左腿超伸软限位保护：当腿长超过0.34m(最大行程)时，施加向下的虚拟力拉回
    // ll = (L0/0.34)^2 - 1，仅在 L0 > 0.34 时为正
    ll = 0 - (1 - pow(left_leg.Get_L0()/0.34, 2));
    if(ll < 0)
    {
        ll = 0;
    }
    left_leg.Set_F0(-ll);  // 负力向下，防止腿过度伸展
    left_leg.VMC_Calc_2();

    // 右轮扭矩 = 目标速度反向
    T_R =  -Target_Velocity;
    // T_R =  + 0;
    
    right_leg.Set_Tp(0);  // 髋关节扭矩清零

    // 右腿超伸软限位保护：线性计算，当腿长超过0.34m时施加向下的力
    // rr = L0/0.34 - 1，仅在 L0 > 0.34 时为正
    rr = 0 - (1 - right_leg.Get_L0()/0.34);
    if(rr < 0)
    {
        rr = 0;
    }
    right_leg.Set_F0(rr);  // 正力向下，防止腿过度伸展
    right_leg.VMC_Calc_2();

    Record_MPC_History(T_L, T_R, left_leg.Get_Tp(), right_leg.Get_Tp());

    // 清零速度滤波器（入场模式下不累积速度估计）
    X_filter = 0;

}


void class_Chassis::Control_leg()
{
    PID_Omega.Set_Integral_Error(0);

    PID_tp.Set_Target(0);
    PID_tp.Set_Now(theta_err);
    PID_tp.TIM_Calculate_PeriodElapsedCallback();

    PID_tp_omega.Set_Target(PID_tp.Get_Out());
    PID_tp_omega.Set_Now(dtheta_err);
    PID_tp_omega.TIM_Calculate_PeriodElapsedCallback();


    left_leg.VMC_Calc_1();
    T_L = 0;
    right_leg.VMC_Calc_1();
    T_R = 0;
    X_filter = 0;

    if (fabs(Target_height - last_Target_height) > 0.001f) 
    {
        height_planner.MoveTo(Now_height, Target_height, Now_dheight, Target_dheight, 2.5f); // 0.5����ɶ���?
        last_Target_height = Target_height;
    }

    if (height_planner.is_planning) 
    {
        height_planner.Step(dt, current_ref_height, current_ref_dheight);
    } else 
    {
        current_ref_height = Target_height;
        current_ref_dheight = 0.0f;
    }

    MPC_body.x(0) = INS->roll;
    MPC_body.x(1) = INS->gyro[1];
    MPC_body.x(2) = Now_height;
    MPC_body.x(3) = Now_dheight;
    MPC_body.x_ref.setZero();
    for (int i = 0; i < N; i++)
    {
        MPC_body.x_ref(i*4) = INS->roll;
        MPC_body.x_ref(i*4+1) = 0.0;
        MPC_body.x_ref(i*4+2) = current_ref_height;
        MPC_body.x_ref(i*4+3) = current_ref_dheight;
    }
    MPC_body.MPC_Calculate();

    spring_forcel = 130 - 10*(1 - (left_leg.Get_L0()-0.1)/0.24);
    spring_forcer = 130 - 10*(1 - (right_leg.Get_L0()-0.1)/0.24);

    T_L = 0.0;
    left_leg.Set_Tp(4.0 - PID_tp_omega.Get_Out());
    left_leg.Set_F0(MPC_body.u_out(0) / 1.5 - spring_forcel);
    left_leg.VMC_Calc_2();

    T_R = 0.0;
    right_leg.Set_Tp(4.0 + PID_tp_omega.Get_Out());
    right_leg.Set_F0(-MPC_body.u_out(1) / 1.5 + spring_forcer);
    right_leg.VMC_Calc_2();

    Record_MPC_History(T_L, T_R, left_leg.Get_Tp(), right_leg.Get_Tp());


}

void class_Chassis::Control_leg_left(float target)
{
    if(target != 0)
    {
        T_L = 0.0;
        Tp_L = (left_leg.Get_dtheta() - target) * Fitting_K[2][5];

        left_leg.Set_Tp(Tp_L);
        left_leg.Set_F0(0.0);
        left_leg.VMC_Calc_2();
    }
    else
    {
        left_leg.Set_Tp(0.0);
        left_leg.Set_F0(0.0);
        left_leg.VMC_Calc_2();
    }

    Record_MPC_History(T_L, T_R, left_leg.Get_Tp(), right_leg.Get_Tp());
}

void class_Chassis::Control_leg_right(float target)
{
    if(target != 0)
    {
        T_R = 0.0;
        Tp_R = (right_leg.Get_dtheta() - target) * Fitting_K[3][7];

        right_leg.Set_Tp(Tp_R);
        right_leg.Set_F0(0.0);
        right_leg.VMC_Calc_2();
    }
    else
    {
        right_leg.Set_Tp(0.0);
        right_leg.Set_F0(0.0);
        right_leg.VMC_Calc_2();
    }

    Record_MPC_History(T_L, T_R, left_leg.Get_Tp(), right_leg.Get_Tp());
}

void class_Chassis::UP_Staris_Control_leg()
{

    PID_Omega.Set_Integral_Error(0);
    PID_tp.Set_Target(0);
    PID_tp.Set_Now(theta_err);
    PID_tp.TIM_Calculate_PeriodElapsedCallback();

    PID_tp_omega.Set_Target(PID_tp.Get_Out());
    PID_tp_omega.Set_Now(dtheta_err);
    PID_tp_omega.TIM_Calculate_PeriodElapsedCallback();

    left_leg.VMC_Calc_1();
    T_L = 0;
    right_leg.VMC_Calc_1();
    T_R = 0;
    X_filter = 0;

    if (fabs(Target_height - last_Target_height) > 0.001f) 
    {
        height_planner.MoveTo(Now_height, Target_height, Now_dheight, 0.25, 2.5f); // 0.5����ɶ���?
        last_Target_height = Target_height;
    }

    if (height_planner.is_planning) 
    {
        height_planner.Step(dt, current_ref_height, current_ref_dheight);
    } else 
    {
        current_ref_height = Target_height;
        current_ref_dheight = 0.0f;
    }

    MPC_body.x(0) = INS->roll;
    MPC_body.x(1) = INS->gyro[1];
    MPC_body.x(2) = Now_height;
    MPC_body.x(3) = Now_dheight;
    MPC_body.x_ref.setZero();
    for (int i = 0; i < N; i++)
    {
        MPC_body.x_ref(i*4) = 0.0;
        MPC_body.x_ref(i*4+1) = 0.0;
        MPC_body.x_ref(i*4+2) = current_ref_height;
        MPC_body.x_ref(i*4+3) = current_ref_dheight;
    }
    MPC_body.MPC_Calculate();
    left_leg.Set_Tp(-1.0 - PID_tp_omega.Get_Out());
    T_L = 0.0;

    spring_forcel = 30 - 10*(1 - (left_leg.Get_L0()-0.1)/0.24);
    spring_forcer = 30 - 10*(1 - (right_leg.Get_L0()-0.1)/0.24);

    left_leg.Set_F0(MPC_body.u_out(0) / 3.5 - spring_forcel - 0);
    left_leg.VMC_Calc_2();

    right_leg.Set_Tp(-1.0 + PID_tp_omega.Get_Out());

    T_R = 0.0;
    
    right_leg.Set_F0(-MPC_body.u_out(1) / 3.5 + spring_forcer + 0);
    right_leg.VMC_Calc_2();

    Record_MPC_History(T_L, T_R, left_leg.Get_Tp(), right_leg.Get_Tp());

    X_filter = 0;
    last_Target_X = 0;
    
}
// void class_Chassis::Self_Observe()
// {
//     static float omega_l, omega_r = 0.0f;
//     static float speed_l, speed_r = 0.0f;
//     //static float average_speed = 0.0f;

//     omega_l = mc_02->Rx_Data.Motor_wheel[0].omega - INS->gyro[0] + left_leg.Get_dtheta();
//     speed_l = 0.06 * omega_l + left_leg.Get_L0() * left_leg.Get_dtheta() * cosf(left_leg.Get_theta()) + left_leg.Get_dL0() * sinf(left_leg.Get_theta());
//     omega_r = mc_02->Rx_Data.Motor_wheel[1].omega - INS->gyro[0] + right_leg.Get_dtheta();
//     speed_r = 0.06 * omega_r + right_leg.Get_L0() * right_leg.Get_dtheta() * cosf(right_leg.Get_theta()) + right_leg.Get_dL0() * sinf(right_leg.Get_theta());
    
//     average_speed = -(speed_l - speed_r) / 2.0f;
//     kf.z << average_speed,
//             0.0f;                       
//     kf.Predict();
//     kf.Update();

//     Velocity_filter = kf.x_hat(0);
   
//     X_filter = X_filter + Velocity_filter * (static_cast<float>(1)/1000.0f);

//     Slip_Detection_And_Suppression();

// }

// void class_Chassis::Slip_Detection_And_Suppression()
// {
//     float w_l = mc_02->Rx_Data.Motor_wheel[0].omega;
//     float w_r = mc_02->Rx_Data.Motor_wheel[1].omega;

//     float alpha_l = (w_l - last_wheel_omega_l) / dt;
//     float alpha_r = (w_r - last_wheel_omega_r) / dt;
//     last_wheel_omega_l = w_l;
//     last_wheel_omega_r = w_r;

//     float v_wheel_l = w_l * 0.06f;
//     float v_wheel_r = -w_r * 0.06f;

//     float v_body = Velocity_filter;

//     float slip_err_l = fabsf(v_wheel_l) - fabsf(v_body) - 0.5f;
//     float slip_err_r = fabsf(v_wheel_r) - fabsf(v_body) - 0.5f;

//     bool slip_l = (slip_err_l > 2.5f && fabsf(v_wheel_l) > 2.5f)
//                || (fabsf(alpha_l) > 250.0f && fabsf(v_wheel_l) > 2.0f);
//     bool slip_r = (slip_err_r > 2.5f && fabsf(v_wheel_r) > 2.5f)
//                || (fabsf(alpha_r) > 250.0f && fabsf(v_wheel_r) > 2.0f);

//     if (slip_l)
//         slip_factor_l *= 0.3f;
//     else
//         slip_factor_l += (1.0f - slip_factor_l) * 0.05f;

//     if (slip_r)
//         slip_factor_r *= 0.3f;
//     else
//         slip_factor_r += (1.0f - slip_factor_r) * 0.05f;

//     Float_Math_Constrain(&slip_factor_l, 0.0f, 1.0f);
//     Float_Math_Constrain(&slip_factor_r, 0.0f, 1.0f);
// }

void class_Chassis::Self_Observe()
{
    static float omega_l, omega_r = 0.0f;

    omega_l = mc_02->Rx_Data.Motor_wheel[0].omega - INS->gyro[0] + left_leg.Get_dtheta();
    v_wheel_l = -(0.06 * omega_l + left_leg.Get_L0() * left_leg.Get_dtheta() * cosf(left_leg.Get_theta()) + left_leg.Get_dL0() * sinf(left_leg.Get_theta()));
    omega_r = mc_02->Rx_Data.Motor_wheel[1].omega - INS->gyro[0] + right_leg.Get_dtheta();
    v_wheel_r = 0.06 * omega_r + right_leg.Get_L0() * right_leg.Get_dtheta() * cosf(right_leg.Get_theta()) + right_leg.Get_dL0() * sinf(right_leg.Get_theta());
    average_speed = (v_wheel_l + v_wheel_r) / 2.0f;

    Slip_Detection_And_Suppression();

    KF_Mode mode = KF_Mode::Normal;
    if (spin_mode)        mode = KF_Mode::Spin;
    else if (slip_active) mode = KF_Mode::Slip;

    kf.Set_Mode(mode);
    kf.Set_Z(mode == KF_Mode::Slip ? kf.x_hat(0) : average_speed,INS->MotionAccel_b[1]); //打滑时，使用KF估计速度x_hat(0)
    kf.Predict();
    kf.Update();

    Velocity_filter = kf.x_hat(0);
    X_filter = X_filter + Velocity_filter * (static_cast<float>(1)/1000.0f);

}
/**
 * @brief 4 点滑动均值滤波，对输入信号进行平滑处理
 */
static float Mean_Filtering4(float input, float *aver)
{
    aver[0] = aver[1]; aver[1] = aver[2]; aver[2] = aver[3]; aver[3] = input;
    return 0.25f * (aver[0] + aver[1] + aver[2] + aver[3]);
}

void class_Chassis::Slip_Detection_And_Suppression()
{
    // ---------- 1. 增量判据的两个量 ----------
    dv_wheel[0] = fabsf(Mean_Filtering4(v_wheel_l - last_v_wheel_l, wheel_aver[0]));
    dv_wheel[1] = fabsf(Mean_Filtering4(v_wheel_r - last_v_wheel_r, wheel_aver[1]));
    last_v_wheel_l = v_wheel_l;
    last_v_wheel_r = v_wheel_r;

    dv_acc = fabsf(INS->MotionAccel_b[1] * 0.001f);            // ★★ 车体加速度积分增量

    // ---------- 2. 外层门：确认"车确实在动/在转" ----------
    //const float v_body = Velocity_filter;   // 上一拍融合速度
    const float est_dyaw = (v_wheel_r - v_wheel_l) / WHEEL_BASE;
    const float db_yaw   = fabsf(est_dyaw - INS->gyro[2]);
    const float dvb_l    = fabsf(v_wheel_l) - fabsf(Velocity_filter);
    const float dvb_r    = fabsf(v_wheel_r) - fabsf(Velocity_filter);

    const bool outer_gate = (db_yaw >= SLIP_DBY_THRESHOLD ||
                             dvb_l  >= SLIP_DVB_THRESHOLD ||
                             dvb_r  >= SLIP_DVB_THRESHOLD);

    if (outer_gate && !spin_mode)           // 小陀螺时关闭打滑检测
    {
        // ---------- 3. 内层判据 + 迟滞锁存 ----------
        for (int i = 0; i < 2; i++)
        {
            const float vi = (i == 0) ? v_wheel_l : v_wheel_r;

            if (dv_wheel[i] - dv_acc >= SLIP_DVW_THRESHOLD && !slip_flag[i])
            {
                slip_flag[i]  = true;                    // 触发
                f_dv_wheel[i] = fabsf(vi);               // 锁存打滑瞬间轮速
            }
            else if (fabsf(vi) <= f_dv_wheel[i] && slip_flag[i])
            {
                slip_flag[i]  = false;                   // 迟滞退出
                f_dv_wheel[i] = 0.0f;
            }
            // else: 保持
        }
    }
    else
    {
        slip_flag[0] = slip_flag[1] = false;             // 外层门不成立 → 强制清零
        f_dv_wheel[0] = f_dv_wheel[1] = 0.0f;            // ★ 显式复位，避免未初始化比较
    }

    slip_active = (slip_flag[0] || slip_flag[1]);        // 任一轮 → 全局 Q/R 切换

    // ---------- 4. 输出抑制因子（打滑硬降，恢复一阶爬回） ----------
    for (int i = 0; i < 2; i++)
    {
        float *f = (i == 0) ? &slip_factor_l : &slip_factor_r;
        if (slip_flag[i]) *f *= SLIP_TORQUE_FACTOR;
        else              *f += (1.0f - *f) * SLIP_RECOVER_RATE;
        Float_Math_Constrain(f, 0.35f, 1.0f);
    }
}


void class_Chassis::Ground_Detection()
{
    above_flag_l = left_leg.ground_detection_L();
    above_flag_r = right_leg.ground_detection_R();
}



void class_Chassis::MPC_Body_Calc()
{
    if (fabs(Target_height - last_Target_height) > 0.001f) 
    {
        height_planner.MoveTo(Now_height, Target_height, Now_dheight, Target_dheight, 0.05f); // 0.5����ɶ���?
        last_Target_height = Target_height;
    }

    if (height_planner.is_planning) 
    {
        height_planner.Step(dt, current_ref_height, current_ref_dheight);
    } else 
    {
        current_ref_height = Target_height;
        current_ref_dheight = 0.0f;
    }

    MPC_body.x_ref.setZero();

    if(chassis_fsm_mode == ABOVE_GROUND)
    {
        MPC_body.x(0) = 0;
        MPC_body.x(1) = 0;
        MPC_body.x(2) = Now_height;
        MPC_body.x(3) = Now_dheight;

        Target_Roll = 0;
    }
    else
    {
        MPC_body.x(0) = INS->roll;
        MPC_body.x(1) = INS->gyro[1];
        MPC_body.x(2) = Now_height;
        MPC_body.x(3) = Now_dheight;
    }

    for (int i = 0; i < N+1; i++)
    {
        MPC_body.x_ref(i*4) = Target_Roll;
        MPC_body.x_ref(i*4+1) = 0.0;
        MPC_body.x_ref(i*4+2) = Target_height;
        MPC_body.x_ref(i*4+3) = Target_dheight;
    }

    MPC_body.MPC_Calculate();
    

}

void class_Chassis::LQR_Calc()
{
    for(int i=0;i<=3;i++)
    {
        if (chassis_fsm_mode == ABOVE_GROUND)
        {
            if (i>1)
            {
                T[i] = err[4]*Fitting_K[i][4]*2.0 + err[5]*Fitting_K[i][5]*0.6
                        + err[6]*Fitting_K[i][6]*2.0 + err[7]*Fitting_K[i][7]*0.6;
            }
            else
            {
                T[i] = 0;
            }
        }
        else
        {
            if (spin_mode)
            {
            //     T[i] = err[0] * 0                      + err[1] * Fitting_K[i][1]
            //          - err[2] * 0                      + err[3] * Fitting_K[i][3]
            //          + err[4] * Fitting_K[i][4] * 1.2f + err[5] * Fitting_K[i][5] * 0.96f
            //          + err[6] * Fitting_K[i][6] * 1.2f + err[7] * Fitting_K[i][7] * 0.96f
            //          + err[8] * Fitting_K[i][8] * 3.0f + err[9] * Fitting_K[i][9] * 1.4f;
                   T[i] = err[0] * 0                      + err[1] * Fitting_K[i][1]
                        - err[2] * 0                      + err[3] * Fitting_K[i][3]
                        + err[4] * Fitting_K[i][4] * 0.9f + err[5] * Fitting_K[i][5] * 0.95f
                        + err[6] * Fitting_K[i][6] * 0.9f + err[7] * Fitting_K[i][7] * 0.95f
                        + err[8] * Fitting_K[i][8] * 3.0f + err[9] * Fitting_K[i][9] * 1.4f;
          
            }
            else
            {
                // T[i] = err[0] * Fitting_K[i][0] + Max_Output(err[1] * Fitting_K[i][1], 16)
                //      - Max_Output(err[2] * Fitting_K[i][2], 15.0f) + err[3] * Fitting_K[i][3]
                //      + err[4] * Fitting_K[i][4] + err[5] * Fitting_K[i][5] * 0.94f
                //      + err[6] * Fitting_K[i][6] + err[7] * Fitting_K[i][7] * 0.94f
                //      + err[8] * Fitting_K[i][8] + err[9] * Fitting_K[i][9];
                T[i] = err[0] * Fitting_K[i][0] + Max_Output(err[1] * Fitting_K[i][1]*0.9f, 16)
                     - Max_Output(err[2] * Fitting_K[i][2], 15.0f) + err[3] * Fitting_K[i][3]*0.9f
                     + err[4] * Fitting_K[i][4] + err[5] * Fitting_K[i][5] * 0.9f
                     + err[6] * Fitting_K[i][6] + err[7] * Fitting_K[i][7] * 0.9f
                     + err[8] * Fitting_K[i][8] + err[9] * Fitting_K[i][9]*0.9f;

                // ★ 打滑侧压制速度环：err[1] 是"轮速发散"进入控制器的唯一通道
                // const float w_l = slip_flag[0] ? 0.0f : 1.0f;
                // const float w_r = slip_flag[1] ? 0.0f : 1.0f;
                // const float err1_l = err[1] * w_l;
                // const float err1_r = err[1] * w_r;

                // const float e1 = (i <= 1) ? ((i == 0) ? err1_l : err1_r) : err[1];

                // T[i] = err[0] * Fitting_K[i][0] + Max_Output(e1 * Fitting_K[i][1] * 0.9f, 16)
                //      - Max_Output(err[2] * Fitting_K[i][2], 15.0f) + err[3] * Fitting_K[i][3] * 0.9f
                //      + err[4] * Fitting_K[i][4] + err[5] * Fitting_K[i][5] * 0.9f
                //      + err[6] * Fitting_K[i][6] + err[7] * Fitting_K[i][7] * 0.9f
                //      + err[8] * Fitting_K[i][8] + err[9] * Fitting_K[i][9] * 0.9f;
            }
        }
    }
    // 位置：必须在 T[0]/T[1] 算完之后、Follow_Gimbal_Control 读取之前
    if (!spin_mode && chassis_fsm_mode != ABOVE_GROUND)
    {
        T[0] *= slip_factor_l;
        T[1] *= slip_factor_r;
    }

    // ★ 保存 LQR 本拍结果（在打滑缩放之后），供与 MPC 对比
    for (int i = 0; i < 4; ++i) { T_lqr[i] = T[i]; }

    // ===== 状态复位：打滑 → 清位置积分，防止 X_filter 继续发散 =====
    if (slip_active)
    {
        X_filter = 0.0f;
        Target_X = X_filter;
    }
    // ====== 调试：保存 LQR 各通道分量，以左轮 i=0 为例 ======
    lqr_term[0] = err[0] * Fitting_K[0][0];
    lqr_term[1] = Max_Output(err[1] * Fitting_K[0][1], 16);
    lqr_term[2] = -Max_Output(err[2] * Fitting_K[0][2], 15.0f);
    lqr_term[3] = err[3] * Fitting_K[0][3];
    lqr_term[4] = err[4] * Fitting_K[0][4];
    lqr_term[5] = err[5] * Fitting_K[0][5];
    lqr_term[6] = err[6] * Fitting_K[0][6];
    lqr_term[7] = err[7] * Fitting_K[0][7];
    lqr_term[8] = err[8] * Fitting_K[0][8];
    lqr_term[9] = err[9] * Fitting_K[0][9];
}

/**
 * @brief 10 状态 MPC 计算（4 ms 任务调用，输出 T_mpc[4] / mpc_ok）
 *
 * 为什么锁要在函数内部：
 *   TIM_4ms 与 TIM_1ms/2ms 是两个线程，共用 mc_02->rx_mutex。
 *   Run() 内部可能触发 0.3~0.8 ms 的模型重建，如果像其他任务那样
 *   在回调最外层加锁，1 kHz 任务会被堵这么久。所以这里只锁两次
 *   「数据拷贝」的瞬间，求解全程不持锁。
 */
void class_Chassis::MPC_Robot_Calc()
{
    Robot_MPC::VecX e;
    Robot_MPC::VecU u_last;
    double Ll, Lr;

    // ---------- 锁内：只拷贝数据 ----------
    {
        std::lock_guard<std::mutex> lock(mc_02->rx_mutex);
        for (int i = 0; i < 10; ++i) e(i) = static_cast<double>(err[i]);
        Ll = static_cast<double>(left_leg.Get_L0());
        Lr = static_cast<double>(right_leg.Get_L0());
        // 历史取「上一拍实际生效的通道命令」，不是 T_mpc：MPC 在 LQR 接管/打滑缩放
        // 期间照样在算，T_mpc 与真正施加到电机的力矩不是一回事。
        for (int i = 0; i < 4; ++i) u_last(i) = static_cast<double>(mpc_u_hist[i]);
    }   // ← 自动解锁

    // ---------- 锁外：求解（可能重建模型）----------
    Robot_MPC::VecU u;
    mpc_robot->SetLastControl(u_last);
    bool ok = mpc_robot->Run(e, Ll, Lr, u);
    if (!ok) u = mpc_robot->Fallback(e);      // QP 失败 -> LQR 兜底（已含等比限幅）

    // ---------- 锁内：只写结果 ----------
    {
        std::lock_guard<std::mutex> lock(mc_02->rx_mutex);
        for (int i = 0; i < 4; ++i) T_mpc[i] = static_cast<float>(u(i));
        mpc_ok = ok;
        mpc_stamp = std::chrono::steady_clock::now();   // 1 ms 侧据此判断输出是否过期
    }
}
