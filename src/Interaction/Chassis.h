/**
  ******************************************************************************
  * @file           : Chassis.h
  * @author         : Chen Haoran
  * @brief          : None
  * @attention      : None
  * @date           : 2025/11/4
  ******************************************************************************
  */
#ifndef WHEEL_LEG_SYS_CHASSIS_H
#define WHEEL_LEG_SYS_CHASSIS_H
/* Includes ------------------------------------------------------------------*/
#include "dm_mc02.h"
#include "INS.h"
#include "MPC_Control.h"
#include "Algorithm/VMC/VMC&LQR_Calc.h"
#include "Algorithm/PID/PID_Control.h"
#include "Algorithm/Slope/slope.h"
#include "Algorithm/LESO/LESO.h"
#include "Robot_MPC.h"          // 10 状态线性 MPC（注意：必须放在 #define N 之前）
#include <memory>
#include <chrono>               // MPC 输出新鲜度时间戳
/* Define --------------------------------------------------------------------*/

#define spring_forceL 0
#define spring_forceR 0

#define N 15    //预测步数
#define WHEEL_BASE          0.25f    
#define SLIP_DVW_THRESHOLD  0.06f    
#define SLIP_DVB_THRESHOLD  1.20f    
#define SLIP_DBY_THRESHOLD  0.90f    
#define SLIP_TORQUE_FACTOR  0.60f    
#define SLIP_RECOVER_RATE   0.05f    

/* Enum ----------------------------------------------------------------------*/
typedef enum
{
    CHASSIS_ZERO_FORCE, 
    CHASSIS_NO_MOVE, 
    CHASSIS_INFANTRY_FOLLOW_GIMBAL_YAW, 
    CHASSIS_NO_FOLLOW_YAW, 
    CHASSIS_INFANTRY_SPIN, 
    CHASSIS_CHECK_IN,
} Enum_chassis_mode;

typedef enum
{
    NORMAL, 
    OVER_TURN, 
    OVER_TURN_UPSIDE1, 
    OVER_TURN_UPSIDE2, 
    OVER_TURNING1, 
    OVER_TURNING2, 
    ABOVE_GROUND, 
} Enum_chassis_fsm_mode;

typedef enum
{
    APPROACH = 0,
    GET_POSITION,
    UP_STARIS_BEND_LEG,
    OVER
} Enum_up_staris_fsm_mode;

/* Variable && Struct --------------------------------------------------------*/

/* Function Declaration ------------------------------------------------------*/

/* Function ------------------------------------------------------------------*/

class class_Chassis
{
public:
    class_INS* INS{};
    mc_02_serial* mc_02{};

    class_vmc_leg left_leg;
    class_vmc_leg right_leg;

    TrajectoryPlanner height_planner;

    Class_PID PID_Omega;
    Class_PID PID_tp;
    Class_PID PID_tp_omega;

    MPC_Controller MPC_body = MPC_Controller(4, 2, 0, N);

    // 10 状态全状态线性 MPC：替代 LQR 输出 T[0..3]（轮矩L/R + 髋矩L/R）
    std::unique_ptr<Robot_MPC> mpc_robot;
    bool mpc_enable = true;      // ★ 调试开关：false = 完全退回原 LQR
    float mpc_timeout = 0.008f;  // 输出新鲜度上限 [s]：超过 8 ms（2 个 MPC 周期）未更新就退回 LQR

    Enum_chassis_fsm_mode chassis_fsm_mode;
    Enum_up_staris_fsm_mode chassis_up_fsm_mode;


    float current_ref_height = 0;
    float current_ref_dheight = 0;

    void Init();
    void feedback_update();
    void Zero_Force_Control();
    void Follow_Gimbal_Control();
    void Check_in_Control();
    void Control_leg();
    void Control_leg_left(float target);
    void Control_leg_right(float target);
    void UP_Staris_Control_leg();
    void Self_Observe();
    void Ground_Detection();
    void Slip_Detection_And_Suppression();
    void MPC_Body_Calc();
    void LQR_Calc();
    void MPC_Robot_Calc();          // 4ms 任务调用：10 状态 MPC 求解一次

    inline void Set_Target_X(float __Target_X)                         { Target_X = __Target_X; }
    inline void Set_Target_Velocity(float __Target_Velocity)           { Target_Velocity = __Target_Velocity; }
    inline void Set_Target_height(float __Target_height)               { Target_height = __Target_height; }
    inline void Set_Target_dheight(float __Target_dheight)             { Target_dheight = __Target_dheight; }
    inline void Set_Target_left_f0(float __Target_left_f0)             { Target_left_f0 = __Target_left_f0; }
    inline void Set_Target_right_f0(float __Target_right_f0)           { Target_right_f0 = __Target_right_f0; }
    inline void Set_Target_Yaw(float __Target_Yaw)                     { Target_Yaw = __Target_Yaw; }
    inline void Set_Yaw_Angle(float __Yaw_Angle)                       { Yaw_Angle = __Yaw_Angle; }
    inline void Set_Target_Omega(float __Target_Omega)                 { Target_Omega = __Target_Omega; }
    inline void Set_Target_Roll(float __Target_Roll)                   { Target_Roll = __Target_Roll; }
    inline void Set_Target_Theta(float __Target_Theta)                 { Target_Theta = __Target_Theta; }
    inline void Set_Target_Pitch(float __Target_Pitch)                 { Target_Pitch = __Target_Pitch; }
    inline void Set_Spin_Mode(bool __spin_mode)                        { spin_mode = __spin_mode; }
    inline void Set_last_Target_leg(float __last_Target_leg)           { last_Target_leg = __last_Target_leg; }
    inline void Set_Velocity_filter(float __Velocity_filter)           { Velocity_filter = __Velocity_filter; }
    inline void Set_T_L(float __T_L)                                   { T_L = __T_L; }
    inline void Set_T_R(float __T_R)                                   { T_R = __T_R; }
    inline void Set_above_flag_l(bool __above_flag_l)                  { above_flag_l = __above_flag_l; }
    inline void Set_above_flag_r(bool __above_flag_r)                  { above_flag_r = __above_flag_r; }
    inline void Set_Jump_force(float __Jump_force)                     { Jump_force = __Jump_force; }
    inline void Set_FSM_mod(Enum_chassis_fsm_mode __mod)               { chassis_fsm_mode = __mod; }
    inline void Set_UP_FSM_mod(Enum_up_staris_fsm_mode __mod)          { chassis_up_fsm_mode = __mod; }
    inline float Get_Velocity_filter()                                 { return Velocity_filter; }
    inline float Get_X_filter()                                        { return X_filter; }
    inline float Get_T_L()                                             { return T_L; }
    inline float Get_Tp_L()                                            { return Tp_L; }
    inline float Get_T_R()                                             { return T_R; }
    inline float Get_Tp_R()                                            { return Tp_R; }
    inline bool Get_above_flag_l()                                     { return above_flag_l; }
    inline bool Get_above_flag_r()                                     { return above_flag_r; }
    inline float Get_Heigh()                                           { return Now_height; }
    inline float Get_average_speed()                                   { return average_speed; }
    inline bool  Get_slip_active()                                     { return slip_active; }
    inline float Get_v_filter()                                        { return kf.x_hat(0); }
    inline float Get_a_filter()                                        { return kf.x_hat(1); } 

    /**
     * @brief MPC 输出是否仍可用：求解成功 **且** 距今不超过 mpc_timeout。
     */
    inline bool MPC_Output_Fresh() const
    {
        if (!mpc_ok) { return false; }
        const std::chrono::duration<double> d =
                std::chrono::steady_clock::now() - mpc_stamp;
        return d.count() <= static_cast<double>(mpc_timeout);
    }

    float ll,rr = 0;
    float lqr_term[10] = {};
    float T_lqr[4] = {};          // LQR 本拍四通道结果（已含打滑缩放），仅调试用
    inline float Get_T_mpc(int i) const { return T_mpc[i]; }  
    inline bool  Get_mpc_ok()   const { return mpc_ok; }       // QP 本拍是否解成功
    inline bool  Get_mpc_used() const { return mpc_applied; }  // 本拍是否真的用了 MPC

private:
    KalmanFilter kf = KalmanFilter(2,2,0,nullptr,nullptr,nullptr,nullptr,nullptr);

    float Fitting_K[4][10] = {};
    float err[10] = {};

    float spring_forcel = 0;
    float spring_forcer = 0;

    float Target_X = 0;
    float Target_Velocity = 0;
    float Target_height = 0;
    float Target_dheight = 0;
    float Target_left_f0 = 0;
    float Target_right_f0 = 0;
    float Target_Yaw = 0;
    float Target_Omega = 0;
    float Target_Roll = 0;
    float Target_Theta = 0;
    float Target_Pitch = 0.0f;
    float last_Target_leg = 0;
    float last_Target_X = 0;
    float Yaw_Angle = 0;

    float roll = 0;
    float pitch = 0;
    float yaw = 0;

    float Now_height = 0;
    float Now_dheight = 0;

    float Velocity_filter = 0; 
    float X_filter = 0; 

    float total_yaw = 0;
    float theta_err = 0; 
    float dtheta_err = 0; 

    float Yaw_turn_T = 0;
    float roll_f0 = 0; 
    float leg_tp = 0; 

    float T[4] = {};
    float T_mpc[4] = {};         // MPC 输出：[T_wl, T_wr, T_bl, T_br]
    bool  mpc_ok = false;        // MPC 本周期的结果是否可用
    bool  mpc_applied = false;   // 本拍是否真的采用 MPC 结果
    std::chrono::steady_clock::time_point mpc_stamp{};   // 最后一次 MPC 求解完成的时刻
    float mpc_u_hist[4] = {};    // 上一拍实际生效的通道命令（MPC 决策变量量纲）

    // 记录本拍实际生效的通道命令，供下一拍 Δu 约束当起点
    inline void Record_MPC_History(float wl, float wr, float hl, float hr)
    {
        mpc_u_hist[0] = wl; mpc_u_hist[1] = wr;
        mpc_u_hist[2] = hl; mpc_u_hist[3] = hr;
    }
    float T_L = 0, T_R = 0;
    float Tp_L = 0, Tp_R = 0;

    float M = 50;
    float Jump_force = 0;
    float F_mc = 0;

    bool above_flag_l = false;
    bool above_flag_r = false;
    bool stop_flag = true;
    bool last_stop_flag = true;
    bool spin_mode = false;

    float last_Target_height = 0.0f;


    float dt = 0.002;
    float dt_mpc = 0.002;

    float last_wheel_omega_l = 0.0f;
    float last_wheel_omega_r = 0.0f;
    float slip_factor_l = 1.0f;
    float slip_factor_r = 1.0f;

    float v_wheel_l = 0, v_wheel_r = 0;        
    float last_v_wheel_l = 0, last_v_wheel_r = 0;
    float dv_wheel[2] = {0, 0};                
    float wheel_aver[2][4] = {};              
    float f_dv_wheel[2] = {0, 0};            
    float dv_acc = 0;                         
    float average_speed = 0.0f;

    int   stall_cnt  = 0;              
    bool  stalling   = false;
    bool slip_flag[2] = {false, false};        
    bool slip_active = false;                 
    bool last_slip_flag[2] = {false, false};   
};

float Float_Math_Constrain(float *x, float Min, float Max);



#endif //WHEEL_LEG_SYS_CHASSIS_H