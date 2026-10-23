/**
  ******************************************************************************
  * @file           : robot.cpp
  * @author         : Chen Haoran
  * @brief          : 机器人顶层调度实现。负责初始化硬件与 FSM、解析遥控输入、
  *                    按底盘状态机分发控制，并处理跳跃、上台阶和翻车自救；
  *                    最终通过 Chassis 输出轮力矩与腿 VMC 指令。
  * @attention      : None
  * @date           : 2025/10/25
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "robot.h"
#include "user_lib.h"
/* Define --------------------------------------------------------------------*/

/* Variable && Struct --------------------------------------------------------*/

/* Function Declaration ------------------------------------------------------*/

/* Function ------------------------------------------------------------------*/


void Class_FSM_Chassis::TIM_1ms_Calculate_PeriodElapsedCallback()
{
    Status[Now_Status_Serial].Count_Time++;

    switch (Now_Status_Serial)
    {
    case NORMAL:
        if ((robot->INS.pitch < M_PI / 3.0 && robot->INS.pitch > - M_PI / 3.0) &&
            ((robot->Chassis.left_leg.Get_theta() < -0.5 || robot->Chassis.left_leg.Get_theta() > 0.9) ||
            (robot->Chassis.right_leg.Get_theta() < -0.5 || robot->Chassis.right_leg.Get_theta() > 0.9)))
        {
            Set_Status(OVER_TURN);
            robot->Chassis.Set_FSM_mod(OVER_TURN);
        }
        else if(robot->INS.pitch > 1.0)
        {
            Set_Status(OVER_TURN_UPSIDE1);
            robot->Chassis.Set_FSM_mod(OVER_TURN_UPSIDE1);
        }
        else if(robot->INS.pitch < -1.1)
        {
            Set_Status(OVER_TURN_UPSIDE2);
            robot->Chassis.Set_FSM_mod(OVER_TURN_UPSIDE2);
        }
        else if ((robot->Chassis.Get_above_flag_l() == 1 && robot->Chassis.Get_above_flag_r() == 1) &&
            (robot->RC.axis[2] <= 0.03f && robot->RC.axis[2] >= -0.03f) && robot->RC.press_c == 0)
        {
            Set_Status(ABOVE_GROUND);
            robot->Chassis.Set_FSM_mod(ABOVE_GROUND);
        }
        break;
    case OVER_TURN_UPSIDE1:
        if(robot->INS.pitch < 0.08 && robot->INS.pitch > -0.08 && Status[Now_Status_Serial].Count_Time > 500)
        {
            Set_Status(OVER_TURN);
            robot->Chassis.Set_FSM_mod(OVER_TURN);
        }
    break;
    case OVER_TURN_UPSIDE2:
        if(robot->INS.pitch < 0.08 && robot->INS.pitch > -0.08 && Status[Now_Status_Serial].Count_Time > 500)
        {
            Set_Status(OVER_TURN);
            robot->Chassis.Set_FSM_mod(OVER_TURN);
        }
    break;
    case OVER_TURN:
        // if (((robot->Chassis.left_leg.Get_theta() >= 1.2) || (robot->Chassis.right_leg.Get_theta() >= 1.2))&&
        // (fabs(robot->Chassis.left_leg.Get_theta() - robot->Chassis.right_leg.Get_theta()) < 0.3f)&&
        //     (robot->INS.pitch >= -M_PI/4.0 && robot->INS.pitch <= M_PI/4.0))
        // {
        //     Set_Status(OVER_TURNING1);
        //     robot->Chassis.Set_FSM_mod(OVER_TURNING1);
        // }
        if(robot->INS.pitch >= -M_PI/4.0 && robot->INS.pitch <= M_PI/4.0)
        {
            if(fabs(robot->Chassis.left_leg.Get_theta() - robot->Chassis.right_leg.Get_theta()) < 0.5f)
            {
                if((robot->Chassis.left_leg.Get_theta() >= 0.7) || (robot->Chassis.right_leg.Get_theta() >= 0.7))
                {
                    Set_Status(OVER_TURNING1);
                    robot->Chassis.Set_FSM_mod(OVER_TURNING1);
                }
            }
            else
            {
                if((robot->Chassis.left_leg.Get_theta() >= 0.9) && (robot->Chassis.right_leg.Get_theta() <= 0.4 && robot->Chassis.right_leg.Get_theta() >= -0.8)||
                (robot->Chassis.right_leg.Get_theta() >= 0.9) && (robot->Chassis.left_leg.Get_theta() <= 0.4 && robot->Chassis.left_leg.Get_theta() >= -0.8))
                {
                    Set_Status(OVER_TURNING1);
                    robot->Chassis.Set_FSM_mod(OVER_TURNING1);
                }
            }

        }
        // else if(((robot->Chassis.left_leg.Get_theta() >= -0.8 && robot->Chassis.left_leg.Get_theta() <= 0.4)&&
        //         (robot->Chassis.right_leg.Get_theta() >= -0.8 && robot->Chassis.right_leg.Get_theta() <= 0.4))&&
        //     (robot->INS.pitch >= -M_PI/6.0 && robot->INS.pitch <= M_PI/6.0))
        // {
        //     Set_Status(OVER_TURNING2);
        //     robot->Chassis.Set_FSM_mod(OVER_TURNING2);
        // }
        else if(robot->INS.pitch > 1.0)
        {
            Set_Status(OVER_TURN_UPSIDE1);
            robot->Chassis.Set_FSM_mod(OVER_TURN_UPSIDE1);
        }
        else if(robot->INS.pitch < -1.1)
        {
            Set_Status(OVER_TURN_UPSIDE2);
            robot->Chassis.Set_FSM_mod(OVER_TURN_UPSIDE2);
        }
        break;
    case OVER_TURNING1:
        if ((robot->INS.pitch <= M_PI / 4.0 && robot->INS.pitch >= -M_PI / 4.0) &&
            (robot->Chassis.left_leg.Get_theta() <= 0.2 && robot->Chassis.left_leg.Get_theta() >= -0.8)&&
            (robot->Chassis.right_leg.Get_theta() <= 0.2 && robot->Chassis.right_leg.Get_theta() >= -0.8)&&
            (robot->Chassis.left_leg.Get_L0() < 0.2 && robot->Chassis.right_leg.Get_L0() < 0.2))
        {
            Set_Status(NORMAL);
            robot->Chassis.Set_FSM_mod(NORMAL);
        }
        else if(
                 ((robot->Chassis.left_leg.Get_theta() <= -1.0 && robot->Chassis.left_leg.Get_theta() >= -4.0 )
                 ||  (robot->Chassis.right_leg.Get_theta() <= -1.0 & robot->Chassis.right_leg.Get_theta() >= -4.0)) 
              || (robot->INS.pitch > M_PI / 3.0 || robot->INS.pitch < - M_PI / 3.0)
               )
        {
            Set_Status(OVER_TURN);
            robot->Chassis.Set_FSM_mod(OVER_TURN);
        }
        break;
    case OVER_TURNING2:
        
            Set_Status(OVER_TURN);
            robot->Chassis.Set_FSM_mod(OVER_TURN); 
        
        break;
    case ABOVE_GROUND:
        if (robot->Chassis.Get_above_flag_l() == 0 || robot->Chassis.Get_above_flag_r() == 0)
        {
            Set_Status(NORMAL);
            robot->Chassis.Set_FSM_mod(NORMAL);
        }
        break;
    default: ;
    }
}

void Class_FSM_Jump::TIM_1ms_Calculate_PeriodElapsedCallback()
{
    Status[Now_Status_Serial].Count_Time++;
    static int mod = 0;

    // �Լ����ű�д״̬ת�ƺ���
    switch (Now_Status_Serial)
    {
    case BEND_LEG:
        robot->jump_flag = 1;
        robot->Target_height = 0.12;
        robot->Chassis.Set_Target_dheight(0.0);
        robot->Chassis.Set_above_flag_l(0);
        robot->Chassis.Set_above_flag_r(0);

        if(robot->Chassis.left_leg.Get_L0() < 0.19 && robot->Chassis.right_leg.Get_L0() < 0.19)
        {
            mod++;   
        }
        if(mod > 5 && switch_is_up(robot->mc02_SerialPort.Rx_Data.RC_ctrl.sw[4]))
        {
            mod = 0;
            Set_Status(STRETCH_LEG);
        }
        break;
    case STRETCH_LEG:
        robot->Target_height = 0.34;
        robot->Chassis.Set_Jump_force(50.0);
        robot->Chassis.Set_Target_dheight(5.0);
        robot->Chassis.Set_above_flag_l(0);
        robot->Chassis.Set_above_flag_r(0);

        if(robot->Chassis.left_leg.Get_L0() > 0.3 && robot->Chassis.right_leg.Get_L0() > 0.3)
        {
            robot->Chassis.Set_Jump_force(0.0);
            robot->Chassis.Set_above_flag_l(1);
            robot->Chassis.Set_above_flag_r(1);
            mod++;   
        }
        if(mod > 5)
        {
            mod = 0;
            Set_Status(BEND_LEG_AIR);
        }
        break;
    case BEND_LEG_AIR:
        robot->Target_height = 0.1;
        robot->Chassis.Set_Jump_force(-100.0);
        robot->Chassis.Set_Target_dheight(0.0);
        robot->Chassis.Set_above_flag_l(1);
        robot->Chassis.Set_above_flag_r(1);

        if(robot->Chassis.left_leg.Get_L0() > 0.09 && robot->Chassis.left_leg.Get_L0() < 0.2 && 
           robot->Chassis.right_leg.Get_L0() > 0.09 && robot->Chassis.right_leg.Get_L0() < 0.2)
        {
            mod++;   
        }
        if(mod > 60)
        {
            mod = 0;
            Set_Status(STRETCH_LEG_AIR);
        }
        break;
    case STRETCH_LEG_AIR:
        robot->Target_height = 0.27;
        robot->jump_flag = 0;
        robot->Chassis.Set_Jump_force(30.0);
        robot->Chassis.Set_above_flag_l(1);
        robot->Chassis.Set_above_flag_r(1);
        
        if(robot->Chassis.left_leg.Get_L0() > 0.1 && robot->Chassis.left_leg.Get_L0() < 0.19 &&
           robot->Chassis.right_leg.Get_L0() > 0.1 && robot->Chassis.right_leg.Get_L0() < 0.19)
        {
            mod++;   
        }
        if(mod > 20)
        {
        robot->Target_height = 0.13;

        mod = 0;
        robot->Chassis.Set_above_flag_l(0);
        robot->Chassis.Set_above_flag_r(0);
        robot->Chassis.Set_Jump_force(0.0);
            
            Set_Status(STOP);
        }

        break;

    case STOP:
        robot->Target_height = 0.13;
        robot->jump_flag = 0;
        robot->Chassis.Set_Jump_force(0.0);
        robot->Chassis.Set_above_flag_l(0);
        robot->Chassis.Set_above_flag_r(0);
        

        break;

    default: ;
    }
}

void Class_FSM_Up_staris::TIM_1ms_Calculate_PeriodElapsedCallback()
{
    Status[Now_Status_Serial].Count_Time++;

    // �Լ����ű�д״̬ת�ƺ���
    switch (Now_Status_Serial)
    {
    case APPROACH:
        if(robot->Chassis.left_leg.Get_Tp() > 6.0 || robot->Chassis.right_leg.Get_Tp() > 6.0)
        {
            Set_Status(GET_POSITION);
            robot->Chassis.Set_UP_FSM_mod(GET_POSITION);
        }
        break;
    case GET_POSITION:
        if(Status[Now_Status_Serial].Count_Time > 50)
        {
            Set_Status(UP_STARIS_BEND_LEG);
            robot->Chassis.Set_UP_FSM_mod(UP_STARIS_BEND_LEG);

        }
        else if(robot->Chassis.left_leg.Get_theta() <= 0.4 || robot->Chassis.right_leg.Get_theta() <= 0.4)
        {
            Set_Status(APPROACH);
            robot->Chassis.Set_UP_FSM_mod(APPROACH);

        }
        break;

    case UP_STARIS_BEND_LEG:

        if(Status[Now_Status_Serial].Count_Time > 200)
        {
            Set_Status(OVER);
            robot->Chassis.Set_UP_FSM_mod(OVER);
            
        }
        break;
    case OVER:
        robot->Target_height = 0.10;
        // Set_Status(APPROACH);

        break;

    default: ;
    }
}


void Robot::Init()
{
    mc02_SerialPort.Init();

    GamePad.Open();

    // Offset.initialise(100);
    // Ahrs.enable_gradient_descent(true);


    //Vofa_TCP.Init("127.0.0.1", 1346);

    FSM_Chassis.robot = this;
    FSM_Chassis.Init(6, NORMAL);

    FSM_Jump.robot = this;
    FSM_Jump.Init(5, BEND_LEG);

    FSM_Up_staris.robot = this;
    FSM_Up_staris.Init(4, APPROACH);

    Chassis.INS = &INS;
    Chassis.left_leg.INS = &INS;
    Chassis.right_leg.INS = &INS;
    Chassis.mc_02 = &mc02_SerialPort;
    Chassis.Init();

    Slope_X.Init(1.0f/5000.0f, 1.0f/600.0f, Slope_First_REAL);
    Slope_X.Set_Max_Target_Up_Delta(2.5f);
    Slope_X.Set_Max_Target_Down_Delta(2.5f);
    Slope_X.Set_Adaptive_Gain(0.20f);
    Slope_X.Set_Max_Adaptive_Ratio(4.0f);

    Slope_Y.Init(1.0f / 1000.0f, 1.0f / 1000.0f, Slope_First_REAL);
    Slope_Y.Set_Max_Target_Up_Delta(2.5f);
    Slope_Y.Set_Max_Target_Down_Delta(2.5f);
    Slope_Y.Set_Adaptive_Gain(0.20f);
    Slope_Y.Set_Max_Adaptive_Ratio(4.0f);

    Slope_Leg_l.Init(0.3f/1000.0f, 0.3f/1000.0f, Slope_First_REAL);
    Slope_Leg_l.Set_Max_Target_Up_Delta(0.000000001f);
    Slope_Leg_l.Set_Max_Target_Down_Delta(0.0000001f);
    Slope_Leg_l.Set_Adaptive_Gain(0.20f);
    Slope_Leg_l.Set_Max_Adaptive_Ratio(4.0f);

    Slope_Leg_r.Init(0.3f/1000.0f, 0.3f/1000.0f, Slope_First_REAL);
    Slope_Leg_r.Set_Max_Target_Up_Delta(0.000000001f);
    Slope_Leg_r.Set_Max_Target_Down_Delta(0.0000001f);
    Slope_Leg_r.Set_Adaptive_Gain(0.20f);
    Slope_Leg_r.Set_Max_Adaptive_Ratio(4.0f);

    Slope_Omega.Init(1.0f / 100.0f, 1.0f / 100.0f, Slope_First_REAL);
    Slope_Omega.Set_Max_Target_Up_Delta(10.5f);
    Slope_Omega.Set_Max_Target_Down_Delta(5.5f);
    Slope_Omega.Set_Adaptive_Gain(0.20f);
    Slope_Omega.Set_Max_Adaptive_Ratio(4.0f);

    PID_Follow_yaw.Init(10.0f, 0.1f, 0.01f, 0.0f, 0.3f, 4.0f, 0.002f, 0.0f, 0.0f, 0.0f, 0.0f, PID_D_First_DISABLE);
}

// ===================================================================
// 函数: Chassis_Control()
// 功能: 底盘主控制函数，根据遥控器模式和底盘状态机，
//       计算并下发底盘运动目标值（速度、角度、高度等）
// 调用周期: 1ms (由定时中断触发)
// ===================================================================
void Robot::Chassis_Control()
{
    // 超级电容加速模式下的最大速度/角速度临时变量
    float tmp_chassis_velocity_max, tmp_chassis_omega_max;
    // mod: 空中状态超时计数器（用于跳跃空中阶段超时保护）
    static int mod = 0;

    // --- 1. 超级电容加速模式：提高速度上限和斜坡增速率 ---
    if (Supercap_Accelerate_Status == true)
    {
        tmp_chassis_velocity_max = 4.0f;
        tmp_chassis_omega_max = 6.0f * std::numbers::pi;
        Slope_X.Set_Increase_Value(5.0f / 1000.0f);
    }
    else
    {
        tmp_chassis_velocity_max = 3.0f;
        tmp_chassis_omega_max = 4.0f * std::numbers::pi;
        Slope_X.Set_Increase_Value(3.0f / 1000.0f);
    }

    // --- 2. 根据遥控器开关选择底盘控制模式 ---
    // set chassis control mod
    if (RC.zero_force)
    {
        chassis_mode = CHASSIS_ZERO_FORCE;                 // 零力模式（电机无力输出）
    }
    else if (RC.competition)
    {
        chassis_mode = CHASSIS_INFANTRY_FOLLOW_GIMBAL_YAW; // 步兵跟随云台偏航模式（正常操控）
    }
    else if(RC.check_in_pos)
    {
        chassis_mode = CHASSIS_CHECK_IN;                   // 入场检测模式
    }

    // --- 3. 根据底盘模式设置目标状态并计算控制量 --- 
    if (chassis_mode == CHASSIS_ZERO_FORCE)
    {
        Target_Velocity = 0.0f;
        Target_X = 0.0f;
        Target_Omega = 0.0f;
        Target_Roll = 0.0f;
        Target_Theta = 0.0f;
        Target_height = 0.1;

        // 初始化标志位清零
        init_pos = false;

        FSM_Jump.Set_Status(STOP);  // 重置跳跃状态机
        
        Chassis.Set_above_flag_l(0); // 清空离地标志位
        Chassis.Set_above_flag_r(0); // 清空离地标志位

        Chassis.Set_Target_left_f0(0);  // 目标腿力清零
        Chassis.Set_Target_right_f0(0); // 目标腿力清零

        // 下发所有目标值到Chassis对象
        Chassis.Set_Target_X(Target_X);
        Chassis.Set_Target_Velocity(Target_Velocity);
        Chassis.Set_Target_Omega(Target_Omega);
        Chassis.Set_Target_Roll(Target_Roll);
        Chassis.Set_Target_Theta(Target_Theta);
        PID_Follow_yaw.Set_Integral_Error(0.0);
        Chassis.Zero_Force_Control();
    }
    else if (chassis_mode == CHASSIS_INFANTRY_FOLLOW_GIMBAL_YAW)
    {
        // -- 正常站立或离地，且不在上楼梯屈腿状态 --
        if ((FSM_Chassis.Get_Now_Status_Serial() == NORMAL || FSM_Chassis.Get_Now_Status_Serial() == ABOVE_GROUND)&& FSM_Up_staris.Get_Now_Status_Serial() != UP_STARIS_BEND_LEG)
        {
            float vx_channel = 0, vy_channel = 0, vz_channel = 0, Roll_channel = 0, vel = 0;
            // sin: 相对角度正弦; cos: 跟随偏航与相对角度差的余弦
            float sin, cos = 0;
            // reletive_angle: 云台相对偏航角; tmp_follow_angle: 临时跟随角度
            float reletive_angle, tmp_follow_angle = 0;

        //原地180度掉头，目前被注释
        if (yaw_rotate_active)
        {
            float current_yaw = mc02_SerialPort.Rx_Data.motor_yaw_angle;    // 获取当前云台偏航角
            float yaw_error = target_yaw_abs - current_yaw; // 计算偏航误差
            if (yaw_error > M_PI) yaw_error -= 2.0f * M_PI; // 角度归一化到 [-π, π]
            if (yaw_error < -M_PI) yaw_error += 2.0f * M_PI;

            // PID计算偏航角速度输出
            PID_Follow_yaw.Set_Target(target_yaw_abs);
            PID_Follow_yaw.Set_Now(current_yaw);
            PID_Follow_yaw.TIM_Calculate_PeriodElapsedCallback();
            float omega_output = PID_Follow_yaw.Get_Out();

            // 当偏航误差小于0.2弧度时，结束掉头
            if (fabs(yaw_error) < 0.2f) {yaw_rotate_active = false;}

            // 旋转期间停止平移速度控制，只进行掉头
            Target_Velocity = 0.0f;
            Target_Velocity_Y = 0.0f;
            Target_Omega = omega_output;
            Target_Roll = 0.0f;
            Target_Pitch = 0.0f;
            Target_Theta = 0.0f;
            // 高度保持不变（不更新Target_height）
        }
        else
        {
            // ---- 正常遥控控制模式 ----
            // 读取遥控器各通道值
            vx_channel = RC.axis[0];      // 前后方向
            vy_channel = RC.axis[1];      // 左右方向
            vz_channel = RC.axis[2];      // 高度方向
            Roll_channel = RC.axis[3];    // 横滚
            reletive_angle = mc02_SerialPort.Rx_Data.motor_yaw_angle ;
            sin = sinf(reletive_angle);
            cos = cosf(follow_yaw_angle - reletive_angle);

            // 跳跃空中阶段超时保护：若起跳后长时间未落地，强制退出跳跃状态
            if(FSM_Jump.Get_Now_Status_Serial() == STRETCH_LEG_AIR)
            {
                mod++;
            }
            if(mod > 1000)
            {
                mod = 0;
                Chassis.Set_above_flag_l(0);
                Chassis.Set_above_flag_r(0);
                jump_flag = 0;
                FSM_Jump.Set_Status(STOP);
            }

            // 遥控器摇杆死区滤波：当摇杆值小于死区阈值时，置零
            // 避免摇杆中位抖动导致误动作
            vx_channel = fabs(vx_channel) > DR16_Rocker_Dead_Zone ? vx_channel : 0.0f;
            vy_channel = fabs(vy_channel) > DR16_Rocker_Dead_Zone ? vy_channel : 0.0f;
            vz_channel = fabs(vz_channel) > DR16_Rocker_Dead_Zone ? vz_channel : 0.0f;
            Roll_channel = fabs(Roll_channel) > DR16_Rocker_Dead_Zone ? Roll_channel : 0.0f;

            // 将遥控器通道值映射为实际目标速度
            // 非小陀螺模式：正常速度映射
            if (!RC.spin)
            {
                Target_Velocity = (vx_channel + 0.0f) * MAX_Velocity;
                Target_Velocity_Y = vy_channel * MAX_Velocity_Y;
            }
            else
            {
                // 小陀螺模式：取消平移速度指令，仅靠倾斜产生位移
                Target_Velocity = (0) * MAX_Velocity;
                Target_Velocity_Y = 0 * MAX_Velocity_Y;
            }
            // 根据跟随偏航角与当前相对角度的差值，余弦衰减X方向速度
            // cos^3 使得小角度偏差时速度损失较小，大角度偏差时(如90°)速度接近0
            Target_Velocity = pow(cos, 3) * Target_Velocity;

            // 使用斜坡函数对X方向速度进行平滑，防止速度突变
            Slope_X.Set_Target(Target_Velocity);
            Slope_X.Set_Now_Real(pow(cos, 3) * Chassis.Get_Velocity_filter());
            Slope_X.TIM_Calculate_PeriodElapsedCallback();
            Target_Velocity = Slope_X.Get_Out();
            
            // Y方向速度同样使用余弦衰减和斜坡平滑
            Target_Velocity_Y = pow(cos, 3) * Target_Velocity_Y;
            Slope_Y.Set_Target(Target_Velocity_Y);
            Slope_Y.Set_Now_Real(pow(cos - M_PI / 2, 3) * Chassis.Get_Velocity_filter());
            Slope_Y.TIM_Calculate_PeriodElapsedCallback();
            Target_Velocity_Y = Slope_Y.Get_Out();

            // 计算合速度大小
            vel = sqrt(Target_Velocity * Target_Velocity + Target_Velocity_Y * Target_Velocity_Y);
            // 当按下A/D键时：底盘跟随速度方向旋转（底盘朝向运动方向）
            if(RC.press_a == 1 || RC.press_d == 1)
            {
                tmp_follow_angle = Get_target_angle(Target_Velocity, Target_Velocity_Y, mc02_SerialPort.Rx_Data.motor_yaw_angle, &vel);
                follow_yaw_angle = tmp_follow_angle + follow_yaw_offset;
            }
            else
            {
                // 无A/D键按下时：不跟踪速度方向，保持当前跟随偏移角
                Get_target_angle(Target_Velocity, Target_Velocity_Y, mc02_SerialPort.Rx_Data.motor_yaw_angle, &vel);
                follow_yaw_angle = follow_yaw_offset;
            }
            // 重新计算cos用于后续速度衰减
            cos = cosf(follow_yaw_angle - reletive_angle);
            // 倒车时速度减半，限制倒车最大速度
            if(vel < 0)
            {
                vel = 0.5 * vel;
            }
        }

            // ---- 小陀螺模式：底盘自旋，通过倾斜控制移动方向 ----
            if(RC.spin)
            {
                // 世界坐标系下的tilt指令转换为机体坐标系下的pitch/roll（旋转矩阵变换）
                float vx_ch = fabs(vx_channel) > DR16_Rocker_Dead_Zone ? vx_channel : 0.0f;
                float vy_ch = fabs(vy_channel) > DR16_Rocker_Dead_Zone ? vy_channel : 0.0f;
                float vx_tilt = vx_ch * RC_to_Chassis_Spin_Pitch_Gain;
                float vy_tilt = vy_ch * RC_to_Chassis_Spin_Pitch_Gain;
                bool mod = 0;

                // 云台角度 + 偏置 → 用于tilt在世界坐标与机体坐标间的转换
                float gimbal_angle = mc02_SerialPort.Rx_Data.motor_yaw_angle + Spin_Pitch_Angle_Offset;
                Target_Pitch =  vx_tilt * cosf(gimbal_angle) + vy_tilt * sinf(gimbal_angle);
                Target_Roll  = -vx_tilt * sinf(gimbal_angle) + vy_tilt * cosf(gimbal_angle);

                // 限制pitch/roll在±0.20弧度内，防止过大倾斜
                Float_Math_Constrain(&Target_Pitch, -0.20f, 0.20f);
                Float_Math_Constrain(&Target_Roll,  -0.20f, 0.20f);

                // 小陀螺模式下平移速度为零，恒速自旋
                Target_Velocity = 0.0f;
                Target_Omega = (fabs(vx_ch) > 0.01f || fabs(vy_ch) > 0.01f) ? -7.0f : -7.0f;

                Chassis.Set_Target_Pitch(Target_Pitch);
                Chassis.Set_Spin_Mode(true);
            }
            else
            {
                // 非小陀螺模式：pitch归零，roll由遥控器控制
                Target_Pitch = 0.0f;
                Target_Roll = Roll_channel * RC_to_Chassis_Roll_Gain;
                Chassis.Set_Target_Pitch(Target_Pitch);
                Chassis.Set_Spin_Mode(false);

                #if FUS_I6X
                // 富斯I6X遥控器：偏航角速度由PID计算
                Target_Omega = -PID_Follow_yaw.Get_Out();
                #endif

                #if GAMEPAD
                // 游戏手柄：偏航角速度直接由摇杆映射
                 Target_Omega = vy_channel * RC_to_Chassis_Yaw_Gain;
                #endif

                // 偏航PID跟随计算
                PID_Follow_yaw.Set_Target(follow_yaw_angle);
                PID_Follow_yaw.Set_Now(mc02_SerialPort.Rx_Data.motor_yaw_angle);
                PID_Follow_yaw.TIM_Calculate_PeriodElapsedCallback();
                Target_Omega = -PID_Follow_yaw.Get_Out();

            }
            
            // 目标高度由遥控器上下通道积分控制，并限幅
            Target_height += vz_channel * RC_to_Chassis_Leg_Gain;
            Target_dheight = 0.0;
            Float_Math_Constrain(&Target_height, 0.13f, 0.34f);

            // 偏航角速度通过斜坡函数平滑处理，防止突变
            Slope_Omega.Set_Target(Target_Omega);
            Slope_Omega.Set_Now_Real(INS.gyro[2]);
            Slope_Omega.TIM_Calculate_PeriodElapsedCallback();
            Target_Omega = Slope_Omega.Get_Out();

            // 下发底盘位置目标
            Chassis.Set_Target_X(Target_X);
            #if FUS_I6X
            // 富斯I6X：目标速度经过cos^3衰减（考虑跟随偏航角差异）
            Chassis.Set_Target_Velocity(vel * cos * cos* cos);
            #endif

            #if GAMEPAD
            // 游戏手柄：目标速度通过斜坡平滑
            Target_Velocity = (vx_channel + 0.0f) * MAX_Velocity;
            Slope_X.Set_Target(Target_Velocity);
            Slope_X.Set_Now_Real(Chassis.Get_Velocity_filter());
            Slope_X.TIM_Calculate_PeriodElapsedCallback();
            Target_Velocity = Slope_X.Get_Out();
            #endif

            // 将所有目标值下发给Chassis底层控制对象
            Chassis.Set_Target_Velocity(Target_Velocity);
            Chassis.Set_Target_height(Target_height);
            Chassis.Set_Target_dheight(Target_dheight);
            Chassis.Set_Target_Omega(Target_Omega);
            Chassis.Set_Yaw_Angle(mc02_SerialPort.Rx_Data.motor_yaw_angle);
            Chassis.Set_Target_Roll(Target_Roll);
            Chassis.Set_Target_Theta(Target_Theta);
            Chassis.Set_Target_Yaw(follow_yaw_angle);
            // 执行跟随云台控制
            Chassis.Follow_Gimbal_Control();
        }
        // -- 子状态：OVER_TURN（侧翻状态），且不在上楼梯屈腿状态 --
        else if(FSM_Chassis.Get_Now_Status_Serial() == OVER_TURN &&
                FSM_Up_staris.Get_Now_Status_Serial() != UP_STARIS_BEND_LEG)
        {
            // 执行侧翻恢复程序
            Check_over_turn_mod();
        }
        // -- 子状态：OVER_TURN_UPSIDE1/2（完全翻倒/仰面朝天状态） --
        else if((FSM_Chassis.Get_Now_Status_Serial() == OVER_TURN_UPSIDE1 ||
                FSM_Chassis.Get_Now_Status_Serial() == OVER_TURN_UPSIDE2)&&
                FSM_Up_staris.Get_Now_Status_Serial() != UP_STARIS_BEND_LEG)
        {
            // 执行自救恢复程序
            Self_Rescue();
        }
        // -- 子状态：OVER_TURNING1（正在翻倒过程中），且不在上楼梯屈腿状态 --
        else if (FSM_Chassis.Get_Now_Status_Serial() == OVER_TURNING1 && FSM_Up_staris.Get_Now_Status_Serial() != UP_STARIS_BEND_LEG)
        {
            float tmp_tar_l, tmp_tar_r = 0.0f;
            // 目标速度与位移清零
            Target_Velocity = 0.0f;
            Target_X = 0.0f;
            // 目标角速度清零
            Target_Omega = 0.0f;
            // 目标横滚角清零
            Target_Roll = 0.0f;
            // 目标腿角度误差清零
            Target_Theta = 0.0f;
            // 翻倒恢复时使用较低高度
            Target_height = 0.13f;
            Target_dheight = 0.0;
            Float_Math_Constrain(&Target_height, 0.12f, 0.31f);

            // 跳跃空中状态超时保护
            if(FSM_Jump.Get_Now_Status_Serial() == STRETCH_LEG_AIR)
            {
                mod++;
            }
            if(mod > 1000)
            {
                mod = 0;
                Chassis.Set_above_flag_l(0);
                Chassis.Set_above_flag_r(0);
                jump_flag = 0;
                FSM_Jump.Set_Status(STOP);
            }

            // 下发目标值并执行腿部控制
            Chassis.Set_Target_X(Target_X);
            Chassis.Set_Target_Velocity(Target_Velocity);
            Chassis.Set_Target_height(Target_height);
            Chassis.Set_Target_dheight(Target_dheight);
            Chassis.Set_Target_Omega(Target_Omega);
            Chassis.Set_Target_Roll(Target_Roll);
            Chassis.Set_Target_Theta(Target_Theta);
            Chassis.Control_leg();
        }
        // -- 子状态：UP_STARIS_BEND_LEG（上楼梯屈腿状态） --
        else if(FSM_Up_staris.Get_Now_Status_Serial() == UP_STARIS_BEND_LEG)
        {
            // 执行上楼梯控制（腿部弯曲以降低重心）
            Chassis.UP_Staris_Control_leg();
        }

    }
    // ===== 3.3 入场检测模式 =====
    else if(chassis_mode == CHASSIS_CHECK_IN)
    {
        // 遥控器通道变量声明
        float vx_channel = 0, vy_channel = 0, vz_channel = 0, Roll_channel = 0;
            vx_channel = RC.axis[0];       // 前后方向
            vy_channel = RC.axis[1];       // 左右方向
            vz_channel = -RC.axis[2];      // 高度方向（取反，入场模式下遥控器上下通道逻辑翻转）
            Roll_channel = RC.axis[3];     // 横滚

            // 遥控器摇杆死区滤波
            vx_channel = fabs(vx_channel) > DR16_Rocker_Dead_Zone ? vx_channel : 0.0f;
            vy_channel = fabs(vy_channel) > DR16_Rocker_Dead_Zone ? vy_channel : 0.0f;
            vz_channel = fabs(vz_channel) > DR16_Rocker_Dead_Zone ? vz_channel : 0.0f;
            Roll_channel = fabs(Roll_channel) > DR16_Rocker_Dead_Zone ? Roll_channel : 0.0f;

            // 目标速度 = 前后通道 × 最大速度
            Target_Velocity = vx_channel * MAX_Velocity;

            if(RC.spin)
            {
                // 小陀螺自旋
                Target_Omega = 7.0;
            }
            else
            {
                Target_Omega = -PID_Follow_yaw.Get_Out();
            }

            // 目标横滚角
            Target_Roll = Roll_channel * RC_to_Chassis_Roll_Gain;
            // 目标腿角度误差清零
            Target_Theta = 0.0f;
            // 目标高度积分控制（入场模式允许更大高度范围）
            Target_height += vz_channel * RC_to_Chassis_Leg_Gain;
            Target_dheight = 0.0;
            Float_Math_Constrain(&Target_height, 0.1f, 0.38f);

            // 下发所有目标值到Chassis
            Chassis.Set_Target_X(Target_X);
            Chassis.Set_Target_Velocity(Target_Velocity);
            Chassis.Set_Target_height(Target_height);
            Chassis.Set_Target_dheight(Target_dheight);
            Chassis.Set_Target_Omega(Target_Omega);
            Chassis.Set_Target_Roll(Target_Roll);
            Chassis.Set_Target_Theta(Target_Theta);

            // 执行入场检测控制
            Chassis.Check_in_Control();
    }

    // --- 4. 发送遥控器数据更新包 ---
    Remote_update();
}

void Robot::Self_Rescue()
{
    if(FSM_Chassis.Get_Now_Status_Serial() == OVER_TURN_UPSIDE1)
    {
        if(INS.pitch < 1.1)
        {
            Chassis.Control_leg_right(0.0f);
            Chassis.Control_leg_left(0.0f);
        }
        else
        {
            Chassis.Control_leg_right(2.0f);
            Chassis.Control_leg_left(2.0f);
        }
        
    }

    if(FSM_Chassis.Get_Now_Status_Serial() == OVER_TURN_UPSIDE2)
    {
        if(INS.pitch > -1.1)
        {
            Chassis.Control_leg_right(0.0f);
            Chassis.Control_leg_left(0.0f);
        }
        else
        {
            Chassis.Control_leg_right(-2.0f);
            Chassis.Control_leg_left(-2.0f);
        }
        
    }

}

void Robot::Check_over_turn_mod()
{
    // �趨�ٶ�
    Target_Velocity = 0.0f;
    Target_X = 0.0f;
    // �趨ת��
    Target_Omega = 0.0f;
    // �趨�����?
    Target_Roll = 0.0f;
    // �趨theta_err
    Target_Theta = 0.0f;
    Target_height = 0.1;
    if(Chassis.left_leg.Get_theta() < -0.9 || Chassis.left_leg.Get_theta() > 1.4)
    {
        Chassis.Control_leg_left(-5.0f);
    }
    else if(Chassis.left_leg.Get_theta() >= -0.8 && Chassis.left_leg.Get_theta() < 0.5)
    { 
        Chassis.Control_leg_left(5.0f);
    }
    else
    {
        Chassis.Control_leg_left(0.0f);
    }

    if(Chassis.right_leg.Get_theta() < -0.9 || Chassis.right_leg.Get_theta() > 1.4)
    {

        Chassis.Control_leg_right(-5.0f);
    }
    else if(Chassis.right_leg.Get_theta() >= -0.8 && Chassis.right_leg.Get_theta() < 0.5)
    {
        Chassis.Control_leg_right(5.0f);

    }
    else
    {
        Chassis.Control_leg_right(0.0f);
    }
}

void Robot::Remote_update()
{
#if FUS_I6X

    RC.axis[0] = mc02_SerialPort.Rx_Data.RC_ctrl.ch[2];
    RC.axis[1] = 0.0;
    RC.axis[2] = mc02_SerialPort.Rx_Data.RC_ctrl.ch[3];
    RC.axis[3] = 0.0;

    // RC.axis[0] = mc02_SerialPort.Rx_Data.RC_ctrl.ch[3];
    // RC.axis[1] = -mc02_SerialPort.Rx_Data.RC_ctrl.ch[1];
    // RC.axis[2] = -mc02_SerialPort.Rx_Data.RC_ctrl.ch[2];
    // RC.axis[3] = 0.0;

    RC.press_w = mc02_SerialPort.Rx_Data.key & KEY_PRESSED_OFFSET_W;
    RC.press_s = mc02_SerialPort.Rx_Data.key & KEY_PRESSED_OFFSET_S;
    RC.press_a = mc02_SerialPort.Rx_Data.key & KEY_PRESSED_OFFSET_A;
    RC.press_d = mc02_SerialPort.Rx_Data.key & KEY_PRESSED_OFFSET_D;
    RC.press_z = mc02_SerialPort.Rx_Data.key & KEY_PRESSED_OFFSET_Z;
    RC.press_x = mc02_SerialPort.Rx_Data.key & KEY_PRESSED_OFFSET_X;
    RC.press_c = mc02_SerialPort.Rx_Data.key & KEY_PRESSED_OFFSET_C;
    RC.press_f = mc02_SerialPort.Rx_Data.key & KEY_PRESSED_OFFSET_F;
    RC.press_v = mc02_SerialPort.Rx_Data.key & KEY_PRESSED_OFFSET_V;
    RC.press_e = mc02_SerialPort.Rx_Data.key & KEY_PRESSED_OFFSET_E;
    RC.press_shift = mc02_SerialPort.Rx_Data.key & KEY_PRESSED_OFFSET_SHIFT;
    // RC.press_ctrl = mc02_SerialPort.Rx_Data.key & KEY_PRESSED_OFFSET_CTRL;
    
    
    if(switch_is_up(mc02_SerialPort.Rx_Data.RC_ctrl.sw[3]))
    {
        RC.competition = false;
        RC.spin = false;
        RC.zero_force = true;
        RC.check_in_pos = false;
    }
    else if(switch_is_mid(mc02_SerialPort.Rx_Data.RC_ctrl.sw[3]) && switch_is_up(mc02_SerialPort.Rx_Data.RC_ctrl.sw[1]))
    {
        RC.check_in_pos = true;
        RC.competition = false;
        RC.zero_force = false;
        RC.spin = false; 
    }
    else if((switch_is_mid(mc02_SerialPort.Rx_Data.RC_ctrl.sw[3]) && switch_is_down(mc02_SerialPort.Rx_Data.RC_ctrl.sw[1]))
    ||(switch_is_down(mc02_SerialPort.Rx_Data.RC_ctrl.sw[3]) && switch_is_down(mc02_SerialPort.Rx_Data.RC_ctrl.sw[1]))&&
    !(switch_is_down(mc02_SerialPort.Rx_Data.RC_ctrl.sw[1]) && switch_is_down(mc02_SerialPort.Rx_Data.RC_ctrl.sw[2])&&
        switch_is_down(mc02_SerialPort.Rx_Data.RC_ctrl.sw[3]) && switch_is_down(mc02_SerialPort.Rx_Data.RC_ctrl.sw[4])))
    {
        RC.competition = true;
        RC.check_in_pos = false;
        RC.zero_force = false;
        if(switch_is_down(mc02_SerialPort.Rx_Data.RC_ctrl.sw[2]))
        {
            RC.spin = true;
        }
        else
        {
            RC.spin = false; 
        }

        // if(switch_is_down(mc02_SerialPort.Rx_Data.RC_ctrl.sw[4]) && switch_is_up(mc02_SerialPort.Rx_Data.RC_ctrl.sw[2]))
        // {
        //     RC.up_staris = true;
        // }
        // else
        // {
        //     RC.up_staris = false;
        // }

        // if(switch_is_down(mc02_SerialPort.Rx_Data.RC_ctrl.sw[2]))
        // {
        //     RC.jump = true;
        // }
        // else 
        // {
        //     RC.jump = false;
        // }
    }
    else if(switch_is_down(mc02_SerialPort.Rx_Data.RC_ctrl.sw[1]) && switch_is_down(mc02_SerialPort.Rx_Data.RC_ctrl.sw[2])&&
        switch_is_down(mc02_SerialPort.Rx_Data.RC_ctrl.sw[3]) && switch_is_down(mc02_SerialPort.Rx_Data.RC_ctrl.sw[4]))
    {
        StateChange(RC.press_v, &RC.last_input_competition, &RC.competition, 2);

        if(RC.competition)
        {
            RC.check_in_pos = false;
            RC.zero_force = false;
            StateChange(RC.press_f, &RC.last_input_spin, &RC.spin, 2);
        }
        else
        {
            RC.check_in_pos = false;
            RC.zero_force = true;
        }
        
        if(RC.press_w == 1)
        {
            if(RC.press_shift == 0)
            {
                if(Chassis.left_leg.Get_L0() > 0.28)
                {
                    RC.axis[0] = 1.0*static_cast<float>(RC.press_w);
                }
                else
                {
                    RC.axis[0] = 1.0*static_cast<float>(RC.press_w);
                }
            }
            else
            {
                if(Chassis.left_leg.Get_L0() > 0.28)
                {
                    RC.axis[0] = 0.9*static_cast<float>(RC.press_w);
                }
                else
                {
                    RC.axis[0] = 1.5*static_cast<float>(RC.press_w);
                }
            }

        }
        else if(RC.press_s == 1)
        {
            if(Chassis.left_leg.Get_L0() > 0.25)
            {
                RC.axis[0] = -0.5*static_cast<float>(RC.press_s);
            }
            else
            {
                RC.axis[0] = -1.0*static_cast<float>(RC.press_s);
            }
        }

        if(RC.press_a == 1)
        {
            if(Chassis.left_leg.Get_L0() > 0.25)
            {
                RC.axis[1] = -0.4*static_cast<float>(RC.press_a);
            }
            else
            {
                RC.axis[1] = -0.8*static_cast<float>(RC.press_a);
            }
        }
        else if(RC.press_d == 1)
        {
            if(Chassis.left_leg.Get_L0() > 0.25)
            {
                RC.axis[1] = 0.4*static_cast<float>(RC.press_d);
            }
            else
            {
                RC.axis[1] = 0.8*static_cast<float>(RC.press_d);
            }
        }

        if(RC.press_x == 1)
        {
            RC.axis[2] = 1.0*static_cast<float>(RC.press_x);
        }
        else if(RC.press_c == 1)
        {
            RC.axis[2] = -1.0*static_cast<float>(RC.press_c);
        }

        if(RC.press_e == 1 && RC.last_press_e == 0)
        {
            Target_height = 0.20;
        }

        if(Chassis.left_leg.Get_L0() > 0.3 && Chassis.right_leg.Get_L0() > 0.3 && RC.press_c == 1)
        {
            RC.up_staris = true;
        }
        else 
        {
            RC.up_staris = false;
        }

        if(RC.press_ctrl == 1 && RC.last_press_ctrl == 0)
        {
            if (!yaw_rotate_active)
            {
                // ��ȡ��ǰʵ�ʺ���ǣ����Ե���������� IMU��
                float current_yaw = mc02_SerialPort.Rx_Data.motor_yaw_angle;
                target_yaw_abs = current_yaw + M_PI;   // �� 180��
                // ��һ���� [-��, ��] ���䣨��ѡ��
                if (target_yaw_abs > M_PI) target_yaw_abs -= 2.0f * M_PI;
                if (target_yaw_abs < -M_PI) target_yaw_abs += 2.0f * M_PI;
                yaw_rotate_active = true;
            }
        }
        
    
        
        RC.last_press_ctrl = mc02_SerialPort.Rx_Data.key & KEY_PRESSED_OFFSET_CTRL;
        RC.last_press_v = mc02_SerialPort.Rx_Data.key & KEY_PRESSED_OFFSET_V;
        RC.last_press_e = mc02_SerialPort.Rx_Data.key & KEY_PRESSED_OFFSET_E;

    }


#endif

#if GAMEPAD

    RC.axis[1] = static_cast<float>(GamePad.Get_LX()) / XBOX_AXIS_VAL_UP;
    RC.axis[0] = static_cast<float>(GamePad.Get_LY()) / XBOX_AXIS_VAL_LEFT;
    RC.axis[2] = -static_cast<float>(GamePad.Get_RX()) / XBOX_AXIS_VAL_UP;
    RC.axis[3] = static_cast<float>(GamePad.Get_RY()) / XBOX_AXIS_VAL_LEFT;
    RC.axis[4] = static_cast<float>(GamePad.Get_LT()) / XBOX_AXIS_VAL_MAX;
    RC.axis[5] = static_cast<float>(GamePad.Get_RT()) / XBOX_AXIS_VAL_MAX;

    if (GamePad.Get_LT() > 150)
    {
        RC.shoot = false;//true
    }
    else
    {
        RC.shoot = false;
    }

    if(GamePad.Get_RT() > 150)
    {
        RC.up_staris = true;//true;
    }
    else
    {
        RC.up_staris = false;
    }

    StateChange(GamePad.Get_LB(), &RC.last_input_friction, &RC.friction, 2);
    StateChange(GamePad.Get_RB(), &RC.last_input_auto_aim, &RC.auto_aim, 2);
    StateChange(GamePad.Get_X(), &RC.last_input_zero_force, &RC.zero_force, 2);
    StateChange(GamePad.Get_Y(), &RC.last_input_competition, &RC.competition, 2);
    

    if (RC.competition == true)
    {
        StateChange(GamePad.Get_B(), &RC.last_input_spin, &RC.spin, 2);
        RC.jump = GamePad.Get_A();
    }
    if (RC.zero_force == true)
    {
        RC.competition = false;
        RC.spin = false;
    }

#endif

#if VT03

    RC.axis[0] = mc02_SerialPort.Rx_Data.RC_ctrl.ch[1];
    RC.axis[1] = -mc02_SerialPort.Rx_Data.RC_ctrl.ch[2];
    RC.axis[2] = mc02_SerialPort.Rx_Data.RC_ctrl.ch[3];
    RC.axis[3] = 0.0;
    
    if(mc02_SerialPort.Rx_Data.RC_ctrl.sw[1] == 0)
    {
        RC.competition = false;
        RC.spin = false;
        RC.zero_force = true;
    }
    else
    {
        RC.competition = true;
        RC.zero_force = false;
        if(switch_is_down(mc02_SerialPort.Rx_Data.RC_ctrl.sw[4]))
        {
            RC.spin = true;
        }
        else
        {
            RC.spin = false; 
        }

        // if(switch_is_down(mc02_SerialPort.Rx_Data.RC_ctrl.sw[4]))
        // {
        //     RC.up_staris = true;
        // }
        // else
        // {
        //     RC.up_staris = false;
        // }
    }

#endif
}

void Robot::Jump_Control()
{
    static int flag = 0;

    if(RC.jump && flag == 0)
    {
        flag = 1;
        FSM_Jump.Set_Status(BEND_LEG);
    }

    if(flag == 1)
    {
        FSM_Jump.TIM_1ms_Calculate_PeriodElapsedCallback();
        if(FSM_Jump.Get_Now_Status_Serial() == STOP)
        {
            flag = 0;
        }
    }
}

void Robot::UP_staris_Control()
{
    static int flag = 0;
    static int mod = 0;

    if(RC.up_staris && flag == 0)
    {
        flag = 1;
        FSM_Up_staris.Set_Status(APPROACH);
    }

    if(flag == 1)
    {
        std::cout<<FSM_Up_staris.Get_Now_Status_Serial()<<std::endl;
        FSM_Up_staris.TIM_1ms_Calculate_PeriodElapsedCallback();
        mod++;
        if(FSM_Up_staris.Get_Now_Status_Serial() == OVER)
        {
            flag = 0;
        }
        // else if(mod > 1000)
        // {
        //     flag = 0;
        // }
    }

    // if(FSM_Up_staris.Get_Now_Status_Serial() != OVER)
    // {
    //     FSM_Up_staris.TIM_1ms_Calculate_PeriodElapsedCallback();
    //     mod++;
    //     if(mod > 1000)
    //     {
    //         flag = 0;
    //     }
    // }
}