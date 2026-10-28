/**
  ******************************************************************************
  * @file           : task_and_callback.cpp
  * @author         : Chen Haoran
  * @brief          : None
  * @attention      : None
  * @date           : 2025/11/8
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include <cstring>
#include <iostream>
#include <thread>
#include <chrono>
#include <mutex>
#include <netinet/in.h>
#include "task_and_callback.h"
#include "CRC8_CRC16.h"
#include "Bsp/BspSerialPort.h"
#include "Interaction/robot.h"
#include "user_lib.h"
/* Define --------------------------------------------------------------------*/

/* Enum ----------------------------------------------------------------------*/

/* Variable && Struct --------------------------------------------------------*/
float rc_buffer[5][19] = {0};
/* Function Declaration ------------------------------------------------------*/

/* Function ------------------------------------------------------------------*/


Robot robot;
class_Timer timer_1ms;
class_Timer timer_2ms;
class_Timer timer_4ms;

using Clock = std::chrono::high_resolution_clock; 
using Duration = std::chrono::duration<double>; 

void mc02_callback(std::vector<uint8_t>& data)
{
    if (verify_CRC16_check_sum(data.data(), sizeof(mc_02_Rx_data) + 2))
    {
        const auto* tmp_buffer = reinterpret_cast<mc_02_Rx_data*>(data.data());
        if (tmp_buffer->header == 0x5A)
        {
            std::lock_guard<std::mutex> lock(robot.mc02_SerialPort.rx_mutex);
            // IMU
            robot.mc02_SerialPort.Rx_Data.gyro[0] = tmp_buffer->gyro[0];
            robot.mc02_SerialPort.Rx_Data.gyro[1] = tmp_buffer->gyro[1];
            robot.mc02_SerialPort.Rx_Data.gyro[2] = tmp_buffer->gyro[2];
            robot.mc02_SerialPort.Rx_Data.accel[0] = tmp_buffer->accel[0];
            robot.mc02_SerialPort.Rx_Data.accel[1] = tmp_buffer->accel[1];
            robot.mc02_SerialPort.Rx_Data.accel[2] = tmp_buffer->accel[2];

            // RC
            robot.mc02_SerialPort.Rx_Data.RC_ctrl.ch[1] = tmp_buffer->RC_ctrl.ch[1];
            robot.mc02_SerialPort.Rx_Data.RC_ctrl.ch[2] = tmp_buffer->RC_ctrl.ch[2];
            robot.mc02_SerialPort.Rx_Data.RC_ctrl.ch[3] = tmp_buffer->RC_ctrl.ch[3];
            robot.mc02_SerialPort.Rx_Data.RC_ctrl.ch[4] = tmp_buffer->RC_ctrl.ch[4];
            robot.mc02_SerialPort.Rx_Data.RC_ctrl.rotory_sw[1] = tmp_buffer->RC_ctrl.rotory_sw[1];
            robot.mc02_SerialPort.Rx_Data.RC_ctrl.rotory_sw[2] = tmp_buffer->RC_ctrl.rotory_sw[2];
            robot.mc02_SerialPort.Rx_Data.RC_ctrl.sw[1] = tmp_buffer->RC_ctrl.sw[1];
            robot.mc02_SerialPort.Rx_Data.RC_ctrl.sw[2] = tmp_buffer->RC_ctrl.sw[2];
            robot.mc02_SerialPort.Rx_Data.RC_ctrl.sw[3] = tmp_buffer->RC_ctrl.sw[3];
            robot.mc02_SerialPort.Rx_Data.RC_ctrl.sw[4] = tmp_buffer->RC_ctrl.sw[4];

            // Motor
            robot.mc02_SerialPort.Rx_Data.Motor_joint[0].Position = tmp_buffer->Motor_joint[0].Position;
            std::cout<< tmp_buffer->Motor_joint[0].Position << std::endl;
            robot.mc02_SerialPort.Rx_Data.Motor_joint[1].Position = tmp_buffer->Motor_joint[1].Position;
            robot.mc02_SerialPort.Rx_Data.Motor_joint[2].Position = tmp_buffer->Motor_joint[2].Position;
            robot.mc02_SerialPort.Rx_Data.Motor_joint[3].Position = tmp_buffer->Motor_joint[3].Position;
            robot.mc02_SerialPort.Rx_Data.Motor_wheel[0].Position = tmp_buffer->Motor_wheel[0].Position;
            robot.mc02_SerialPort.Rx_Data.Motor_wheel[1].Position = tmp_buffer->Motor_wheel[1].Position;

            robot.mc02_SerialPort.Rx_Data.Motor_joint[0].omega = tmp_buffer->Motor_joint[0].omega;
            robot.mc02_SerialPort.Rx_Data.Motor_joint[1].omega = tmp_buffer->Motor_joint[1].omega;
            robot.mc02_SerialPort.Rx_Data.Motor_joint[2].omega = tmp_buffer->Motor_joint[2].omega;
            robot.mc02_SerialPort.Rx_Data.Motor_joint[3].omega = tmp_buffer->Motor_joint[3].omega;
            robot.mc02_SerialPort.Rx_Data.Motor_wheel[0].omega = tmp_buffer->Motor_wheel[0].omega;
            robot.mc02_SerialPort.Rx_Data.Motor_wheel[1].omega = tmp_buffer->Motor_wheel[1].omega;

            robot.mc02_SerialPort.Rx_Data.roll = tmp_buffer->roll;
            robot.mc02_SerialPort.Rx_Data.pitch = tmp_buffer->pitch;
            robot.mc02_SerialPort.Rx_Data.yaw = tmp_buffer->yaw;
        }
    }
    else
    {
        std::cout<<"CRC Filed"<<std::endl;
    }
}

void mc02_callback_usb(const uint8_t* data, int length)
{
    if (length < static_cast<int>(sizeof(mc_02_Rx_data) + 2)) return;
    if (verify_CRC16_check_sum(data, sizeof(mc_02_Rx_data) + 2))
    {
        const auto* tmp_buffer = reinterpret_cast<const mc_02_Rx_data*>(data);
        if (tmp_buffer->header == 0x5A)
        {
            std::lock_guard<std::mutex> lock(robot.mc02_SerialPort.rx_mutex);
            // IMU
            robot.mc02_SerialPort.Rx_Data.gyro[0] = -tmp_buffer->gyro[0];
            robot.mc02_SerialPort.Rx_Data.gyro[1] = -tmp_buffer->gyro[1];
            robot.mc02_SerialPort.Rx_Data.gyro[2] = tmp_buffer->gyro[2];
            robot.mc02_SerialPort.Rx_Data.accel[0] = -tmp_buffer->accel[0];
            robot.mc02_SerialPort.Rx_Data.accel[1] = -tmp_buffer->accel[1];
            robot.mc02_SerialPort.Rx_Data.accel[2] = tmp_buffer->accel[2];
            robot.mc02_SerialPort.Rx_Data.motionAccel_b[0] = tmp_buffer->motionAccel_b[0];
            robot.mc02_SerialPort.Rx_Data.motionAccel_b[1] = tmp_buffer->motionAccel_b[1];
            robot.mc02_SerialPort.Rx_Data.motionAccel_b[2] =  tmp_buffer->motionAccel_b[2];

            // RC
            // robot.mc02_SerialPort.Rx_Data.RC_ctrl.ch[1] = tmp_buffer->RC_ctrl.ch[1];
            // robot.mc02_SerialPort.Rx_Data.RC_ctrl.ch[2] = tmp_buffer->RC_ctrl.ch[2];
            // robot.mc02_SerialPort.Rx_Data.RC_ctrl.ch[3] = tmp_buffer->RC_ctrl.ch[3];
            // robot.mc02_SerialPort.Rx_Data.RC_ctrl.ch[4] = tmp_buffer->RC_ctrl.ch[4];
            robot.mc02_SerialPort.Rx_Data.RC_ctrl.ch[1] = median_filter(tmp_buffer->RC_ctrl.ch[1], rc_buffer[0], 19);
            robot.mc02_SerialPort.Rx_Data.RC_ctrl.ch[2] = median_filter(tmp_buffer->RC_ctrl.ch[2], rc_buffer[1], 19);
            robot.mc02_SerialPort.Rx_Data.RC_ctrl.ch[3] = median_filter(tmp_buffer->RC_ctrl.ch[3], rc_buffer[2], 19);
            robot.mc02_SerialPort.Rx_Data.RC_ctrl.ch[4] = median_filter(tmp_buffer->RC_ctrl.ch[4], rc_buffer[3], 19);
            robot.mc02_SerialPort.Rx_Data.RC_ctrl.rotory_sw[1] = tmp_buffer->RC_ctrl.rotory_sw[1];
            robot.mc02_SerialPort.Rx_Data.RC_ctrl.rotory_sw[2] = tmp_buffer->RC_ctrl.rotory_sw[2];
            robot.mc02_SerialPort.Rx_Data.RC_ctrl.sw[1] = tmp_buffer->RC_ctrl.sw[1];
            robot.mc02_SerialPort.Rx_Data.RC_ctrl.sw[2] = tmp_buffer->RC_ctrl.sw[2];
            robot.mc02_SerialPort.Rx_Data.RC_ctrl.sw[3] = tmp_buffer->RC_ctrl.sw[3];
            robot.mc02_SerialPort.Rx_Data.RC_ctrl.sw[4] = tmp_buffer->RC_ctrl.sw[4];

            // Motor
            robot.mc02_SerialPort.Rx_Data.Motor_joint[0].Position = tmp_buffer->Motor_joint[0].Position;
            robot.mc02_SerialPort.Rx_Data.Motor_joint[1].Position = tmp_buffer->Motor_joint[1].Position;
            robot.mc02_SerialPort.Rx_Data.Motor_joint[2].Position = tmp_buffer->Motor_joint[2].Position;
            robot.mc02_SerialPort.Rx_Data.Motor_joint[3].Position = tmp_buffer->Motor_joint[3].Position;
            robot.mc02_SerialPort.Rx_Data.Motor_wheel[0].Position = tmp_buffer->Motor_wheel[0].Position;
            robot.mc02_SerialPort.Rx_Data.Motor_wheel[1].Position = tmp_buffer->Motor_wheel[1].Position;

            robot.mc02_SerialPort.Rx_Data.Motor_joint[0].omega = tmp_buffer->Motor_joint[0].omega;
            robot.mc02_SerialPort.Rx_Data.Motor_joint[1].omega = tmp_buffer->Motor_joint[1].omega;
            robot.mc02_SerialPort.Rx_Data.Motor_joint[2].omega = tmp_buffer->Motor_joint[2].omega;
            robot.mc02_SerialPort.Rx_Data.Motor_joint[3].omega = tmp_buffer->Motor_joint[3].omega;
            robot.mc02_SerialPort.Rx_Data.Motor_wheel[0].omega = tmp_buffer->Motor_wheel[0].omega;
            robot.mc02_SerialPort.Rx_Data.Motor_wheel[1].omega = tmp_buffer->Motor_wheel[1].omega;

            robot.mc02_SerialPort.Rx_Data.roll = tmp_buffer->roll;
            robot.mc02_SerialPort.Rx_Data.pitch = tmp_buffer->pitch;
            robot.mc02_SerialPort.Rx_Data.yaw = tmp_buffer->yaw;

            // robot.mc02_SerialPort.Rx_Data.current[0] = tmp_buffer->current[0];
            // robot.mc02_SerialPort.Rx_Data.current[1] = tmp_buffer->current[1];

            robot.mc02_SerialPort.Rx_Data.motor_yaw_angle = tmp_buffer->motor_yaw_angle;
            robot.mc02_SerialPort.Rx_Data.key = median_filter(tmp_buffer->key, rc_buffer[4], 19);
            
        }
    }
    else
    {
        std::cout<<"CRC Filed"<<std::endl;
    }
}

void mc02_send_data()
{
    auto* tmp_buffer = reinterpret_cast<mc_02_Tx_data*>(robot.mc02_SerialPort.Tx_buffer);

    tmp_buffer->header = 0xA5;

    tmp_buffer->joint_torque_set[0] = robot.Chassis.left_leg.Get_torque1()*1.0;
    tmp_buffer->joint_torque_set[1] = robot.Chassis.left_leg.Get_torque2()*1.0;
    tmp_buffer->joint_torque_set[2] = -robot.Chassis.right_leg.Get_torque1()*1.0;
    tmp_buffer->joint_torque_set[3] = -robot.Chassis.right_leg.Get_torque2()*1.0;
    tmp_buffer->wheel_torque_set[0] = robot.Chassis.Get_T_L()*1.0;
    tmp_buffer->wheel_torque_set[1] = -robot.Chassis.Get_T_R()*1.0;
    tmp_buffer->spin_flag = static_cast<uint8_t> (robot.RC.spin);
    // tmp_buffer->wheel_torque_set[0] = 0.5;
    // tmp_buffer->wheel_torque_set[1] = 0.5;


    append_CRC16_check_sum(robot.mc02_SerialPort.Tx_buffer, sizeof(mc_02_Tx_data) + 2);

    if (robot.mc02_SerialPort.usb.is_connected())
    {
        robot.mc02_SerialPort.usb.send(robot.mc02_SerialPort.Tx_buffer, sizeof(mc_02_Tx_data) + 2);
    }

}

/**
 * @brief 1ms周期回调函数 用于更新INS和底盘卡尔曼滤波
 * 
 */
void TIM_1ms_PeriodElapsedCallback()
{
    std::lock_guard<std::mutex> lock(robot.mc02_SerialPort.rx_mutex);

    robot.INS.INS_Update(robot.mc02_SerialPort.Rx_Data);

    robot.Chassis.Self_Observe();
}

/**
 * @brief 1ms周期回调函数 用于底盘控制和状态更新
 * 
 */
void TIM_2ms_PeriodElapsedCallback()
{
    // 记录回调起始时间，用于性能测量
    auto start = std::chrono::high_resolution_clock::now(); 
    {
        // 加锁保护串口接收缓冲区，防止与串口接收中断产生竞争
        std::lock_guard<std::mutex> lock(robot.mc02_SerialPort.rx_mutex);
        robot.FSM_Chassis.TIM_1ms_Calculate_PeriodElapsedCallback();
        robot.Jump_Control();
        robot.UP_staris_Control();
        robot.Chassis.feedback_update();
        robot.Chassis_Control();
        if(robot.Get_jump_flag() == 0)
        {
            robot.Chassis.Ground_Detection();
        }
        Vofa_start();
    }
    // 发送MC02串口数据
    mc02_send_data();

    // 计算并输出本轮回调耗时（微秒）
    auto stop = std::chrono::high_resolution_clock::now();  
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start).count();  
  
    std::cout << "Elapsed time: " << duration << " us\n"; 

}

/**
 * @brief 4ms 周期回调：10 状态全状态线性 MPC
 *        周期必须与 Robot_MPC::Init(0.004) 的 dt 一致
 *        锁在 MPC_Robot_Calc() 内部（只包数据拷贝），这里不加锁
 */
void TIM_4ms_PeriodElapsedCallback()
{
    robot.Chassis.MPC_Robot_Calc();
}

void Task_Init()
{
    robot.Init();
    // robot.mc02_SerialPort.serial_port_.SetCallbackFunction(mc02_callback);
    robot.mc02_SerialPort.usb.on_data(mc02_callback_usb);

    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    // timer initialize
    timer_1ms.start(0.001, TIM_1ms_PeriodElapsedCallback);

    timer_2ms.start(0.001, TIM_2ms_PeriodElapsedCallback);

    timer_4ms.start(0.004, TIM_4ms_PeriodElapsedCallback);


}

void Vofa_start()
{
    // robot.Vofa_TCP.send_message.v0 = robot.INS.MotionAccel_n[1];
    // robot.Vofa_TCP.send_message.v0 = robot.INS.yaw;
    // robot.Vofa_TCP.send_message.v1 = robot.INS.pitch;
    // robot.Vofa_TCP.send_message.v2 = robot.INS.roll;
    // robot.Vofa_TCP.send_message.v3 = robot.mc02_SerialPort.Rx_Data.motor_yaw_angle;
    // robot.Vofa_TCP.send_message.v4 = robot.Chassis.right_leg.Get_theta();
    // robot.Vofa_TCP.send_message.v5 = robot.Chassis.left_leg.Get_theta();
    // robot.Vofa_TCP.send_message.v6 = robot.FSM_Chassis.Get_Now_Status_Serial();
    // robot.Vofa_TCP.send_message.v7 = robot.RC.axis[2];
    // robot.Vofa_TCP.send_message.v8 = robot.Get_Target_height();
    // robot.Vofa_TCP.send_message.v9 = robot.Chassis.left_leg.Get_L0();
    // robot.Vofa_TCP.send_message.v10 = robot.Chassis.right_leg.Get_L0();
    // robot.Vofa_TCP.send_message.v11 = robot.Chassis.left_leg.Get_torque1();
    // robot.Vofa_TCP.send_message.v12 = robot.Chassis.Get_Heigh();
    // robot.Vofa_TCP.send_message.v13 = robot.Chassis.left_leg.Get_Fn();
    // robot.Vofa_TCP.send_message.v14 = robot.Chassis.right_leg.Get_Fn();
    // robot.Vofa_TCP.send_message.v15 = robot.Chassis.Get_above_flag_r();
    // robot.Vofa_TCP.send_message.v16 = robot.mc02_SerialPort.Rx_Data.Motor_joint[0].Position;
    // robot.Vofa_TCP.send_message.v17 = robot.mc02_SerialPort.Rx_Data.Motor_joint[1].Position;
    // robot.Vofa_TCP.send_message.v18 = robot.mc02_SerialPort.Rx_Data.Motor_joint[2].Position;
    // robot.Vofa_TCP.send_message.v19 = robot.mc02_SerialPort.Rx_Data.Motor_joint[3].Position;
   
    // robot.Vofa_TCP.send_message.v0 = robot.INS.yaw;
    // robot.Vofa_TCP.send_message.v1 = robot.INS.pitch;
    // robot.Vofa_TCP.send_message.v2 = robot.INS.roll;
    // robot.Vofa_TCP.send_message.v3 = robot.INS.gyro[0];//dpitch
    // robot.Vofa_TCP.send_message.v4 = robot.INS.gyro[2];//dyaw
    // robot.Vofa_TCP.send_message.v5 = robot.INS.gyro[1];//droll

    // robot.Vofa_TCP.send_message.v0 = robot.mc02_SerialPort.Rx_Data.Motor_joint[1].Position + 3.1415; //left_phi1
    // robot.Vofa_TCP.send_message.v1 = robot.mc02_SerialPort.Rx_Data.Motor_joint[0].Position; //left_phi4
    // robot.Vofa_TCP.send_message.v2 = 3.1415 - robot.mc02_SerialPort.Rx_Data.Motor_joint[2].Position; //right_phi1
    // robot.Vofa_TCP.send_message.v3 = -robot.mc02_SerialPort.Rx_Data.Motor_joint[3].Position;//right_phi4
    // robot.Vofa_TCP.send_message.v4 = robot.mc02_SerialPort.Rx_Data.Motor_joint[0].omega; // left_dphi4
    // robot.Vofa_TCP.send_message.v5 = robot.mc02_SerialPort.Rx_Data.Motor_joint[1].omega; // left_dphi1
    // robot.Vofa_TCP.send_message.v6 = -robot.mc02_SerialPort.Rx_Data.Motor_joint[2].omega; // right_dphi1
    // robot.Vofa_TCP.send_message.v7 = -robot.mc02_SerialPort.Rx_Data.Motor_joint[3].omega; // right_dphi4

    // robot.Vofa_TCP.send_message.v0 = robot.Chassis.left_leg.Get_theta();
    // robot.Vofa_TCP.send_message.v1 = robot.Chassis.right_leg.Get_theta();

    // robot.Vofa_TCP.send_message.v0 = robot.mc02_SerialPort.Rx_Data.Motor_wheel[0].omega; //left
    // robot.Vofa_TCP.send_message.v1 = robot.mc02_SerialPort.Rx_Data.Motor_wheel[1].omega; //left

    robot.Vofa_TCP.send_message.v0 = robot.Chassis.left_leg.Get_torque1();  //joint_torque_set[0]
    robot.Vofa_TCP.send_message.v1 = robot.Chassis.left_leg.Get_torque2();  //joint_torque_set[1]
    robot.Vofa_TCP.send_message.v2 = -robot.Chassis.right_leg.Get_torque1(); //joint_torque_set[2]
    robot.Vofa_TCP.send_message.v3 = -robot.Chassis.right_leg.Get_torque2(); //joint_torque_set[3]
    robot.Vofa_TCP.send_message.v4 = robot.Chassis.Get_T_L();  //wheel_torque_set[0]
    robot.Vofa_TCP.send_message.v5 = -robot.Chassis.Get_T_R(); //wheel_torque_set[1]

    // robot.Vofa_TCP.send_message.v0 = robot.Chassis.left_leg.Get_L0();
    // robot.Vofa_TCP.send_message.v1 = robot.Chassis.right_leg.Get_L0();
    // robot.Vofa_TCP.send_message.v2 = robot.Chassis.right_leg.Get_theta();
    // robot.Vofa_TCP.send_message.v3 = robot.Chassis.left_leg.Get_theta();

    // robot.Vofa_TCP.send_message.v6 = robot.Chassis.left_leg.Get_Tp();
    // robot.Vofa_TCP.send_message.v7 = robot.Chassis.right_leg.Get_Tp();
    // robot.Vofa_TCP.send_message.v8 = robot.Chassis.left_leg.Get_F0();
    // robot.Vofa_TCP.send_message.v9 = robot.Chassis.right_leg.Get_F0();
    // robot.Vofa_TCP.send_message.v10 = robot.Chassis.left_leg.Get_theta();
    // robot.Vofa_TCP.send_message.v11 = robot.Chassis.right_leg.Get_theta();
    // robot.Vofa_TCP.send_message.v12 = robot.Chassis.MPC_body.u_out(0);
    // robot.Vofa_TCP.send_message.v13 = robot.Chassis.MPC_body.u_out(1);
    // robot.Vofa_TCP.send_message.v14 = robot.Chassis.PID_tp_omega.Get_Out();

    // robot.Vofa_TCP.send_message.v0  = robot.Chassis.Get_average_speed();   // z(0) 轮子说的
    // // robot.Vofa_TCP.send_message.v1  = robot.Chassis.Get_v_filter();           // z(1) 加速度说的
    // robot.Vofa_TCP.send_message.v2 = robot.INS.MotionAccel_b[1];
    //robot.Vofa_TCP.send_message.v3 = robot.mc02_SerialPort.Rx_Data.motionAccel_b[1];
    // robot.Vofa_TCP.send_message.v4 = robot.Chassis.Get_a_filter();

    // robot.Vofa_TCP.send_message.v0  = robot.Chassis.lqr_term[0];   // err0 * K0
    // robot.Vofa_TCP.send_message.v1  = robot.Chassis.lqr_term[1];   // Max(err1*K1, 16)
    // robot.Vofa_TCP.send_message.v2  = robot.Chassis.lqr_term[2];   // -Max(err2*K2, 15)
    // robot.Vofa_TCP.send_message.v3  = robot.Chassis.lqr_term[3];   // err3 * K3
    // robot.Vofa_TCP.send_message.v4 = robot.Chassis.lqr_term[4];   // err4 * K4
    // robot.Vofa_TCP.send_message.v5 = robot.Chassis.lqr_term[5];   // err5 * K5
    // robot.Vofa_TCP.send_message.v6 = robot.Chassis.lqr_term[6];   // err6 * K6
    // robot.Vofa_TCP.send_message.v7 = robot.Chassis.lqr_term[7];   // err7 * K7
    // robot.Vofa_TCP.send_message.v8 = robot.Chassis.lqr_term[8];   // err8 * K8
    // robot.Vofa_TCP.send_message.v9 = robot.Chassis.lqr_term[9];   // err9 * K9
    robot.Vofa_TCP.Start();
}

float get_mahony_pitch()
{
    return robot.mc02_SerialPort.Rx_Data.pitch;
}

float get_mahony_yaw()
{
    return robot.mc02_SerialPort.Rx_Data.yaw;
}