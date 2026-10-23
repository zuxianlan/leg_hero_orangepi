/**
  ******************************************************************************
  * @file           : robot.h
  * @author         : Chen Haoran
  * @brief          : None
  * @attention      : None
  * @date           : 2025/10/25
  ******************************************************************************
  */
#ifndef WHEEL_LEG_SYS_ROBOT_H
#define WHEEL_LEG_SYS_ROBOT_H
/* Includes ------------------------------------------------------------------*/
#include "MPC_Control.h"
#include "QuaternionEKF.h"
#include "Algorithm/FSM/fsm.h"
#include "Algorithm/Slope/slope.h"
#include "Devices/dm_mc02.h"
#include "Devices/Vofa_TCP.h"
#include "Chassis.h"
#include "Devices/INS.h"
#include "GamePad.h"
#include "Algorithm/Fusion_AHRS/Fusion_AHRS.h"
#include "Algorithm/PID/PID_Control.h"
#include "Algorithm/Fusion_AHRS/Tactical_Fusion.h"
/* Define --------------------------------------------------------------------*/

#define KEY_PRESSED_OFFSET_W            ((uint16_t)1 << 0)
#define KEY_PRESSED_OFFSET_S            ((uint16_t)1 << 1)
#define KEY_PRESSED_OFFSET_A            ((uint16_t)1 << 2)
#define KEY_PRESSED_OFFSET_D            ((uint16_t)1 << 3)
#define KEY_PRESSED_OFFSET_SHIFT        ((uint16_t)1 << 4)
#define KEY_PRESSED_OFFSET_CTRL         ((uint16_t)1 << 5)
#define KEY_PRESSED_OFFSET_Q            ((uint16_t)1 << 6)
#define KEY_PRESSED_OFFSET_E            ((uint16_t)1 << 7)
#define KEY_PRESSED_OFFSET_R            ((uint16_t)1 << 8)
#define KEY_PRESSED_OFFSET_F            ((uint16_t)1 << 9)
#define KEY_PRESSED_OFFSET_G            ((uint16_t)1 << 10)
#define KEY_PRESSED_OFFSET_Z            ((uint16_t)1 << 11)
#define KEY_PRESSED_OFFSET_X            ((uint16_t)1 << 12)
#define KEY_PRESSED_OFFSET_C            ((uint16_t)1 << 13)
#define KEY_PRESSED_OFFSET_V            ((uint16_t)1 << 14)
#define KEY_PRESSED_OFFSET_B            ((uint16_t)1 << 15)

#define FUS_I6X 1
#define GAMEPAD 0
#define VT03 0

#define RC_SW_UP                ((uint16_t)1)
#define RC_SW_MID               ((uint16_t)3)
#define RC_SW_DOWN              ((uint16_t)2)
#define switch_is_down(s)       (s == RC_SW_DOWN)
#define switch_is_mid(s)        (s == RC_SW_MID)
#define switch_is_up(s)         (s == RC_SW_UP)

#define RC_to_Chassis_Yaw_Gain (3.0f)
#define RC_to_Chassis_Leg_Gain 0.0003f
#define DR16_Rocker_Dead_Zone 0.1f
#define RC_to_Chassis_Roll_Gain 0.001f
#define RC_to_Chassis_Spin_Pitch_Gain 0.1f
#define Spin_Pitch_Angle_Offset 2.5f   

#define MAX_Velocity 2.0f
#define MAX_Velocity_Y 2.0f

/* Variable && Struct --------------------------------------------------------*/

typedef enum
{
    STOP = 0,
    BEND_LEG,
    STRETCH_LEG,
    BEND_LEG_AIR,
    STRETCH_LEG_AIR
} Enum_jump_fsm_mode;



/* Function Declaration ------------------------------------------------------*/

/* Function ------------------------------------------------------------------*/

class Robot;

typedef struct
{
    float axis[6];
    float leg_length;

    bool friction;
    bool shoot;
    bool auto_aim;
    bool jump;
    bool zero_force;
    bool competition;
    bool check_in_pos;
    bool spin;
    bool up_staris;
    bool leg_low;
    bool leg_middle;
    bool leg_heigh;

    bool last_input_friction;
    bool last_input_auto_aim;
    bool last_input_jump;
    bool last_input_zero_force;
    bool last_input_competition;
    bool last_input_spin;
    bool last_leg_low;
    bool last_leg_middle;
    bool last_leg_heigh;

    bool press_w;
    bool press_s;
    bool press_a;
    bool press_d;
    bool press_z;
    bool press_x;
    bool press_c;
    bool press_f;
    bool press_v;
    bool press_e;
    bool press_shift;
    bool press_ctrl;
    bool last_press_ctrl;
    bool last_press_v;
    bool last_press_e;


}Remote_Control;

class Class_FSM_Chassis : public Class_FSM
{
public:
    Robot *robot{};

    void TIM_1ms_Calculate_PeriodElapsedCallback();

private:

};

class Class_FSM_Jump : public Class_FSM
{
public:
    Robot *robot{};

    void TIM_1ms_Calculate_PeriodElapsedCallback();

private:

};

class Class_FSM_Up_staris : public Class_FSM
{
public:
    Robot *robot{};

    void TIM_1ms_Calculate_PeriodElapsedCallback();

private:

};

class Robot
{
public:
    mc_02_serial mc02_SerialPort;

    Class_Vofa_TCP Vofa_TCP;

    class_INS INS;

    Remote_Control RC{};

    Class_FSM_Chassis FSM_Chassis;
    friend class Class_FSM_Chassis;

    Class_FSM_Jump FSM_Jump;
    friend class Class_FSM_Jump;

    Class_FSM_Up_staris FSM_Up_staris;
    friend class Class_FSM_Up_staris;

    Class_Slope Slope_X;
    Class_Slope Slope_Y;
    Class_Slope Slope_Omega;
    Class_Slope Slope_Leg_l;
    Class_Slope Slope_Leg_r;

    Class_PID PID_Follow_yaw;

    class_Chassis Chassis;
    Enum_chassis_mode chassis_mode;

    void Init();

    void Jump_Control();

    void UP_staris_Control();

    void Chassis_Control();

    void Check_over_turn_mod();

    void Self_Rescue();

    inline float Get_Target_Omega() const       { return Target_Omega; }
    inline float Get_Target_Velocity() const    { return Target_Velocity; }
    inline float Get_Target_Velocity_Y() const  { return Target_Velocity_Y; }
    inline float Get_Target_X() const           { return Target_X; }
    inline float Get_Target_height() const      { return Target_height; }
    inline float Get_jump_flag() const          { return jump_flag; }

private:

    class_GamePad GamePad = class_GamePad("/dev/input/event6");

    void Remote_update();

    float Target_X = 0;
    float Target_Velocity_Y = 0;
    float Target_Velocity = 0;
    float Target_left_f0 = 0;
    float Target_right_f0 = 0;
    float Target_Omega = 0;
    float Target_Roll = 0;
    float Target_Theta = 0;
    float Target_Pitch = 0.0f;
    float last_Target_leg = 0;
    float follow_yaw_angle = 0.0;
    float Target_height = 0;
    float Target_dheight = 0;
    float follow_yaw_offset = 2.34;
    float target_yaw_abs = 0;          


    bool above_flag_l = false;
    bool above_flag_r = false;
    bool jump_flag = false;
    bool Supercap_Accelerate_Status = false;
    bool init_pos = false;
    bool yaw_rotate_active = false;       

};

#endif //WHEEL_LEG_SYS_ROBOT_H