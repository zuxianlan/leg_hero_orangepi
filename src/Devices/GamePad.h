/**
  ******************************************************************************
  * @file           : GamePad.h
  * @author         : Chen Haoran
  * @brief          : None
  * @attention      : None
  * @date           : 2025/11/12
  ******************************************************************************
  */
#ifndef WHEEL_LEG_SYS_GAMEPAD_H
#define WHEEL_LEG_SYS_GAMEPAD_H
/*
cat /proc/bus/input/devices
cat /dev/input/event12 | hexdump
*/
/* Includes ------------------------------------------------------------------*/
#include <atomic>
#include <cstring>
#include <iostream>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <functional>
#include <thread>
#include <linux/input.h>
#include <linux/joystick.h>
/* Define --------------------------------------------------------------------*/

#define XBOX_TYPE_BUTTON    1
#define XBOX_TYPE_AXIS      3

#define XBOX_BUTTON_A       304
#define XBOX_BUTTON_B       305
#define XBOX_BUTTON_X       307
#define XBOX_BUTTON_Y       308
#define XBOX_BUTTON_LB      310
#define XBOX_BUTTON_RB      311
#define XBOX_BUTTON_FUNC    315
#define XBOX_BUTTON_BACK    314
#define XBOX_BUTTON_HOME    316
#define XBOX_BUTTON_LO      317    /* ◊Û“°∏À∞¥º¸ */
#define XBOX_BUTTON_RO      318    /* ”““°∏À∞¥º¸ */

#define XBOX_BUTTON_ON      1
#define XBOX_BUTTON_OFF     0

#define XBOX_AXIS_LX        0    /* ◊Û“°∏ÀX÷· */
#define XBOX_AXIS_LY        1    /* ◊Û“°∏ÀY÷· */
#define XBOX_AXIS_RX        3    /* ”““°∏ÀX÷· */
#define XBOX_AXIS_RY        4    /* ”““°∏ÀY÷· */
#define XBOX_AXIS_LT        2
#define XBOX_AXIS_RT        5
#define XBOX_AXIS_XX        16    /* ∑ΩœÚº¸X÷· */
#define XBOX_AXIS_YY        17    /* ∑ΩœÚº¸Y÷· */

#define XBOX_AXIS_VAL_UP        (-32767)
#define XBOX_AXIS_VAL_DOWN      32767
#define XBOX_AXIS_VAL_LEFT      (-32767)
#define XBOX_AXIS_VAL_RIGHT     32767

#define XBOX_AXIS_VAL_MIN       (0)
#define XBOX_AXIS_VAL_MAX       255
#define XBOX_AXIS_VAL_MID       0

/* Variable && Struct --------------------------------------------------------*/

/* Function Declaration ------------------------------------------------------*/

/* Function ------------------------------------------------------------------*/

class class_GamePad
{
public:
    explicit class_GamePad(std::string file_name);
    ~class_GamePad();

    bool Open();
    void Close();

    int Get_A();
    int Get_B();
    int Get_X();
    int Get_Y();
    int Get_LB();
    int Get_RB();
    int Get_FUNC();
    int Get_BACK();
    int Get_HOME();
    int Get_LO();
    int Get_RO();
    int Get_LX();
    int Get_LY();
    int Get_RX();
    int Get_RY();
    int Get_LT();
    int Get_RT();
    int Get_XX();
    int Get_YY();

private:

    void Receive_Data();

    // check is opened
    [[nodiscard]] bool IsOpen() const;

    // read thread function
    void readThreadFunction();

    int fd_{}; // port profile
    std::string file_name;
    std::atomic<bool> is_open_;
    std::atomic<bool> stop_read_thread_;
    std::thread read_thread_;

    input_event event{};

    int time = 0;
    int a = 0;
    int b = 0;
    int x = 0;
    int y = 0;
    int lb = 0;
    int rb = 0;
    int func = 0;
    int back = 0;
    int home = 0;
    int lo = 0;
    int ro = 0;
    int lx = 0;
    int ly = 0;
    int rx = 0;
    int ry = 0;
    int lt = 0;
    int rt = 0;
    int xx = 0;
    int yy = 0;

    int type = 0;
    int code = 0;
    int value = 0;
};

inline int class_GamePad::Get_A()
{
    return a;
}

inline int class_GamePad::Get_B()
{
    return b;
}

inline int class_GamePad::Get_X()
{
    return x;
}

inline int class_GamePad::Get_Y()
{
    return y;
}

inline int class_GamePad::Get_LB()
{
    return lb;
}

inline int class_GamePad::Get_RB()
{
    return rb;
}

inline int class_GamePad::Get_FUNC()
{
    return func;
}

inline int class_GamePad::Get_BACK()
{
    return back;
}

inline int class_GamePad::Get_HOME()
{
    return home;
}

inline int class_GamePad::Get_LO()
{
    return lo;
}

inline int class_GamePad::Get_RO()
{
    return ro;
}

inline int class_GamePad::Get_LX()
{
    return lx;
}

inline int class_GamePad::Get_LY()
{
    return ly;
}

inline int class_GamePad::Get_RX()
{
    return rx;
}

inline int class_GamePad::Get_RY()
{
    return ry;
}

inline int class_GamePad::Get_LT()
{
    return lt;
}

inline int class_GamePad::Get_RT()
{
    return rt;
}

inline int class_GamePad::Get_XX()
{
    return xx;
}

inline int class_GamePad::Get_YY()
{
    return yy;
}




#endif //WHEEL_LEG_SYS_GAMEPAD_H