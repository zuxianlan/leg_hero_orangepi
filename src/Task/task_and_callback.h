/**
  ******************************************************************************
  * @file           : task_and_callback.h
  * @author         : Chen Haoran
  * @brief          : None
  * @attention      : None
  * @date           : 2025/11/8
  ******************************************************************************
  */
#ifndef WHEEL_LEG_SYS_TASK_AND_CALLBACK_H
#define WHEEL_LEG_SYS_TASK_AND_CALLBACK_H
/* Includes ------------------------------------------------------------------*/

/* Define --------------------------------------------------------------------*/

/* Enum ----------------------------------------------------------------------*/

/* Variable && Struct --------------------------------------------------------*/

/* Function Declaration ------------------------------------------------------*/
void Task_Init();

void mc02_send_data();
void Vofa_start();

void TIM_1ms_PeriodElapsedCallback();
/* Function ------------------------------------------------------------------*/
extern float get_mahony_pitch();
extern float get_mahony_yaw();

#endif //WHEEL_LEG_SYS_TASK_AND_CALLBACK_H