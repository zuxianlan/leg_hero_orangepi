/**
  ******************************************************************************
  * @file           : Vofa_TCP.h
  * @author         : Chen Haoran
  * @brief          : None
  * @attention      : None
  * @date           : 2025/10/25
  ******************************************************************************
  */
#ifndef WHEEL_LEG_SYS_VOFA_TCP_H
#define WHEEL_LEG_SYS_VOFA_TCP_H
/* Includes ------------------------------------------------------------------*/
#include <iostream>
#include <unistd.h>
#include <iomanip>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
/* Define --------------------------------------------------------------------*/
#define byte0(dw_temp)     (*(char*)(&dw_temp))
#define byte1(dw_temp)     (*((char*)(&dw_temp) + 1))
#define byte2(dw_temp)     (*((char*)(&dw_temp) + 2))
#define byte3(dw_temp)     (*((char*)(&dw_temp) + 3))
/* Variable && Struct --------------------------------------------------------*/

/* Function Declaration ------------------------------------------------------*/

/* Function ------------------------------------------------------------------*/

typedef struct
{
  float v0;
  float v1;
  float v2;
  float v3;
  float v4;
  float v5;
  float v6;
  float v7;
  float v8;
  float v9;
  float v10;
  float v11;
  float v12;
  float v13;
  float v14;
  float v15;
  float v16;
  float v17;
  float v18;
  float v19;
}SEND_Message;

class Class_Vofa_TCP
{
  public:
  SEND_Message send_message;

  void Init(const char* port_name, int Port);

  void Start();

  void Send(uint8_t num, float data);

  void sendframetail();

  private:
  int client_fd = 0;
  int cnt = 0;
  uint8_t Buffer[256] = {0};

};

#endif //WHEEL_LEG_SYS_VOFA_TCP_H
