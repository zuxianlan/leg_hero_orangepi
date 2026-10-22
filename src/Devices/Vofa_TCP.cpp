/**
  ******************************************************************************
  * @file           : Vofa_TCP.cpp
  * @author         : Chen Haoran
  * @brief          : None
  * @attention      : None
  * @date           : 2025/10/25
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "Vofa_TCP.h"
/* Define --------------------------------------------------------------------*/

/* Variable && Struct --------------------------------------------------------*/

/* Function Declaration ------------------------------------------------------*/

/* Function ------------------------------------------------------------------*/

void Class_Vofa_TCP::Init(const char* port_name, int Port)
{
    // 1. 创建 TCP 客户端 socket
    client_fd = socket(AF_INET, SOCK_STREAM, 0);
    if (client_fd < 0)
    {
        std::cerr << "Socket creation failed" << std::endl;
    }

    // 2. 配置服务器地址
    struct sockaddr_in server_addr{};
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(Port); // 端口与服务端对应
    if (inet_pton(AF_INET, port_name, &server_addr.sin_addr) <= 0)
    {
        std::cerr << "Invalid address" << std::endl;
    }

    // 3. 连接到 PlotJuggler 服务器
    if (connect(client_fd, (struct sockaddr*)&server_addr, sizeof(server_addr)) < 0)
    {
        std::cerr << "Connection failed (ensure server opened)" << std::endl;
    }
}

void Class_Vofa_TCP::Start()
{
    Send(0, send_message.v0);
    Send(1, send_message.v1);
    Send(2, send_message.v2);
    Send(3, send_message.v3);
    Send(4, send_message.v4);
    Send(5, send_message.v5);
    Send(6, send_message.v6);
    Send(7, send_message.v7);
    Send(8, send_message.v8);
    Send(9, send_message.v9);
    Send(10, send_message.v10);
    Send(11, send_message.v11);
    Send(12, send_message.v12);
    Send(13, send_message.v13);
    Send(14, send_message.v14);
    Send(15, send_message.v15);
    Send(16, send_message.v16);
    Send(17, send_message.v17);
    Send(18, send_message.v18);
    Send(19, send_message.v19);

    // Call the function to send the frame tail
    sendframetail();

}

void Class_Vofa_TCP::Send(uint8_t num, float data)
{
    Buffer[cnt++] = byte0(data);
    Buffer[cnt++] = byte1(data);
    Buffer[cnt++] = byte2(data);
    Buffer[cnt++] = byte3(data);
}

void Class_Vofa_TCP::sendframetail()
{
    Buffer[cnt++] = 0x00;
    Buffer[cnt++] = 0x00;
    Buffer[cnt++] = 0x80;
    Buffer[cnt++] = 0x7f;

    send(client_fd, Buffer, cnt, 0);
    cnt = 0;
}
