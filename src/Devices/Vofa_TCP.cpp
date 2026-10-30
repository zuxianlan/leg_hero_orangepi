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
#include <cstring>
#include <netinet/tcp.h>
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
        return;
    }

    // 2. 配置服务器地址
    struct sockaddr_in server_addr{};
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(Port); // 端口与服务端对应
    if (inet_pton(AF_INET, port_name, &server_addr.sin_addr) <= 0)
    {
        std::cerr << "Invalid address" << std::endl;
        Close();
        return;
    }

    // 3. 连接到 Vofa 服务器
    if (connect(client_fd, (struct sockaddr*)&server_addr, sizeof(server_addr)) < 0)
    {
        std::cerr << "Connection failed (ensure server opened)" << std::endl;
        Close();
        return;
    }

    // 4. 关闭 Nagle：每帧立即发出，曲线不会成批跳动
    int flag = 1;
    setsockopt(client_fd, IPPROTO_TCP, TCP_NODELAY, &flag, sizeof(flag));
}

void Class_Vofa_TCP::Close()
{
    if (client_fd >= 0)
    {
        close(client_fd);
        client_fd = -1;
    }
}

void Class_Vofa_TCP::Start()
{
    // 未连接时直接返回，不打包也不发送
    if (client_fd < 0)
    {
        return;
    }

    // SEND_Message 是 20 个连续 float，整块拷贝即为 JustFloat 数据段
    std::memcpy(Buffer, &send_message, sizeof(send_message));
    cnt = sizeof(send_message);

    // Call the function to send the frame tail
    sendframetail();
}

void Class_Vofa_TCP::Send(uint8_t num, float data)
{
    std::memcpy(&Buffer[cnt], &data, sizeof(data));
    cnt += sizeof(data);
}

void Class_Vofa_TCP::sendframetail()
{
    Buffer[cnt++] = 0x00;
    Buffer[cnt++] = 0x00;
    Buffer[cnt++] = 0x80;
    Buffer[cnt++] = 0x7f;

    // MSG_DONTWAIT：发送缓冲区满时丢掉本帧，不阻塞控制线程
    // MSG_NOSIGNAL：上位机断开时不触发 SIGPIPE（默认会直接杀掉进程）
    const ssize_t ret = send(client_fd, Buffer, cnt, MSG_DONTWAIT | MSG_NOSIGNAL);
    cnt = 0;

    if (ret < 0 && errno != EAGAIN && errno != EWOULDBLOCK)
    {
        // 连接已断开：关闭 socket，后续帧不再发送
        std::cerr << "Vofa connection lost" << std::endl;
        Close();
    }
}
