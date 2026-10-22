#include <cstring>
#include <iostream>
#include <thread>
#include <chrono>
#include "Task/task_and_callback.h"
#include "Interaction/robot.h"


int main()
{
    Task_Init();

    while (true)
    {

        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    return 0;
}
