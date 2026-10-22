/**
  ******************************************************************************
  * @file           : Timer.cpp
  * @author         : Chen Haoran
  * @brief          : None
  * @attention      : None
  * @date           : 2025/11/7
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "Timer.h"
#include <iostream>
/* Define --------------------------------------------------------------------*/

/* Enum ----------------------------------------------------------------------*/

/* Variable && Struct --------------------------------------------------------*/

/* Function Declaration ------------------------------------------------------*/

/* Function ------------------------------------------------------------------*/

class_Timer::class_Timer() : running_(false), interval_(0.001), cycle_count_(0)
{
}

class_Timer::~class_Timer()
{
    stop();
}

bool class_Timer::start(double interval_seconds, Callback callback)
{
    // check parameters is effective
    if (running_ || interval_seconds <= 0)
    {
        return false;
    }

    // set timer parameters
    running_ = true;
    interval_ = interval_seconds;
    callback_ = std::move(callback);
    cycle_count_ = 0;
    start_time_ = Clock::now();

    // create and start work thread
    worker_thread_ = std::thread(&class_Timer::run, this);
    std::this_thread::sleep_for(std::chrono::microseconds(50));

    return true;
}

void class_Timer::stop()
{
    if (!running_)
    {
        return;
    }

    // set parameters
    running_ = false;
    cv_.notify_all();

    if (worker_thread_.joinable())
    {
        worker_thread_.join();
    }
}

void class_Timer::run()
{
    // initialize the next trigger time to the current time
    TimePoint next_time = Clock::now();
    auto now = Clock::now();

    while (running_)
    {
        // calculate the next trigger time : current time + interval time
        next_time += std::chrono::duration_cast<Clock::duration>(Duration(interval_));

        // wait precisely the next trigger time
        preciseWait(next_time);

        // check the operating status again, ensure timer is opening
        if (!running_)
        {
            break;
        }

        // increase the cycle counter(automic operation, thread-safe)
        ++cycle_count_;

        // if callback function is not null, running it
        if (callback_)
        {
            try
            {
                callback_();
            }
            catch (const std::exception& e)
            {
                std::cout << "Timer callback error: " << e.what() << std::endl;
            }
        }
    }
}

void class_Timer::spinWaitUntil(const TimePoint& target_time) const
{
    // 自旋锁等待 - 最高精度但CPU占用100%
    while (Clock::now() < target_time && running_)
    {
        // 空循环，完全自旋
        // 可以插入编译器屏障防止过度优化
        asm volatile("" ::: "memory");
    }
}

void class_Timer::preciseWait(const TimePoint& target_time)
{
    // loop and check until the target time is reached or the timer is stopped
    while (running_)
    {
        auto now = Clock::now();
        auto remaining = target_time - now;

        if (remaining <= std::chrono::nanoseconds(0))
        {
            break;
        }

        // choose strategy based on remaining time
        auto remaining_us = std::chrono::duration_cast<std::chrono::microseconds>(remaining);

        // if (remaining_us > std::chrono::microseconds(200))
        // {
            // remaining time > 200μs：use condition variable
            std::unique_lock<std::mutex> lock(mutex_);
            cv_.wait_for(lock, remaining_us ); // wait only half the time, then check
        // }
        // else if (remaining_us > std::chrono::microseconds(100))
        // {
        //     // remaining time 100-200μs：shortly sleep
        //     std::this_thread::sleep_for(std::chrono::microseconds(10));
        // }
        // else
        // {
        //     // remaining time < 100μs：spin wait
        //     spinWaitUntil(target_time);
        //     break;
        // }
    }
    // wait for completion then continue execution
}

bool class_Timer::waitNext()
{
    // check the precondition: the timer must be running and must not be in callback mode
    if (!running_ || callback_)
    {
        return false;
    }

    // calculate the next trigger time
    TimePoint next_time = start_time_ + std::chrono::duration_cast<Clock::duration>(
        Duration(interval_ * (cycle_count_ + 1)));

    // wait precisely the next trigger time
    preciseWait(next_time);

    // check the operating status again, ensure timer is opening
    if (running_)
    {
        ++cycle_count_;
        return true;
    }

    return false;
}

bool class_Timer::isRunning() const
{
    return running_;
}

double class_Timer::getElapsedTime() const
{
    // if timer is running, calculate the duration from the start until now
    if (running_)
    {
        return std::chrono::duration_cast<Duration>(Clock::now() - start_time_).count();
    }

    // timer is not running, return 0
    return 0.0;
}

uint64_t class_Timer::getCycleCount() const
{
    return cycle_count_;
}
