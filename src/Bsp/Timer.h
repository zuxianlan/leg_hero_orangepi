/**
  ******************************************************************************
  * @file           : Timer.h
  * @author         : Chen Haoran
  * @brief          : None
  * @attention      : None
  * @date           : 2025/11/7
  ******************************************************************************
  */
#ifndef WHEEL_LEG_SYS_TIMER_H
#define WHEEL_LEG_SYS_TIMER_H
/* Includes ------------------------------------------------------------------*/
#include <chrono>
#include <thread>
#include <functional>
#include <atomic>
#include <mutex>
#include <condition_variable>
/* Define --------------------------------------------------------------------*/

/* Enum ----------------------------------------------------------------------*/

/* Variable && Struct --------------------------------------------------------*/

/* Function Declaration ------------------------------------------------------*/

/* Function ------------------------------------------------------------------*/

class class_Timer
{
  public:
  using Clock = std::chrono::high_resolution_clock;        // 高精度时钟类型
  using TimePoint = std::chrono::time_point<Clock>;        // 时间点类型
  using Duration = std::chrono::duration<double>;          // 持续时间类型（秒为单位）
  using Callback = std::function<void()>;                  // 回调函数类型

  class_Timer();

  ~class_Timer();

  bool start(double interval_seconds, Callback callback); // 启动定时器

  void stop(); // 停止定时器

  bool isRunning() const; // 检查定时器是否正在运行

  double getElapsedTime() const; // 获取从启动开始经过的时间

  uint64_t getCycleCount() const; // 获取已完成的定时周期数

  bool waitNext(); // 手动等待下一个定时周期（手动模式）

private:

  void run(); // 定时器工作线程的主函数

  void spinWaitUntil(const TimePoint& target_time) const;

  void preciseWait(const TimePoint& target_time); // 精确等待到指定的目标时间

  // 原子变量，用于线程间安全的状态同步
  std::atomic<bool> running_;          // 定时器运行状态标志
  std::atomic<double> interval_;       // 定时器间隔时间（秒）
  std::atomic<uint64_t> cycle_count_;  // 已完成的定时周期计数

  TimePoint start_time_;               // 定时器启动的时间点
  Callback callback_;                  // 用户提供的回调函数

  // 线程同步相关成员
  std::thread worker_thread_;          // 工作线程对象
  std::mutex mutex_;                   // 互斥锁，用于线程同步
  std::condition_variable cv_;         // 条件变量，用于线程间通信和精确等待
};

#endif //WHEEL_LEG_SYS_TIMER_H