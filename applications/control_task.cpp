#include "cmsis_os.h"
#include "io/can/can.hpp"
#include "io/dbus/dbus.hpp"
#include "motor/rm_motor/rm_motor.hpp"
#include "tools/mahony/mahony.hpp"
#include "tools/math_tools/math_tools.hpp"
#include "tools/pid/pid.hpp"

namespace
{
// CAN ID、方向符号和 PID 参数。
constexpr uint8_t MOTOR_A_ID = 1;
constexpr uint8_t MOTOR_B_ID = 2;

constexpr float CONTROL_DT = 0.001f;
constexpr float MAX_SPEED = 3.0f;      // rad/s，位置外环输出限幅
constexpr float MAX_TORQUE = 0.30f;    // N*m，速度内环输出限幅
constexpr float MAX_I_TORQUE = 0.10f;  // N*m，速度环积分限幅

// 复位角需按实际箭头标定，先用 0 rad 占位。
constexpr float RESET_A_ANGLE = 0.0f;
constexpr float RESET_B_ANGLE = 0.0f;
constexpr float RESET_TOLERANCE = 0.03f;

float ratio_from_left_switch(sp::DBusSwitchMode left_switch)
{
  switch (left_switch) {
    case sp::DBusSwitchMode::DOWN:
      return 0.5f;

    case sp::DBusSwitchMode::MID:
      return -1.0f;

    case sp::DBusSwitchMode::UP:
      return 3.0f;
  }

  return -1.0f;
}

}  // namespace

sp::CAN can1(&hcan1);
extern sp::DBus remote;
sp::RM_Motor motor_a(MOTOR_A_ID, sp::RM_Motors::GM6020);
sp::RM_Motor motor_b(MOTOR_B_ID, sp::RM_Motors::GM6020);

// 位置外环：角度(rad) -> 速度(rad/s)。
sp::PID position_pid_a(CONTROL_DT, 6.0f, 0.0f, 0.0f, MAX_SPEED, 0.0f);
sp::PID position_pid_b(CONTROL_DT, 6.0f, 0.0f, 0.0f, MAX_SPEED, 0.0f);
// 速度内环：速度(rad/s) -> 力矩(N*m)。
sp::PID speed_pid_a(CONTROL_DT, 0.20f, 0.02f, 0.0f, MAX_TORQUE, MAX_I_TORQUE);
sp::PID speed_pid_b(CONTROL_DT, 0.20f, 0.02f, 0.0f, MAX_TORQUE, MAX_I_TORQUE);

extern sp::Mahony imu;

extern "C" void control_task()
{
  osDelay(500);
  can1.config();
  can1.start();

  float yaw_reference = 0.0f;
  float motor_a_reference = 0.0f;
  float motor_b_reference = 0.0f;
  bool reference_ready = false;
  sp::AngleUnwrapper yaw_unwrapper;

  while (true) {
    const auto now = osKernelSysTick();
    const bool motors_alive = motor_a.is_alive(now) && motor_b.is_alive(now);
    const bool remote_alive = remote.is_alive(now);
    const float yaw_continuous = yaw_unwrapper.update(imu.yaw);
    float torque_a = 0.0f;
    float torque_b = 0.0f;

    if (motors_alive && remote_alive) {
      const auto mode = remote.sw_r;

      switch (mode) {
        case sp::DBusSwitchMode::DOWN:
          // 失能时持续更新参考零点；切到 MID 后从当前位置开始联动。
          yaw_reference = yaw_continuous;
          motor_a_reference = motor_a.angle;
          motor_b_reference = motor_b.angle;
          reference_ready = true;
          position_pid_a.clear();
          position_pid_b.clear();
          speed_pid_a.clear();
          speed_pid_b.clear();
          break;

        case sp::DBusSwitchMode::MID: {
          // 防止系统刚上线时还没有经过 DOWN 分支。
          if (!reference_ready) {
            yaw_reference = yaw_continuous;
            motor_a_reference = motor_a.angle;
            motor_b_reference = motor_b.angle;
            reference_ready = true;
          }

          const float yaw_delta = yaw_continuous - yaw_reference;
          const float ratio = ratio_from_left_switch(remote.sw_l);
          const float target_a = motor_a_reference + yaw_delta;
          const float target_b = motor_b_reference + ratio * yaw_delta;

          position_pid_a.calc(target_a, motor_a.angle);
          position_pid_b.calc(target_b, motor_b.angle);
          speed_pid_a.calc(position_pid_a.out, motor_a.speed);
          speed_pid_b.calc(position_pid_b.out, motor_b.speed);
          torque_a = speed_pid_a.out;
          torque_b = speed_pid_b.out;
          break;
        }

        case sp::DBusSwitchMode::UP:
          position_pid_a.calc(RESET_A_ANGLE, motor_a.angle);
          position_pid_b.calc(RESET_B_ANGLE, motor_b.angle);
          speed_pid_a.calc(position_pid_a.out, motor_a.speed);
          speed_pid_b.calc(position_pid_b.out, motor_b.speed);
          torque_a = speed_pid_a.out;
          torque_b = speed_pid_b.out;
          if (fabsf(motor_a.angle - RESET_A_ANGLE) < RESET_TOLERANCE) torque_a = 0.0f;
          if (fabsf(motor_b.angle - RESET_B_ANGLE) < RESET_TOLERANCE) torque_b = 0.0f;
          break;
      }
    }
    else {
      // 遥控器或任一电机掉线时立即失能，并清除积分。
      position_pid_a.clear();
      position_pid_b.clear();
      speed_pid_a.clear();
      speed_pid_b.clear();
      reference_ready = false;
    }

    motor_a.cmd(torque_a);
    motor_b.cmd(torque_b);
    motor_a.write(can1.tx_data);
    motor_b.write(can1.tx_data);
    can1.send(motor_a.tx_id);
    osDelay(1);
  }
}

extern "C" void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef * hcan)
{
  const auto stamp_ms = osKernelSysTick();
  while (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0) > 0) {
    if (hcan == &hcan1) {
      can1.recv();
      if (can1.rx_id == motor_a.rx_id) motor_a.read(can1.rx_data, stamp_ms);
      if (can1.rx_id == motor_b.rx_id) motor_b.read(can1.rx_data, stamp_ms);
    }
  }
}
