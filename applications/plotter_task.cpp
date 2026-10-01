#include "cmsis_os.h"
#include "io/bmi088/bmi088.hpp"
#include "io/plotter/plotter.hpp"
#include "motor/rm_motor/rm_motor.hpp"
#include "tools/mahony/mahony.hpp"

extern sp::BMI088 bmi088;
extern sp::Mahony imu;

extern sp::RM_Motor motor6020;

sp::Plotter plotter(&huart1);

extern "C" void plotter_task()
{
  while (1) {
    plotter.plot(imu.yaw, imu.pitch, imu.roll, imu.vyaw, imu.vpitch, imu.vroll);
    osDelay(1);
  }
}