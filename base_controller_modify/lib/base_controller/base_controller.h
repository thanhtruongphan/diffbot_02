#ifndef DIFFBOT_BASE_CONTROLLER_H
#define DIFFBOT_BASE_CONTROLLER_H

#include "diffbot_base_config.h"
#include "encoder_diffbot.h"
#include "motor_controller_interface.h"
#include "pid.h"

namespace diffbot {

  template <typename TMotorController, typename TMotorDriver>
  class BaseController
  {
  public:

    BaseController(TMotorController* motor_controller_left, TMotorController* motor_controller_right);

    void initEncoders(int encoder_resolution);
    void readEncoders();
    void setSpeed(int speed_left, int speed_right);
    long getEncoderTicksLeft();
    long getEncoderTicksRight();
    double getAngularVelocityLeft();
    double getAngularVelocityRight();

  private:
    // constants
    float wheel_radius_ = 0.035; // Bán kính bánh xe (m)
    float max_linear_velocity_ = 0.5; // Vận tốc tuyến tính tối đa (m/s)
    float max_angular_velocity_ = max_linear_velocity_ / wheel_radius_; // Vận tốc góc tối đa (rad/s)

    Encoder encoder_left_;
    Encoder encoder_right_;

    MotorControllerIntf<TMotorDriver>* p_motor_controller_right_;
    MotorControllerIntf<TMotorDriver>* p_motor_controller_left_;

    PID motor_pid_left_;
    PID motor_pid_right_;
  };
}

// Khai báo template alias cho BaseController
template <typename TMotorController, typename TMotorDriver>
using BC = diffbot::BaseController<TMotorController, TMotorDriver>;

// Định nghĩa constructor
template <typename TMotorController, typename TMotorDriver>
diffbot::BaseController<TMotorController, TMotorDriver>::BaseController(TMotorController* motor_controller_left, TMotorController* motor_controller_right)
  : encoder_left_(ENCODER_LEFT_H1, ENCODER_LEFT_H2, ENCODER_RESOLUTION)
  , encoder_right_(ENCODER_RIGHT_H1, ENCODER_RIGHT_H2, ENCODER_RESOLUTION)
  , motor_pid_left_(PWM_MIN, PWM_MAX, K_P, K_I, K_D)
  , motor_pid_right_(PWM_MIN, PWM_MAX, K_P, K_I, K_D)
{
  p_motor_controller_left_ = motor_controller_left;
  p_motor_controller_right_ = motor_controller_right;
}

#endif // DIFFBOT_BASE_CONTROLLER_H