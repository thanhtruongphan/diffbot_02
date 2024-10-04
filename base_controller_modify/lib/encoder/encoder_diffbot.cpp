#include "encoder_diffbot.h"

// Thay thế ros::Time bằng millis()
unsigned long prev_update_time_ = 0; 

diffbot::Encoder::Encoder(uint8_t pin1, uint8_t pin2, int encoder_resolution)
  : encoder(pin1, pin2)
  , encoder_resolution_(encoder_resolution)
  , prev_encoder_ticks_(0)
{

}

diffbot::JointState diffbot::Encoder::jointState()
{
  long encoder_ticks = encoder.read();

  // Tính toán vận tốc góc dựa trên encoder ticks và thời gian
  unsigned long current_time = millis();
  unsigned long dt = current_time - prev_update_time_;

  // Chuyển đổi thời gian sang giây
  double dts = dt / 1000.0;

  // Tính toán thay đổi ticks và góc
  double delta_ticks = encoder_ticks - prev_encoder_ticks_;
  double delta_angle = ticksToAngle(delta_ticks);

  joint_state_.angular_position_ += delta_angle;
  joint_state_.angular_velocity_ = delta_angle / dts;

  prev_update_time_ = current_time;
  prev_encoder_ticks_ = encoder_ticks;

  return joint_state_;
}

double diffbot::Encoder::angularPosition()
{
  return joint_state_.angular_position_;
}

double diffbot::Encoder::angularVelocity()
{
  return joint_state_.angular_velocity_;
}

double diffbot::Encoder::ticksToAngle(const int &ticks) const
{
  // Chuyển đổi số ticks sang góc (radian)
  double angle = (double)ticks * (2.0 * M_PI / encoder_resolution_);
  return angle;
}

int diffbot::Encoder::getRPM()
{
  long encoder_ticks = encoder.read();

  // Tính toán RPM dựa trên encoder ticks và thời gian
  unsigned long current_time = millis();
  unsigned long dt = current_time - prev_update_time_;

  // Chuyển đổi thời gian sang phút
  double dtm = dt / 60000.0; 
  double delta_ticks = encoder_ticks - prev_encoder_ticks_;

  prev_update_time_ = current_time;
  prev_encoder_ticks_ = encoder_ticks;

  return (delta_ticks / encoder_resolution_) / dtm;
}