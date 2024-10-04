#include "motor_l298_controller.h"

namespace diffbot {

L298MotorController::L298MotorController(int inA, int inB, int pwm) : inA_(inA), inB_(inB), pwm_(pwm) {
  // Set pin modes in the constructor (you might want to do this in your main code)
  pinMode(inA_, OUTPUT);
  pinMode(inB_, OUTPUT);
  pinMode(pwm_, OUTPUT);
}

void L298MotorController::setSpeed(int value) {
  if (value > 0) {
    digitalWrite(inA_, HIGH);
    digitalWrite(inB_, LOW);
  } else if (value < 0) {
    digitalWrite(inA_, LOW);
    digitalWrite(inB_, HIGH);
    value = -value; // Make negative value positive for PWM control
  } else {
    // Stop the motor
    digitalWrite(inA_, LOW);
    digitalWrite(inB_, LOW);
  }

  // Use analogWrite for PWM control (adjust range based on your motor driver)
  analogWrite(pwm_, abs(value));
}

} // namespace diffbot