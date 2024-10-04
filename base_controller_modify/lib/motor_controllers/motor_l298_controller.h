#ifndef MOTOR_L298_CONTROLLER_H
#define MOTOR_L298_CONTROLLER_H

#include "motor_controller_interface.h"

#include <Arduino.h> // Include Arduino library for pin manipulation

namespace diffbot {

  /** \brief Implementation of the MotorControllerIntf for the L298 motor driver
   * 
   * Implements the abstract setSpeed method from the MotorControllerIntf to control a motor
   * connected to the L298 motor driver.
   */
  class L298MotorController : public MotorControllerIntf<int> // Use int for L298 pin numbers
  {
  public:
      /** \brief Construct an \ref L298MotorController for a single motor
       * 
       * Specify the pins used for controlling the L298 motor driver:
       * - inA (IN1 pin on L298)
       * - inB (IN2 pin on L298)
       * - pwm (PWM pin on L298)
       */
      L298MotorController(int inA, int inB, int pwm);

      /** \brief No initialization required for L298, 
          * just set pin modes in your main code.
       */
      void begin(uint16_t freq = 0) {} // Remove override keyword

      /** \brief Set the speed of the motor using L298 pin controls.
       * 
       * Concrete implementation of the setSpeed interface method to control a single motor
       * connected to the L298 motor driver. The input parameter \p value ranges from -255 to 255.
       * Inside this method, the value is mapped between 0 and 255 and the sign is used to set the direction.
       * A positive \p value results in the motor spinning forward.
       * Negative \p value rotates the motor backward and
       * a zero value stops the motor.
       * 
       * \param value positive or negative value to set the direction
       * and speed of the motor.
       */
      void setSpeed(int value) override;

  private:
      int inA_; // IN1 pin on L298
      int inB_; // IN2 pin on L298
      int pwm_;  // PWM pin on L298
  };

}

#endif // MOTOR_L298_CONTROLLER_H