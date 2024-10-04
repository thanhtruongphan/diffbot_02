#ifndef DIFFBOT_ENCODER_H
#define DIFFBOT_ENCODER_H

#include <Encoder.h>

namespace diffbot
{
  struct JointState
  {
    double angular_position_;
    double angular_velocity_;
  };

  /** \brief  Class to read the angular wheel velocity from quadrature wheel encoders.
   * 
   * This class is composed of \ref ::Encoder, which is capable of reading rising and 
   * falling edges of two Hall effect signals. This yields the highest possible 
   * tick count (encoder resolution) from the encoders.
   */
  class Encoder
  {
  public:
    ::Encoder encoder;

    /** \brief Construct a diffbot::Encoder providing access to quadrature 
     * encoder ticks and angular joint velocity.
     * 
     * \param pin1 Pin of the first Hall effect sensor
     * \param pin2 Pin of the second Hall effect sensor
     * \param encoder_resolution number of tick counts for one full revolution 
     * of the wheel (not the motor shaft). Keep track of gear reduction ratio.
     */
    Encoder(uint8_t pin1, uint8_t pin2, int encoder_resolution);

    /** \brief Get revolutions per minute
     *
     * Calculates the wheels revolution per minute using the encoder ticks.
     * 
     * \returns revolutions per minute
     */
    int getRPM();

    double angularPosition();

    /** \brief Get the angular joint velocity
     *
     * Calculates the angular velocity of the wheel joint using the encoder ticks.
     * 
     * \returns angular wheel joint velocity (rad/s)
     */
    double angularVelocity();

    JointState jointState();

    /** \brief Convert number of encoder ticks to angle in radians 
     *
     * Calculates the current tick count of the encoder to its absolute angle 
     * in radians using the \ref encoder_resolution_.
     * 
     * \param ticks tick count from an encoder which is converted to a 
     * corresponding absolute angle.
     * 
     * \returns angle corresponding to encoder ticks (rad)
     */
    double ticksToAngle(const int &ticks) const;

    /** \brief Read the current encoder tick count
     * 
     * \returns encoder ticks
     */
    inline int32_t read() { return encoder.read(); };

    /** \brief Set the encoder tick count
     * 
     * Mainly used to reset the encoder back to zero.
     * 
     * \param p encoder ticks
     */
    inline void write(int32_t p) { encoder.write(p); };

    /** \brief Setter for encoder resolution
     * 
     * Used to initialize the encoder with a new resolution.
     * 
     * \param resolution value to which the encoder tick count should be set
     */
    inline void resolution(int resolution) { encoder_resolution_ = resolution; };

    /** \brief Getter for encoder resolution
     * 
     * Returns the currently set encoder resolution.
     */
    inline int resolution() { return encoder_resolution_; };

  private:
    // Number of tick counts for one full revolution of the wheel 
    // (not the motor shaft). Keep track of gear reduction ratio.
    int encoder_resolution_;

    JointState joint_state_;

    // Previous encoder tick count when the \ref getRPM or \ref angularVelocity 
    // method was called to calculated the delta tick count.
    long prev_encoder_ticks_ = 0;
  };
}

#endif // DIFFBOT_ENCODER_H