#ifndef DIFFBOT_BASE_CONTROLLER_H
#define DIFFBOT_BASE_CONTROLLER_H

#define USE_CLASSIC_ARDUINO_HARDWARE
#include <ros.h>
#include <diffbot_msgs/EncodersStamped.h>
#include <diffbot_msgs/WheelsCmdStamped.h>
#include <diffbot_msgs/AngularVelocities.h>
#include <diffbot_msgs/PIDStamped.h>
#include <std_msgs/Empty.h>
#include <sensor_msgs/JointState.h>

#include "diffbot_base_config.h"
#include "encoder_diffbot.h"
#include "motor_controller_interface.h"
// #include "adafruit_feather_wing/adafruit_feather_wing.h"
#include "pid.h"

namespace diffbot {

    template <typename TMotorController, typename TMotorDriver>
    class BaseController
    {
    public:

        BaseController(ros::NodeHandle &nh, TMotorController* motor_controller_left, TMotorController* motor_controller_right);

        struct UpdateRate
        {
            double imu_;
            double control_;
            double debug_;

            struct Period
            {
                double imu_;
                double control_;
                double debug_;

                inline Period(double imu_frequency, double control_frequency, double debug_frequency)
                    : imu_(1.0 / imu_frequency)
                    , control_(1.0 / control_frequency)
                    , debug_(1.0 / debug_frequency) {};
            } period_;

            inline Period& period() { return period_; };

            inline UpdateRate(double imu_frequency,
                               double control_frequency,
                               double debug_frequency)
                : imu_(imu_frequency)
                , control_(control_frequency)
                , debug_(debug_frequency)
                , period_(imu_frequency, control_frequency, debug_frequency) {};
        } update_rate_;

        inline int period(double frequency) { return 1 / frequency; };

        inline UpdateRate& publishRate() { return update_rate_; };

        struct LastUpdateTime
        {
            // Time when the last angular wheel command velocity was received.
            ros::Time command_received;
            // Time when the last control update happened.
            ros::Time control;
            // Time when the last imu update took place.
            ros::Time imu;
            // Time when the last debug message was logged.
            ros::Time debug;

            inline LastUpdateTime(ros::Time start)
                : command_received(start.toSec(), start.toNsec())
                , control(start.toSec(), start.toNsec())
                , imu(start.toSec(), start.toNsec())
                , debug(start.toSec(), start.toNsec()) {};
        } last_update_time_;

        inline LastUpdateTime& lastUpdateTime() { return last_update_time_; };


        inline bool debug() { return debug_; };

        void setup();

        void init();

        void eStop();

        void read();

        void write();
        void printDebug();

        void commandCallback(const diffbot_msgs::WheelsCmdStamped& cmd_msg);

        void resetEncodersCallback(const std_msgs::Empty& reset_msg);

        void pidLeftCallback(const diffbot_msgs::PIDStamped& pid_msg);

        void pidRightCallback(const diffbot_msgs::PIDStamped& pid_msg);

    private:
        // Reference to global node handle from main.cpp
        ros::NodeHandle& nh_;

        // constants
        float wheel_radius_ = 0.0;
        float max_linear_velocity_ = 0.0;
        float max_angular_velocity_ = 0.0;

        diffbot::Encoder encoder_left_;
        diffbot::Encoder encoder_right_;
        long ticks_left_ = 0, ticks_right_ = 0;

        // Measured left and right joint states (angular position (rad) and angular velocity (rad/s))
        diffbot::JointState joint_state_left_, joint_state_right_;

        int encoder_resolution_;

        ros::Subscriber<std_msgs::Empty, BaseController<TMotorController, TMotorDriver>> sub_reset_encoders_;

        diffbot_msgs::EncodersStamped encoder_msg_;
        ros::Publisher pub_encoders_;

        sensor_msgs::JointState msg_measured_joint_states_;
        ros::Publisher pub_measured_joint_states_;

        MotorControllerIntf<TMotorDriver>* p_motor_controller_right_;
        MotorControllerIntf<TMotorDriver>* p_motor_controller_left_;

        ros::Subscriber<diffbot_msgs::WheelsCmdStamped, BaseController<TMotorController, TMotorDriver>> sub_wheel_cmd_velocities_;
        float wheel_cmd_velocity_left_ = 0.0;
        float wheel_cmd_velocity_right_ = 0.0;

        int motor_cmd_left_ = 0;
        int motor_cmd_right_ = 0;

        ros::Subscriber<diffbot_msgs::PIDStamped, BaseController<TMotorController, TMotorDriver>> sub_pid_left_;
        ros::Subscriber<diffbot_msgs::PIDStamped, BaseController<TMotorController, TMotorDriver>> sub_pid_right_;
        PID motor_pid_left_;
        PID motor_pid_right_;

        // DEBUG
        bool debug_;
    };


}

template <typename TMotorController, typename TMotorDriver>
using BC = diffbot::BaseController<TMotorController, TMotorDriver>;


template <typename TMotorController, typename TMotorDriver>
diffbot::BaseController<TMotorController, TMotorDriver>
    ::BaseController(ros::NodeHandle &nh, TMotorController* motor_controller_left, TMotorController* motor_controller_right)
    : nh_(nh)
    , encoder_left_(nh, ENCODER_LEFT_H1, ENCODER_LEFT_H2, ENCODER_RESOLUTION)
    , encoder_right_(nh, ENCODER_RIGHT_H1, ENCODER_RIGHT_H2, ENCODER_RESOLUTION)
    , sub_reset_encoders_("reset", &BC<TMotorController, TMotorDriver>::resetEncodersCallback, this)
    , pub_encoders_("encoder_ticks", &encoder_msg_)
    , pub_measured_joint_states_("measured_joint_states", &msg_measured_joint_states_)
    , sub_wheel_cmd_velocities_("wheel_cmd_velocities", &BC<TMotorController, TMotorDriver>::commandCallback, this)
    , last_update_time_(nh.now())
    , update_rate_(UPDATE_RATE_IMU, UPDATE_RATE_CONTROL, UPDATE_RATE_DEBUG)
    , sub_pid_left_("pid_left", &BC<TMotorController, TMotorDriver>::pidLeftCallback, this)
    , sub_pid_right_("pid_right", &BC<TMotorController, TMotorDriver>::pidRightCallback, this)
    , motor_pid_left_(PWM_MIN, PWM_MAX, K_P, K_I, K_D)
    , motor_pid_right_(PWM_MIN, PWM_MAX, K_P, K_I, K_D)
{
    p_motor_controller_left_ = motor_controller_left;
    p_motor_controller_right_ = motor_controller_right;
}


template <typename TMotorController, typename TMotorDriver>
void diffbot::BaseController<TMotorController, TMotorDriver>::setup()
{
    nh_.initNode();
    nh_.advertise(pub_encoders_);

    msg_measured_joint_states_.position = (float*)malloc(sizeof(float) * 2);
    msg_measured_joint_states_.position_length = 2;
    msg_measured_joint_states_.velocity = (float*)malloc(sizeof(float) * 2);
    msg_measured_joint_states_.velocity_length = 2;
    nh_.advertise(pub_measured_joint_states_);

    nh_.subscribe(sub_wheel_cmd_velocities_);
    nh_.subscribe(sub_reset_encoders_);

    nh_.subscribe(sub_pid_left_);
    nh_.subscribe(sub_pid_right_);

    while (!nh_.connected())
    {
        nh_.spinOnce();
    }
}

template <typename TMotorController, typename TMotorDriver>
void diffbot::BaseController<TMotorController, TMotorDriver>::init()
{
    nh_.loginfo("Get Parameters from Parameter Server");
    nh_.getParam("/diffbot/encoder_resolution", &this->encoder_resolution_);
    String log_msg = String("/diffbot/encoder_resolution: ") + String(encoder_resolution_);
    nh_.loginfo(log_msg.c_str());
    nh_.getParam("/diffbot/mobile_base_controller/wheel_radius", &wheel_radius_);
    log_msg = String("/diffbot/mobile_base_controller/wheel_radius: ") + String(wheel_radius_);
    nh_.loginfo(log_msg.c_str());
    nh_.getParam("/diffbot/mobile_base_controller/linear/x/max_velocity", &max_linear_velocity_);
    log_msg = String("/diffbot/mobile_base_controller/linear/x/max_velocity: ") + String(max_linear_velocity_);
    nh_.loginfo(log_msg.c_str());
    nh_.getParam("/diffbot/debug/base_controller", &debug_);
    log_msg = String("/diffbot/debug/base_controller: ") + String(debug_);
    nh_.loginfo(log_msg.c_str());

    nh_.loginfo("Initialize DiffBot Wheel Encoders");
    encoder_left_.resolution(encoder_resolution_);
    encoder_right_.resolution(encoder_resolution_);

    std_msgs::Empty reset;
    this->resetEncodersCallback(reset);
    delay(1);

    max_angular_velocity_ = max_linear_velocity_ / wheel_radius_;

    delay(1000);
}

template <typename TMotorController, typename TMotorDriver>
void diffbot::BaseController<TMotorController, TMotorDriver>::commandCallback(const diffbot_msgs::WheelsCmdStamped& cmd_msg)
{
    wheel_cmd_velocity_left_ = cmd_msg.wheels_cmd.angular_velocities.joint[0];
    wheel_cmd_velocity_right_ = cmd_msg.wheels_cmd.angular_velocities.joint[1];

    lastUpdateTime().command_received = nh_.now();
}

template <typename TMotorController, typename TMotorDriver>
void diffbot::BaseController<TMotorController, TMotorDriver>::resetEncodersCallback(const std_msgs::Empty& reset_msg)
{
    // reset both back to zero.
    this->encoder_left_.write(0);
    this->encoder_right_.write(0);
    this->nh_.loginfo("Reset both wheel encoders to zero");
}


template <typename TMotorController, typename TMotorDriver>
void diffbot::BaseController<TMotorController, TMotorDriver>::pidLeftCallback(const diffbot_msgs::PIDStamped& pid_msg)
{
    motor_pid_left_.updateConstants(pid_msg.pid.kp, pid_msg.pid.ki, pid_msg.pid.kd);
    motor_pid_left_.updateConstants(pid_msg.pid.kp, pid_msg.pid.ki, pid_msg.pid.kd);
}

template <typename TMotorController, typename TMotorDriver>
void diffbot::BaseController<TMotorController, TMotorDriver>::pidRightCallback(const diffbot_msgs::PIDStamped& pid_msg)
{
    motor_pid_left_.updateConstants(pid_msg.pid.kp, pid_msg.pid.ki, pid_msg.pid.kd);
    motor_pid_left_.updateConstants(pid_msg.pid.kp, pid_msg.pid.ki, pid_msg.pid.kd);
}


template <typename TMotorController, typename TMotorDriver>
void diffbot::BaseController<TMotorController, TMotorDriver>::read()
{
    joint_state_left_ = encoder_left_.jointState();
    joint_state_right_ = encoder_right_.jointState();

    msg_measured_joint_states_.position[0] = joint_state_left_.angular_position_;
    msg_measured_joint_states_.position[1] = joint_state_right_.angular_position_;

    msg_measured_joint_states_.velocity[0] = joint_state_left_.angular_velocity_;
    msg_measured_joint_states_.velocity[1] = joint_state_right_.angular_velocity_;
    pub_measured_joint_states_.publish(&msg_measured_joint_states_);

    // get the current tick count of each encoder
    ticks_left_ = encoder_left_.read();
    ticks_right_ = encoder_right_.read();

    encoder_msg_.encoders.ticks[0] = ticks_left_;
    encoder_msg_.encoders.ticks[1] = ticks_right_;
}

template <typename TMotorController, typename TMotorDriver>
void diffbot::BaseController<TMotorController, TMotorDriver>::write()
{
    motor_cmd_left_ = motor_pid_left_.compute(wheel_cmd_velocity_left_, joint_state_left_.angular_velocity_);
    motor_cmd_right_ = motor_pid_right_.compute(wheel_cmd_velocity_right_, joint_state_right_.angular_velocity_);

    p_motor_controller_left_->setSpeed(motor_cmd_left_);
    p_motor_controller_right_->setSpeed(motor_cmd_right_);
}

template <typename TMotorController, typename TMotorDriver>
void diffbot::BaseController<TMotorController, TMotorDriver>::eStop()
{
    wheel_cmd_velocity_left_ = 0;
    wheel_cmd_velocity_right_ = 0;
}

template <typename TMotorController, typename TMotorDriver>
void diffbot::BaseController<TMotorController, TMotorDriver>::printDebug()
{
    String log_msg =
            String("\nRead:\n") +
                String("ticks_left_: ") + String(ticks_left_) +
                String("\nticks_right_: ") + String(ticks_right_) +
                String("\nmeasured_ang_vel_left: ") + String(joint_state_left_.angular_velocity_) +
                String("\nmeasured_ang_vel_right: ") + String(joint_state_right_.angular_velocity_) +
                String("\nwheel_cmd_velocity_left_: ") + String(wheel_cmd_velocity_left_) +
                String("\nwheel_cmd_velocity_right_: ") + String(wheel_cmd_velocity_right_) +

            String("\nWrite:\n") +
                String("motor_cmd_left_: ") + String(motor_cmd_left_) +
                String("\nmotor_cmd_right_: ") + String(motor_cmd_right_) +
                String("\npid_left_errors (p, i, d): ") + String(motor_pid_left_.proportional()) + String(" ") + String(motor_pid_left_.integral()) + String(" ") + String(motor_pid_left_.derivative()) +
                String("\npid_right_error (p, i, d): ") + String(motor_pid_right_.proportional()) + String(" ") + String(motor_pid_right_.integral()) + String(" ") + String(motor_pid_right_.derivative());
    nh_.loginfo(log_msg.c_str());
}


#endif // DIFFBOT_BASE_CONTROLLER_H