#define USE_CLASSIC_ARDUINO_HARDWARE
#include <ros.h>
#include <Wire.h>
#include "diffbot_base_config.h"
#include "base_controller.h"
#include "motor_l298_controller.h" // Include the new header file

ros::NodeHandle nh;

using namespace diffbot;

// Define pin numbers for L298 motor driver (replace with your actual pins)
int motor_right_inA = 44;
int motor_right_inB = 45;
int motor_right_pwm = 4;

int motor_left_inA = 46;
int motor_left_inB = 47;
int motor_left_pwm = 13;

// Create L298 motor controller objects
L298MotorController motor_controller_right(motor_right_inA, motor_right_inB, motor_right_pwm);
L298MotorController motor_controller_left(motor_left_inA, motor_left_inB, motor_left_pwm);

BaseController<L298MotorController, int> base_controller(nh, &motor_controller_left, &motor_controller_right);


void setup()
{
  // No initialization needed for L298 motor controller (remove begin() calls)
  // base_controller.setup();
  // base_controller.init();

  Serial.begin(9600);
  nh.getHardware() -> setBaud(9600);
  nh.initNode();


  nh.loginfo("Initialize DiffBot Motor Controllers");
  // Set pin modes for L298 motor driver in your main code (optional)
  // pinMode(motor_right_inA, OUTPUT);
  // pinMode(motor_right_inB, OUTPUT);
  // pinMode(motor_right_pwm, OUTPUT);

  // pinMode(motor_left_inA, OUTPUT);
  // pinMode(motor_left_inB, OUTPUT);
  // pinMode(motor_left_pwm, OUTPUT);

  nh.loginfo("Setup finished");
}


void loop()
{
  static bool imu_is_initialized;

  // The main control loop for the base_conroller.
  // This block drives the robot based on a defined control rate
  ros::Duration command_dt = nh.now() - base_controller.lastUpdateTime().control;
  if (command_dt.toSec() >= ros::Duration(1.0 / base_controller.publishRate().control_, 0).toSec())
  {
    base_controller.read();
    base_controller.write();
    base_controller.lastUpdateTime().control = nh.now();
  }

  // This block stops the motor when no wheel command is received
  // from the high level hardware_interface::RobotHW
  command_dt = nh.now() - base_controller.lastUpdateTime().command_received;
  if (command_dt.toSec() >= ros::Duration(E_STOP_COMMAND_RECEIVED_DURATION, 0).toSec())
  {
    nh.logwarn("Emergency STOP");
    base_controller.eStop();
  }

  // This block publishes the IMU data based on a defined imu rate
  ros::Duration imu_dt = nh.now() - base_controller.lastUpdateTime().imu;
  if (imu_dt.toSec() >= base_controller.publishRate().period().imu_)
  {
    // Sanity check if the IMU is connected
    if (!imu_is_initialized)
    {
      //imu_is_initialized = initIMU();
      if(imu_is_initialized)
      {
        nh.loginfo("IMU Initialized");
      }
      else
      {
        nh.logfatal("IMU failed to initialize. Check your IMU connection.");
      }
    }
    else
    {
      //publishIMU();
    }
    base_controller.lastUpdateTime().imu = nh.now();
  }

  // This block displays the encoder readings. change DEBUG to 0 if you don't want to display
  if(base_controller.debug())
  {
    ros::Duration debug_dt = nh.now() - base_controller.lastUpdateTime().debug;
    if (debug_dt.toSec() >= base_controller.publishRate().period().debug_)
    {
      base_controller.printDebug();
      base_controller.lastUpdateTime().debug = nh.now();
    }
  }
}
  //