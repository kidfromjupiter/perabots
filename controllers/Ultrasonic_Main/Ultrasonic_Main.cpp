#include <webots/DistanceSensor.hpp>
#include <webots/Motor.hpp>
#include <webots/Accelerometer.hpp>
#include <webots/Robot.hpp>
#include <webots/Gyro.hpp>
#include <webots/Compass.hpp>
#include <string>
#include <iomanip>
#include <cmath>
#include <iostream>
#include <webots/Keyboard.hpp>

#define TIME_STEP 64
#define MAX_SPEED 20
#define SPEED 5.0

using namespace webots;

int main(int argc, char **argv) {
  Robot *robot = new Robot();
  
  // Initializing sensors
  DistanceSensor *ds[5];
  Gyro *gyro = robot->getGyro("gyro");
  Compass *compass = robot -> getCompass("compass");
  Accelerometer *accel = robot->getAccelerometer("accelerometer");
  accel->enable(TIME_STEP);
  gyro->enable(TIME_STEP);
  compass -> enable(TIME_STEP);

  char dsNames[5][10] = {"L","FL","F","FR","R"};
  for (int i = 0; i < 5; i++) {
    ds[i] = robot->getDistanceSensor(dsNames[i]);
    ds[i]->enable(TIME_STEP);
  }
  Motor *wheels[2];
  char wheels_names[2][8] = {"motor1", "motor2"};
  for (int i = 0; i < 2; i++) {
    wheels[i] = robot->getMotor(wheels_names[i]);
    wheels[i]->setPosition(INFINITY);
    wheels[i]->setVelocity(0.0);
  }
  
  struct SensData{ //Structure to store sensor num and values
   int num;
   float Val;
    float dVal;
  };
  struct SensData prev_data[5]; //Variable Array
  struct SensData current_data[5]; //Variable Array

  double Kp = 0.05;   // Proportional gain
  double Ki = 0.0;   // Integral gain
  double Kd = 0.015;   // Derivative gain

  double error = 0, prevError = 0, integral = 0;


  //////////////////////////////////////
  // Sensor initialisation done
  
  double pitch = 0.0, roll = 0.0;
  const double alpha = 0.98;
  const double dt = TIME_STEP / 1000.0; // Convert to seconds

  Keyboard keyboard;
  keyboard.enable(TIME_STEP);

  // exponential moving average for rate of change
  const double ALPHA = 0.5;  // 0 < ALPHA <= 1. Lower = smoother
  double prevValues[5] = {0};
  double smoothedRate[5] = {0};

  
  while (robot->step(TIME_STEP) != -1) {
    // Get sensor data
    const double *a = accel->getValues(); // ax, ay, az

    for (int i = 0; i < 5; i++) {
      double current = ds[i]->getValue();
      double rate = (current - prevValues[i]) / (TIME_STEP / 1000.0);
      prevValues[i] = current;
  
      // Exponential moving average
      smoothedRate[i] = ALPHA * rate + (1 - ALPHA) * smoothedRate[i];
      std::cout << ds[i] -> getValue() << std::endl;
  
      //std::cout << dsNames[i] << ": " << smoothedRate[i] << " units/s" << std::endl;

    }
    //std::cout << "=======================" << std::endl;

    // Calculate angles from accelerometer
    const double *values = compass->getValues();
    double yaw = atan2(values[0], values[1]);

    //std::cout << "Yaw: " << yaw * 180.0 / M_PI << std::endl;

    // Calculate geometry of maze ( straight, sharp turn )

    double wall_threshold = 300;     // <30 cm = wall
    double open_threshold = 500;     // >80 cm = open space

    bool right_turn = false;
    bool left_turn = false;
    double left = ds[0]->getValue();   // L
    double right = ds[4]->getValue();  // R
  
    // Calculate deviation: positive if too close to right wall
    error = left - right;
    std::cout << error << std::endl;
  
    integral += error * (TIME_STEP / 1000.0);
    double derivative = (error - prevError) / (TIME_STEP / 1000.0);
  
    double correction = Kp * error + Ki * integral + Kd * derivative;
    prevError = error;
  
    // Base speed
    double baseSpeed = 7.0;
  
    // Apply correction to motors (assuming differential drive)
    double leftSpeed = baseSpeed - correction;
    double rightSpeed = baseSpeed + correction;
  
    wheels[0]->setVelocity(leftSpeed);
    wheels[1]->setVelocity(rightSpeed);

    if (smoothedRate[0] > 100.0) {
     std::cout << "Left turn starts here!" << std::endl;
    }
    if (smoothedRate[0] < -100.0) {
     std::cout << "Left wall here!" << std::endl;
    }
    if (smoothedRate[4] > 100.0) {
      std::cout << "Right turn starts here!" << std::endl;
    }
    if (smoothedRate[4] < -100.0) {
      std::cout << "Right wall here!" << std::endl;
    }

    if (ds[2] -> getValue() > 900){
      std::cout << "In straight section" << std::endl;
    }

    // Use PID to control 

    //int key = keyboard.getKey();

    //double leftSpeed = 0.0;
    //double rightSpeed = 0.0;

    //switch (key) {
    //  case Keyboard::UP:
    //    leftSpeed = SPEED;
    //    rightSpeed = SPEED;
    //    break;
    //  case Keyboard::DOWN:
    //    leftSpeed = -SPEED;
    //    rightSpeed = -SPEED;
    //    break;
    //  case Keyboard::LEFT:
    //    leftSpeed = -SPEED;
    //    rightSpeed = SPEED;
    //    break;
    //  case Keyboard::RIGHT:
    //    leftSpeed = SPEED;
    //    rightSpeed = -SPEED;
    //    break;
    //  default:
    //    // No key or unsupported key
    //    break;
    //}

    //wheels[0]->setVelocity(leftSpeed);
    //wheels[1]->setVelocity(rightSpeed);
   
  }
  delete robot;
  return 0;  // EXIT_SUCCESS
}