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

#define TIME_STEP 64
#define MAX_SPEED 6.28
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
  };
  struct SensData d[5]; //Variable Array

  //////////////////////////////////////
  // Sensor initialisation done
  
  double pitch = 0.0, roll = 0.0;
  const double alpha = 0.98;
  const double dt = TIME_STEP / 1000.0; // Convert to seconds
  
  while (robot->step(TIME_STEP) != -1) {
    // Get sensor data
    //read sensors
    for (int i = 0; i < 5; i++) {
        struct SensData *S_val = &d[i] ;//pointer to the variable
        S_val->num = i; 
        S_val->Val = (ds[i]->getValue());
        std::cout << std::to_string(ds[i] -> getValue()) << std::endl; 
    }
    const double *a = accel->getValues(); // ax, ay, az

    // Calculate angles from accelerometer
    const double *values = compass->getValues();
    double yaw = atan2(values[0], values[1]);

    std::cout << "Yaw: " << yaw * 180.0 / M_PI << std::endl;

    std::cout << "=======================" << std::endl;
    // Calculate geometry of maze ( straight, sharp turn )

    double wall_threshold = 300;     // <30 cm = wall
    double open_threshold = 500;     // >80 cm = open space

    // Sample left turn detection
    if (d[0].Val > open_threshold && d[1].Val > open_threshold &&
        d[2].Val < wall_threshold && d[3].Val < wall_threshold && d[4].Val < wall_threshold) {
      std::cout << "Left 90° turn detected!" << std::endl;
    }

    // Sample right turn detection
    if (d[4].Val > open_threshold && d[3].Val > open_threshold &&
        d[2].Val < wall_threshold && d[1].Val < wall_threshold && d[0].Val < wall_threshold) {
      std::cout << "Right 90° turn detected!" << std::endl;
    }


    // Use PID to control 

    double leftSpeed = MAX_SPEED ;
    double rightSpeed = MAX_SPEED ;
    if (d[0].Val > 300) { //left 90 sensor
      leftSpeed = 0.00;//turn right
      rightSpeed = -MAX_SPEED;
    } else if(d[2].Val > 300){ //Front sensor
         leftSpeed = 0.00;//turn right
         rightSpeed = -MAX_SPEED;
        
    }else if (d[1].Val>500 && d[3].Val>500){//left and Right Angled sensor
       leftSpeed = -MAX_SPEED ; //go Backwards
       rightSpeed = -MAX_SPEED ;
    }
    
    wheels[0]->setVelocity(leftSpeed);
    wheels[1]->setVelocity(rightSpeed);
   
  }
  delete robot;
  return 0;  // EXIT_SUCCESS
}