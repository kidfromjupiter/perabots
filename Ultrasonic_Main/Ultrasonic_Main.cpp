#include <webots/DistanceSensor.hpp>
#include <webots/Motor.hpp>
#include <webots/Robot.hpp>

#define TIME_STEP 64
#define MAX_SPEED 6.28
using namespace webots;

int main(int argc, char **argv) {
  Robot *robot = new Robot();
  DistanceSensor *ds[5];
  char dsNames[5][10] = {"U1","U4","U3","U5","U2"};
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
  
  while (robot->step(TIME_STEP) != -1) {
    double leftSpeed = MAX_SPEED ;
    double rightSpeed = MAX_SPEED ;
    //read sensors
    for (int i = 0; i < 5; i++) {
        struct SensData *S_val = &d[i] ;//pointer to the variable
        S_val->num = i; 
        S_val->Val = (ds[i]->getValue());
      }
    if (d[0].Val < 900) { //left 90 sensor
      leftSpeed = 0.00;//turn right
      rightSpeed = -MAX_SPEED;
    } else if(d[2].Val < 900){ //Front sensor
         leftSpeed = 0.00;//turn right
         rightSpeed = -MAX_SPEED;
        
    }else if (d[1].Val<500 && d[3].Val<500){//left and Right Angled sensor
       leftSpeed = -MAX_SPEED ; //go Backwards
       rightSpeed = -MAX_SPEED ;
    }
    
    wheels[0]->setVelocity(leftSpeed);
    wheels[1]->setVelocity(rightSpeed);
   
  }
  delete robot;
  return 0;  // EXIT_SUCCESS
}