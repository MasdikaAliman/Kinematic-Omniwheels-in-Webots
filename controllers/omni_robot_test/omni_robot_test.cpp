#include "webots/Motor.hpp"
#include "webots/Robot.hpp"
#include "webots/PositionSensor.hpp"
#include "webots/InertialUnit.hpp"
#include "iostream"
#include "Kinematik.h"
#include "control_pid.h"


using namespace webots;

#define maxVel 100

enum moveType
{
  LOCAL,
  GLOBAL,
  VELOCITY_LOCAL,
  VELOCITY_GLOBAL
};

Motor *motor[4];
PositionSensor *enc[4];
InertialUnit *imu_device;
std::vector<double> prevPulse{0, 0, 0, 0};
std::vector<double> encData{0, 0, 0, 0};


Point2D velocity;
Point2D velocityWorld;
Point2D posTarget;
Point2D posNow;

void setVel(motion &motor);


float outWheel1, outWheel2, outWheel3, outWheel4;
Point2D outPOS(0, 0, 0);
static int mode = 0;
using namespace Kinematic;
KinematicCoba coba;
// pid aptPID;
PID pid;
Point2D nowPos(0, 0, 0);
int count = 0;

std::vector<Point2D> targetPos = {Point2D(4.5, 0, 45)}; 


int main(int argc, char **argv)
{

  motion outInvers;
  // motion_return outMotion;
  Robot *robot = new Robot();


  Point2D outputPID(0, 0, 0);


  int stepTime = robot->getBasicTimeStep();
  imu_device = robot->getInertialUnit("IMU");
  imu_device->enable(stepTime);
  // Motor Initial
  char motorNames[4][8] = {"wheel1", "wheel2", "wheel3", "wheel4"};

  // set Motor to webots
  for (int i = 0; i < 4; i++)
  {
    motor[i] = robot->getMotor(motorNames[i]);
    motor[i]->setPosition(INFINITY);
    motor[i]->setVelocity(0);
  }

  // Encoder Initial
  char encNames[4][8] = {"pw1", "pw2", "pw3", "pw4"};

  for (int i = 0; i < 4; i++)
  {
    enc[i] = robot->getPositionSensor(encNames[i]);
    enc[i]->enable(stepTime);
  }

  while (robot->step(stepTime) != -1)
  {
    // std::cout << "Hello" <<std:: endl;
    const double *orientation = imu_device->getRollPitchYaw();
    const double yaw = orientation[2];
    nowPos = KinematicCoba::getInstance()->getPos();
    // motion velocity;
    velocity.x = 5;
    velocity.y = 5;
    velocity.theta = 0;
    mode = 0;

   
    IC(nowPos.x, nowPos.y, nowPos.theta);
    IC(count);
  

    outputPID = pid.goTo(nowPos, targetPos[count]);

    double errorX = fabs(targetPos[count].x - nowPos.x);
    double errorY = fabs(targetPos[count].y - nowPos.y);

    if (errorX < 0.07 && errorY < 0.07 && fabs(targetPos[count].theta - (nowPos.theta * 180 / M_PI)) < 8)
    {
      pid.reset();
      count++;
    }

    if (count > targetPos.size() - 1)
    {
      outputPID.x = 0;
      outputPID.y = 0;
      outputPID.theta = 0;
    }

    KinematicCoba::getInstance()->inversKinematic(outInvers, outputPID.x, outputPID.y, outputPID.theta, GLOBAL, yaw);
    setVel(outInvers);

    // Assign Enc val
    for (int i = 0; i < 4; i++)
    {
      KinematicCoba::getInstance()->encData[i] = enc[i]->getValue();
    }
    KinematicCoba::getInstance()->CalculateOdometry(yaw);
  
    IC(motor[0]->getVelocity(), motor[1]->getVelocity(), motor[2]->getVelocity(), motor[3]->getVelocity());
  }

  delete robot;


  return 0;
}

void setVel(motion &outMotor)
{
  motor[0]->setVelocity(outMotor.w1);
  motor[1]->setVelocity(outMotor.w2);
  motor[2]->setVelocity(outMotor.w3);
  motor[3]->setVelocity(outMotor.w4);
}

