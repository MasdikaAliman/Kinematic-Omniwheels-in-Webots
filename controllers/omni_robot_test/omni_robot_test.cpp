#include "webots/Motor.hpp"
#include "webots/Robot.hpp"
#include "webots/PositionSensor.hpp"
#include "webots/InertialUnit.hpp"
#include "iostream"
// #include "icecream.hpp"
#include "Kinematik.h"
// #include "PID.h"
#include "control_pid.h"
// #include "PIDtes.h"
// #include "motion.h"
// #define baseLength 20.6 // cm

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
std::vector<double> Venc{0, 0, 0, 0};
std::vector<double> phidotData{0, 0, 0, 0};

Point2D velocity;
Point2D velocityWorld;
Point2D posTarget;
Point2D posNow;

void setVel(motion &motor);
// void moveTo(Point2D &tarPos, int type, PIDparam *K, PIDparam *KT, motion &outInvers, double ori);

float outWheel1, outWheel2, outWheel3, outWheel4;
Point2D outPOS(0, 0, 0);
static int mode = 0;
using namespace Kinematic;
KinematicCoba coba;
// pid aptPID;
PID pid;
Point2D nowPos(0, 0, 0);
int count = 0;
// std::vector<Point2D> targetPos = {Point2D(3, 0, 0), Point2D(2, -2, 0), Point2D(-2.5, 0, 0), Point2D(0, -2.5, 0), Point2D(-2.5, 2.0, 0)};

// cobaPID pidTes;

// MotionControl cobaMotion;
std::vector<Point2D> targetPos = {Point2D(4.5, 0, 45)}; 
// Point2D(-3, -3, 90), Point2D(0, 3, 90), Point2D(1, 0, 90)};
// motion_data targetData;
// std::vector<motion_data> targetData = {motion_data(3.5, 0.0, 45), motion_data(-3.5, 2.0, 45), motion_data(-3.0, 0.0, 45)};


int main(int argc, char **argv)
{

  motion outInvers;
  // motion_return outMotion;
  Robot *robot = new Robot();
  // aptPID.XY->setMax_PID(maxVel, 0.001);
  // aptPID.XY->setTunning(0.4, 0.003, 0.1);

  // aptPID.Local->setMax_PID(maxVel, 5);
  // aptPID.Local->setTunning(0.4, 0.003, 0.1);

  // aptPID.Theta->setTunning(0.4, 0, 0.2);
  // aptPID.Theta->setMax_PID(maxVel, 1);

  // PIDparam k(0.05, 0.1, 0.5);
  // PIDparam kt(0.4, 0, 0.3);

  // pidTes.setMaxPID(100, 0.5);
  // pidTes.setTunning(0.05, 0.1, 1, 0);

  Point2D outputPID(0, 0, 0);
  // PID *pid = new PID();

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

    // float errorX = targetPos[0].x - nowPos.x;
    // float errorY = targetPos[0].y - nowPos.y;

    // // IC(errorX, errorY);

    // float error = sqrt(pow(errorX, 2) + pow(errorY, 2));

    // // IC(error);

    // float target_angle = pidTes.cartesian2polar_angleDegNormalize(nowPos.x, nowPos.y,
    //                                                               targetPos[count].x, targetPos[count].y);

    // // IC(target_angle);

    // int moveDir = pidTes.movementDirection(0, target_angle);

    // int outLinear = pidTes.linVelPID(error);

    // int outX = outLinear * cos((moveDir)*M_PI / 180);
    // int outY = outLinear * sin((moveDir)*M_PI / 180);

    // outPOS.theta = 0;
    // // IC(outX, outY);

    // if (abs(targetPos.at(count).x - nowPos.x) < 10 && abs(targetPos.at(count).y - nowPos.y) < 10)

    // // abs(targetPos[count].theta - (nowPos.theta * 180 / M_PI) < 10))
    // {
    //   // pidTes.reset();
    //   outY = 0;
    //   outX = 0;
    // }
    // targetData.target_x = -2;
    // targetData.target_y = -2;
    // targetData.target_th = 0.785;
    // targetData.acceleration = 10;

    // targetData.vel_x = 15;
    // targetData.vel_y = 0;
    // targetData.vel_th = 0;

    // cobaMotion.ManualMotion(&targetData, &outMotion);

    // Vel with Vector
    // cobaMotion.ManualMotionVector(10, 0, 180, nowPos.theta, &outMotion);

    // POSITION CONTROL
    IC(nowPos.x, nowPos.y, nowPos.theta);
    IC(count);
    // cobaMotion.PositionAngularMotion(&targetData[count], &outMotion, nowPos);

    // // IC(outMotion.vx, outMotion.vy, outMotion.vtheta);
    // float distance = sqrt(pow((targetData[count].target_x - nowPos.x), 2) + pow((targetData[count].target_y - nowPos.y), 2));
    // IC(distance);

    // // if (fabs(targetData[count].target_x - nowPos.x) < 0.1 && fabs(targetData[count].target_y - nowPos.y) < 0.1
    // if(abs(distance) < 0.1
    // && abs(targetData[count].target_th - (nowPos.theta * 180 / M_PI)) < 10)
    // {
    //   fprintf(stderr, "REACH TARGET \n");

    //   count++;
    // }

    // if (count >= targetData.size())
    // {
    //   IC("masuk");
    //   // targetData[count].target_x = 0;
    //   // targetData[count].target_y = 0;
    //   // targetData[count].target_th = 0;
    //   outMotion.vx = 0;
    //   outMotion.vy = 0;
    //   outMotion.vtheta = 0;
    //   // break;
    // }

    // KinematicCoba::getInstance()->inversKinematic(outInvers, outMotion.vx, outMotion.vy, outMotion.vtheta, GLOBAL, yaw);
    // setVel(outInvers);
    // if (mode == 0)
    // {
    //   posTarget.x = -200;
    //   posTarget.y = 100
    //   posTarget.theta = 0 * (M_PI / 180);

    //   if (abs(posTarget.x - KinematicCoba::getInstance()->getPos().x) < 5 && abs(posTarget.y - KinematicCoba::getInstance()->getPos().y) < 5)
    //   {
    //     fprintf(stderr, "STOP POSE");
    //     // outInvers.w1 = 0;
    //     // outInvers.w2 = 0;
    //     // outInvers.w3 = 0;
    //     // outInvers.w4 = 0;

    //     // setVel(outInvers);
    //     // mode = 2;
    //     IC(mode);
    //   }
    //   moveTo(posTarget, GLOBAL, &k, &kt, outInvers, yaw);
    // }
    // IC(count);

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

    // KinematicCoba::getInstance()->inversKinematic(outInvers, velocity.x, velocity.y, velocity.theta, GLOBAL, yaw);
    // setVel(outInvers);

    // moveTo(targetPos[count], GLOBAL, &k, &kt, outInvers, yaw);
    // if (mode == 2)
    // {
    //   posTarget.x = 0;
    //   posTarget.y = 0;
    //   posTarget.theta = 0;

    //   if (abs(posTarget.x - KinematicCoba::getInstance()->getPos().x) < 5 && abs(posTarget.y - KinematicCoba::getInstance()->getPos().y) < 5)
    //   {
    //     fprintf(stderr, "STOP POSE");
    //     outInvers.w1 = 0;
    //     outInvers.w2 = 0;
    //     outInvers.w3 = 0;
    //     outInvers.w4 = 0;

    //     setVel(outInvers);
    //     // mode = 2;
    //     // IC(mode);
    //   }
    //   moveTo(posTarget, GLOBAL, &k, &kt, outInvers, yaw);
    // }
    // posTarget.x = 100;
    // posTarget.y = 0;
    // posTarget.theta = 0;
    // moveTo(posTarget, GLOBAL, &k, &kt, outInvers, yaw);

    // moveTo(velocity, VELOCITY_GLOBAL, &k, &kt, outInvers,yaw);

    // Assign Enc val
    for (int i = 0; i < 4; i++)
    {
      KinematicCoba::getInstance()->encData[i] = enc[i]->getValue();
    }
    // coba.CalculateOdometry(yaw);
    KinematicCoba::getInstance()->CalculateOdometry(yaw);
    // calculateOdom(yaw);

    // IC(thetaGlobal);

    // IC(pos.x, pos.y, pos.theta);
    // tesKinematik.calculateOdom(&wheelRobot, &kordinatRobot, &outputforward);
    // IC(wheelRobot.encData[0], wheelRobot.encData[1], wheelRobot.encData[2], wheelRobot.encData[3]);
    // IC(kordinatRobot.x, kordinatRobot.y, kordinatRobot.theta);
    IC(motor[0]->getVelocity(), motor[1]->getVelocity(), motor[2]->getVelocity(), motor[3]->getVelocity());
  }

  // delete speedControl1;
  delete robot;
  // delete pid;

  return 0;
}

void setVel(motion &outMotor)
{
  motor[0]->setVelocity(outMotor.w1);
  motor[1]->setVelocity(outMotor.w2);
  motor[2]->setVelocity(outMotor.w3);
  motor[3]->setVelocity(outMotor.w4);
}

// void moveTo(Point2D &tarPos, int type, PIDparam *K, PIDparam *Kt, motion &outInvers, double ori)
// {
//   float outX, outY, outTheta;
//   float errX, errY;

//   outTheta = 0;

//   // int target_angle = 0;
//   // // IC(tar.theta, tar.x, tar.y);
//   // target_angle = aptPID.XY->cartesian2polar_angleDegNormalize(KinematicCoba::getInstance()->getPos().x, KinematicCoba::getInstance()->getPos().y, tarPos.x, tarPos.y);
//   // IC(target_angle);

//   // float moveDir = aptPID.XY->movementDirection(0, target_angle);
//   // aptPID.XY->setTunning(1, 0, 0);
//   // aptPID.Local->setTunning(0.5, 0, 0.1);
//   // aptPID.Theta->setTunning(1, 0, 0);

//   if (K != NULL)
//   {
//     // aptPID.XY->setTunning(K->kp, K->ki, K->kd);
//     // aptPID.Local->setTunning(K->kp, K->ki, K->kd);
//   }
//   if (Kt != NULL)
//     // aptPID.Theta->setTunning(Kt->kp, Kt->ki, Kt->kd);

//   switch (type)
//   {
//   case LOCAL:
//     errX = tarPos.x - KinematicCoba::getInstance()->getPosLocal().x;
//     errY = tarPos.y - KinematicCoba::getInstance()->getPosLocal().y;
//     // aptPID.PID_Local(errX, errY, outX, outY);
//     // outTheta = aptPID.PID_theta(tarPos.theta);
//     KinematicCoba::getInstance()->inversKinematic(outInvers, outX, outY, outTheta, LOCAL, ori);
//     break;

//   case GLOBAL:
//     // errX = tarPos.x - KinematicCoba::getInstance()->getPos().x;
//     // IC(errY);
//     // IC(errX);
//     // errY = tarPos.y - KinematicCoba::getInstance()->getPos().y;

//     // aptPID.PID_XY(errX, errY, outX, outY);
//     // outX = outX * std::cos((moveDir)*M_PI / 180);
//     // outY = outY * std::sin((moveDir)*M_PI / 180);

//     IC(outX, outY);
//     // outTheta = aptPID.PID_theta(tarPos.theta - KinematicCoba::getInstance()->getPos().theta);
//     KinematicCoba::getInstance()->inversKinematic(outInvers, outX, outY, outTheta, GLOBAL, ori);
//     break;

//   case VELOCITY_GLOBAL:
//     KinematicCoba::getInstance()->inversKinematic(outInvers, tarPos.x, tarPos.y, tarPos.theta, GLOBAL, ori);
//     break;

//   case VELOCITY_LOCAL:
//     KinematicCoba::getInstance()->inversKinematic(outInvers, tarPos.x, tarPos.y, tarPos.theta, LOCAL, ori);
//     break;
//   default:
//     break;
//   }

//   setVel(outInvers);
// }