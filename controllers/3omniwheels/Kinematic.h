#ifndef KINEMATIC_3OMNI_H
#define KINEMATIC_3OMNI_H

#include "math.h"
#include "icecream.hpp"
#include "vector"
#include "def.h"
class Kinematic
{
public:
    Kinematic();
    std::vector<double> encData = {0, 0, 0};
    static Kinematic *getIstance();

    void calcOdometry(float a);
    void InversKinematic(float vx, float vy, float vtheta, motion_data &ret);
    void setInitialPositon(float x, float y ,float theta);
    Point2D getPosition();
    double deg2rad(double a);
    double rad2deg(double a);

private:
    Point2D pos = {0, 0, 0};
    std::vector<double> prevEnc = {0, 0, 0};
    std::vector<double> Venc = {0, 0, 0};
    float a = 30;
    float a1 = 150, a2 = 180, a3 = 210;
    static Kinematic *istance;
    float baseLength = 20.6, r_wheel = 6;

    Point2D output = {0, 0, 0};

    Point2D ForwardKinematic(float s1, float s2, float s3, float s4);
};

#endif