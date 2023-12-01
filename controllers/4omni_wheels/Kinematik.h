#ifndef KINEMATIC_H
#define KINEMATIC_H

#include "vector"
#include "math.h"
#include "icecream.hpp"
#include "def.h"
namespace Kinematic
{

    class RobotKinematic
    {
    public:
        static RobotKinematic *getInstance();
        std::vector<double> encData = {0, 0, 0, 0};
        RobotKinematic(){
        };
        Point2D getPos();
        void ForwardKinematic(Point2D &outForward, float s1 , float s2 ,float s3 ,float s4);
        void CalculateOdometry(const double yaw);
        void inversKinematic(motion &outputInvers, float vx, float vy, float theta, const double ori);
        double angleNormalize(double angle);
        
        void setInitialPositon(float x, float y, float theta);

    private:
        Point2D pos = {0, 0, 0};
        Point2D vel = {0, 0, 0};

        static RobotKinematic *instance;
        float a = 45;  // Angle beetwen wheels
        float L = 20.8;    // radius from center to wheel (cm) //baseLength 2397.97 MM;
        float r_wheel = 6; // wheel radius  (cm)
        float circumrefrence = 2 * M_PI * r_wheel;
        std::vector<double> Venc = {0, 0, 0, 0};
        std::vector<double> prevEnc = {0, 0, 0, 0};
    };
};

#endif