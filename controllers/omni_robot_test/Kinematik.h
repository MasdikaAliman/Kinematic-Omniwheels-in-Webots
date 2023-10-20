#ifndef KINEMATIC_H
#define KINEMATIC_H

#include "vector"
#include "math.h"
#include "icecream.hpp"
#include "def.h"
namespace Kinematic
{

    class KinematicCoba
    {
    public:
        static KinematicCoba *getInstance();
        std::vector<double> encData = {0, 0, 0, 0};
        KinematicCoba();

        Point2D getPos();
        Point2D getPos_coba();
        Point2D getPosLocal();
        Point2D getOutVel();
        void ForwardKinematic(Point2D &outForward, motion &mtr);
        void CalculateOdometry(const double yaw);
        std::vector<double> getPhidot();
        void inversKinematic(motion &outputInvers, float vx, float vy, float theta, int type, const double ori);

    private:
        // float posCobaX = 0, posCobaY = 0, posCobaTheta = 0;
        Point2D pos = {0,0,0};


        static KinematicCoba *instance;
        float a = 45;
        float L = 20.8;    // radius from center to wheel (cm) //baseLength 2397.97 MM;
        float r_wheel = 6; // wheel radius  (cm)
        float circumrefrence = 2 * M_PI * r_wheel;
        std::vector<double> phidot = {0, 0, 0, 0};
        std::vector<double> PosEnc = {0, 0, 0, 0};
        std::vector<double> Venc = {0, 0, 0, 0};
        std::vector<double> prevEnc = {0, 0, 0, 0};
    };
};

#endif