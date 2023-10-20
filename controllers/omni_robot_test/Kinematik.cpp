#include "Kinematik.h"

using namespace Kinematic;

KinematicCoba *KinematicCoba::instance = 0;

KinematicCoba *KinematicCoba::getInstance()
{
    if (instance == 0)
    {
        instance = new KinematicCoba();
    }
    return instance;
}

KinematicCoba::KinematicCoba()
{
    pos.theta = 45 * M_PI / 180;
};

void KinematicCoba::CalculateOdometry(const double ori)
{

    // IC(dt);
    for (int i = 0; i < 4; i++)
    {
        double tick = (encData[i] - prevEnc[i]);
        Venc[i] = tick * (circumrefrence / (2 * M_PI));
    }

    float b = sqrt(2);
    Point2D output;
    output.x = ((-b * Venc[0]) - (b * Venc[1]) + (b * Venc[2]) + (b * Venc[3])) / 4;
    output.y = ((b * Venc[0]) - (b * Venc[1]) - (b * Venc[2]) + (b * Venc[3])) / 4;
    output.theta = (Venc[0] + Venc[1] + Venc[2] + Venc[3]) / (4 * L);

    double radian = fmod(pos.theta, M_PI * 2);
    if (radian < 0)
        radian += M_PI * 2;

    pos.theta = radian;

    Point2D velGlobal;
    velGlobal.x = (std::cos(radian) * output.x) - (std::sin(radian) * output.y);
    velGlobal.y = (std::cos(radian) * output.y) + (std::sin(radian) * output.x);

    pos.x += velGlobal.x / 100;
    pos.y += velGlobal.y / 100;
    pos.theta += output.theta;

    prevEnc = encData;
}

Point2D KinematicCoba::getPos()
{
    return pos;
}



void KinematicCoba::ForwardKinematic(Point2D &outForward, motion &mtr_)
{
    float b = sqrt(2);
    outForward.x = ((-b * mtr_.w1) - (b * mtr_.w2) + (b * mtr_.w3) + (b * mtr_.w4)) / 4;
    outForward.y = ((b * mtr_.w1) - (b * mtr_.w2) - (b * mtr_.w3) + (b * mtr_.w4)) / 4;
    outForward.theta = (mtr_.w1 + mtr_.w2 + mtr_.w3 + mtr_.w4) / (4 * L);
}

void KinematicCoba::inversKinematic(motion &outputInvers, float velglobal_x, float velglobal_y, float velglobal_theta, int type, const double ori)
{

    switch (type)
    {
    case 0:
        vel.x = velglobal_x;
        vel.y = velglobal_y;
        vel.theta = velglobal_theta;
        break;

    case 1:
        vel.x = cos(pos.theta) * velglobal_x + sin(pos.theta) * velglobal_y;
        vel.y = -sin(pos.theta) * velglobal_x + cos(pos.theta) * velglobal_y;
        vel.theta = velglobal_theta;

        break;

    default:
        break;
    }
    outputInvers.w1 = (-cos(a * M_PI / 180) * vel.x + sin(a * M_PI / 180) * vel.y + L * vel.theta) / r_wheel;
    outputInvers.w2 = (-cos(a * M_PI / 180) * vel.x - sin(a * M_PI / 180) * vel.y + L * vel.theta) / r_wheel;
    outputInvers.w3 = (cos(a * M_PI / 180) * vel.x - sin(a * M_PI / 180) * vel.y + L * vel.theta) / r_wheel;
    outputInvers.w4 = (cos(a * M_PI / 180) * vel.x + sin(a * M_PI / 180) * vel.y + L * vel.theta) / r_wheel;

 
}
