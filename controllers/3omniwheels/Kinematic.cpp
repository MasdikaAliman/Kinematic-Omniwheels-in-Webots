#include "Kinematic.h"

Kinematic *Kinematic::istance = 0;

Kinematic *Kinematic::getIstance()
{
    if (istance == 0)
        istance = new Kinematic();

    return istance;
}

Kinematic::Kinematic()
{
    // pos.x = -2.0;
    // pos.y = 0.0;

    // pos.theta = deg2rad(30);
}

void Kinematic::setInitialPositon(float x, float y, float theta)
{
    pos.x = x;
    pos.y = y;
    pos.theta = theta;
}

void Kinematic::InversKinematic(float vx, float vy, float vtheta, motion_data &ret)
{
    ret.w1 = ((-cos(deg2rad(a)) * vx) + (sin(deg2rad(a)) * vy) + (baseLength * vtheta)) / r_wheel;
    ret.w2 = ((-1 * vy) + (baseLength * vtheta)) / r_wheel;
    ret.w3 = ((cos(deg2rad(a)) * vx) + (sin(deg2rad(a)) * vy) + (baseLength * vtheta)) / r_wheel;
}

Point2D Kinematic::ForwardKinematic(float s1, float s2, float s3, float s4)
{
    Point2D outForward;
    float b = sqrt(3);

    outForward.x = ((-b * s1) + (b * s3)) / 3;
    outForward.y = ((-2 * s2) + s1 + s3) / 3;
    outForward.theta = (s1 + s2 + s3 + s4) / (3 * baseLength);

    return outForward;
}

double Kinematic::deg2rad(double a)
{
    return (a * M_PI / 180);
}
double Kinematic::rad2deg(double a)
{
    return (a * 180 / M_PI);
}

Point2D Kinematic::getPosition()
{
    return pos;
}

void Kinematic::calcOdometry(float a)
{
    for (int i = 0; i < 3; i++)
    {
        double tick = (encData[i] - prevEnc[i]);
        // IC(tick);
        Venc[i] = tick * r_wheel;
    }
    // IC(Venc[0], Venc[1], Venc[2], Venc[3]);

    output = ForwardKinematic(Venc[0], Venc[1], Venc[2], Venc[3]);

    // IC(output.x, output.y, output.theta);

    double radian = fmod(pos.theta, 2 * M_PI);
    if (radian < 0)
        radian += 2 * M_PI;

    pos.theta = radian;

    Point2D posGlobal;
    // posGlobal.x = cos(radian) * output.x - sin(radian) * output.y;
    // posGlobal.y = cos(radian) * output.y + sin(radian) * output.x;

    posGlobal.x = cos(a) * output.x - sin(a) * output.y;
    posGlobal.y = cos(a) * output.y + sin(a) * output.x;
    // IC(posGlobal.x, posGlobal.y, posGlobal.theta);

    pos.x += posGlobal.x / 100;
    pos.y += posGlobal.y / 100;
    // pos.theta += output.theta;
    pos.theta = a;

    IC(pos.x, pos.y, pos.theta);
    prevEnc = encData;
}
