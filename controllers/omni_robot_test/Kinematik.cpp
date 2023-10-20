#include "Kinematik.h"

using namespace Kinematic;

KinematicCoba *KinematicCoba::instance = 0;
Point2D posLocal;
Point2D output;
Point2D velGlobal;
Point2D velCoba;
motion mtr_;
Point2D vel;
// Point2D pos;
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
        // if (tick < 0.001)
        // {
        //     tick = 0;
        //     encData[i] = prevEnc[i];
        // }

        // Venc[i] = tick * r_wheel;
        // IC(tick);
        Venc[i] = tick * (circumrefrence / (2 * M_PI));
        // IC(dt);

        // phidot[i] = (Venc[i] / dt);
        // PosEnc[i] = PosEnc[i] + tick * ((2 * M_PI) / 25);
        // phidot[i] = (tick * ((2 * M_PI) / 25)) / dt;
    }
    // mtr_.w1 = Venc[0];
    // mtr_.w2 = Venc[1];
    // mtr_.w3 = Venc[2];
    // mtr_.w4 = Venc[3];

    // IC(PosEnc[0], PosEnc[1], PosEnc[2], PosEnc[3]);
    // IC(phidot[0], phidot[1], phidot[2], phidot[3]);
    // ForwardKinematic(output, mtr_);
    // IC(Venc[0], Venc[1], Venc[2], Venc[3]);

    float b = sqrt(2);
    output.x = ((-b * Venc[0]) - (b * Venc[1]) + (b * Venc[2]) + (b * Venc[3])) / 4;
    output.y = ((b * Venc[0]) - (b * Venc[1]) - (b * Venc[2]) + (b * Venc[3])) / 4;
    output.theta = (Venc[0] + Venc[1] + Venc[2] + Venc[3]) / (4 * L);
    IC(output.x, output.y);
    // float vb = sqrt(output.x * output.x + output.y * output.y);
    // IC(vb);

    

    //   IC(outForward.velx, outForward.vely, outForward.veltheta);
    // IC(pos.theta);
    double radian = fmod(pos.theta, M_PI * 2);
    if (radian < 0)
        radian += M_PI * 2;

    // IC(radian, sin(radian), cos(radian));
    // IC(output.x * sin(radian), output.y * cos(radian));
    pos.theta = radian;
    // posCobaTheta = radian2;

    // thetaPOS = pos.theta;

    // IC(output.x, output.y, output.theta);

    velGlobal.x = (std::cos(radian) * output.x) - (std::sin(radian) * output.y);
    velGlobal.y = (std::cos(radian) * output.y) + (std::sin(radian) * output.x);

    // float vgx = vb * cos(radian);
    // IC(vgx,velGlobal.x, velGlobal.y, sqrt(pow(velGlobal.x,2) + pow(velGlobal.y, 2)));

    // velGlobal.x = (std::cos(ori) * output.x) - (std::sin(ori) * output.y);
    // velGlobal.y = (std::cos(ori) * output.y) + (std::sin(ori) * output.x);

    // IC(velGlobal.x, velGlobal.y);
    // IC(dt);

    // posCobaX += velCoba.x;
    // posCobaY += velCoba.y;
    // posCobaTheta += output.theta;

    // IC(posCobaX, posCobaY, posCobaTheta);

    posLocal.x += output.x / 100;
    posLocal.y += output.y / 100;
    posLocal.theta += output.theta;

    pos.x += velGlobal.x / 100;
    pos.y += velGlobal.y / 100;
    pos.theta += output.theta;

    // pos.theta = ori;

    // IC(posLocal.x, posLocal.y, posLocal.theta);

    // IC(pos.x, pos.theta, pos.y);
    //   count++;
    //   IC(count);
    // IC(output.x, output.y, output.theta);

    prevEnc = encData;
}

Point2D KinematicCoba::getPos()
{
    return pos;
}

Point2D KinematicCoba::getPosLocal()
{
    return posLocal;
}

Point2D KinematicCoba::getOutVel()
{
    return output;
}
std::vector<double> KinematicCoba::getPhidot()
{
    return phidot;
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
        // fprintf(stderr, "LOCAL\n");
        vel.x = velglobal_x;
        vel.y = velglobal_y;
        vel.theta = velglobal_theta;
        break;

    case 1:
        // vel.x = cos(getCurrentPose().theta) * velglobal_x - sin(getCurrentPose().theta) * velglobal_y;
        // vel.y = -sin(getCurrentPose().theta) * velglobal_x + cos(getCurrentPose().theta) * velglobal_y;
        // //  vel.theta = velglobal_theta;
        // fprintf(stderr, "global\n");
        // vel.x = cos(ori) * velglobal_x + sin(ori) * velglobal_y;
        // vel.y = -sin(ori) * velglobal_x + cos(ori) * velglobal_y;
        // vel.theta = velglobal_theta;
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

    // IC(outputInvers.w1, outputInvers.w2, outputInvers.w3, outputInvers.w4);
}
