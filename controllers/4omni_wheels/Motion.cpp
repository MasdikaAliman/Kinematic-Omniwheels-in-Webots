#include "Motion.h"

Motion::Motion()
{
    pid_position = new PID();
    angular = new PID();
};

void Motion::accelControl(Point2D *out, Point2D &data)
{
    static float v_buffer[2];

    float delta_v[2];
    delta_v[0] = data.x - v_buffer[0];
    delta_v[1] = data.y - v_buffer[1];
    float r = sqrt(delta_v[0] * delta_v[0] + delta_v[1] * delta_v[1]);
    float theta = atan2(delta_v[1], delta_v[0]);
    if (r > 3) // maks acceleration
        r = 3;
    v_buffer[0] += r * cos(theta);
    v_buffer[1] += r * sin(theta);

    out->x = v_buffer[0];
    out->y = v_buffer[1];
    out->theta = data.theta;
    IC(out->x, out->y, out->theta);
};

void Motion::PositionAngularControl(double &errorX, double &errorY, double &errorTheta, double yaw, Point2D &outMotor)
{

    // error X
    error[0] = errorX;
    error[1] = errorY;
    error[2] = sqrt(error[0] * error[0] + error[1] * error[1]);
    error[3] = errorTheta;

    pid_position->setParam(15, 5, 8);

    output[2] = pid_position->PID_calculate(error[2], 100);

    angular->setParam(1, 0, 0);
    IC(error[3]);

    output[3] = angular->PID_calculate(error[3] * M_PI / 180.0, 5);
    output[0] = output[2] * cos(atan2(error[1], error[0]));
    output[1] = output[2] * sin(atan2(error[1], error[0]));

    // IC(output[0], output[1], output[2], output[3]);
    basicMotion(output[0], output[1], output[3], yaw, outMotor);
};

void Motion::basicMotion(float vx, float vy, float vtheta, float thetaRobot, Point2D &output)
{
    Point2D dataInput;
    float velout[3];
    velout[0] = cos(thetaRobot) * vx + sin(thetaRobot) * vy;
    velout[1] = -sin(thetaRobot) * vx + cos(thetaRobot) * vy;
    IC(thetaRobot);
    dataInput.x = velout[0];
    dataInput.y = velout[1];
    dataInput.theta = vtheta;
    accelControl(&output, dataInput);
};
