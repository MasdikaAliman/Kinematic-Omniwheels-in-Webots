
#include "control_pid.h"
#include <icecream.hpp>

PID::PID()
{
    lin_previous_error = 0, lin_integral = 0;
    rot_previous_error = rot_integral = 0;
    config.kp_lin = 0.08;
    config.ki_lin = 3;
    config.kd_lin = 0.05;

    config.kp_rot = 40;
    config.kd_rot = 0.01;
    config.ki_rot = 1;
}

int PID::cartesian2polar_angleDegNormalize(float pointAx, float pointAy, float pointBx, float pointBy)
{
    int angleDegrees;

    angleDegrees = (int)((atan2((pointBy - pointAy), (pointBx - pointAx))) * 180.0 / M_PI);

    if (angleDegrees < 0)
        angleDegrees += 360;
    // else if(angleDegrees > )
    //     angleDegrees -;
    IC(angleDegrees);
    return angleDegrees;
}

float PID::linVelPID(float error, ketetapan config, int max_lin_vel)
{

    gettimeofday(&lin_stop_timer, NULL);

    float dt = (lin_stop_timer.tv_usec - lin_start_timer.tv_usec) / 1000000.0;
    if (dt < 0.0)
        dt = 0.03333;

    if (linear_velocity > max_lin_vel)
    {
        config.kp_lin = 20;
        config.kd_lin = 0.5;
        config.ki_lin = 0.05;
    }
    if (error < 1.0)
    {
        config.kp_lin = 30;
        config.ki_lin = 0.5;
        config.kd_lin = 0.05;
    }
    float proportionalAction = config.kp_lin * error;

    float integralAction = config.ki_lin * lin_integral;

    float derivative = (error - lin_previous_error) / dt;

    float derivativeAction = config.kd_lin * derivative;


    linear_velocity = (proportionalAction + integralAction + derivativeAction);

    if (linear_velocity > max_lin_vel)
    {
        linear_velocity = max_lin_vel;
    }
    else
        lin_integral += error * dt;

    lin_previous_error = error;

    gettimeofday(&lin_start_timer, NULL);

    return linear_velocity;
}

float PID::angularPID(float angle, float target_angle, ketetapan config, int max_ang_vel)
{
    float angular_velocity = 0;

    if (target_angle > 180)
        target_angle -= 360;

    float error = target_angle - angle;
    if (error > 180.0)
        error -= 360.0;
    if (error < -180.0)
        error += 360.0;

    if (abs(error) < 2)
        return 0;
    IC(angle, target_angle, error);

    gettimeofday(&rot_stop_timer, NULL);
    float dt = (rot_stop_timer.tv_usec - rot_start_timer.tv_usec) / 1000000.0;
    if (dt < 0.0)
        dt = 0.03333;

    float proportionalAction = config.kp_rot * error;
    if ((error * rot_previous_error) <= 0)
        rot_integral = 0.0;

    float integralAction = config.ki_rot * rot_integral;

    float derivative = (error - rot_previous_error) / dt;
    float derivativeAction = config.kd_rot * derivative;

    angular_velocity = (int)(proportionalAction + integralAction + derivativeAction);

    if (angular_velocity > max_ang_vel)
        angular_velocity = max_ang_vel;
    else if (angular_velocity < -max_ang_vel)
        angular_velocity = -max_ang_vel;
    else
        rot_integral += error * dt;

    rot_previous_error = error;

    gettimeofday(&rot_start_timer, NULL);

    return angular_velocity;
}

int PID::movementDirection(int angle, int reference_angle)
{
    int movement_direction = 360 - angle + reference_angle;

    if (movement_direction >= 360)
        movement_direction -= 360;
    IC(movement_direction);
    return movement_direction;
}
Point2D PID::goTo(Point2D pos, Point2D tar)
{
    Point2D ret(0, 0, 0);
    Point2D output;
    float posT;
    posT = pos.theta * 180 / M_PI;

    float errX = tar.x - pos.x;
    float errY = tar.y - pos.y;

    float error = sqrt(errX * errX + errY * errY);

    int target_angle = 0;
    // IC(tar.theta, tar.x, tar.y);
    target_angle = this->cartesian2polar_angleDegNormalize(pos.x, pos.y, tar.x, tar.y);

    float Ang = posT * M_PI / 180;

    float yaw = Ang < 0 ? Ang += M_PI : Ang;
    // IC(yaw);
    float moveDir = this->movementDirection(0, target_angle);
    IC(moveDir);
    float outTheta = this->angularPID(yaw * 180 / M_PI, tar.theta, config, 60);
    IC(outTheta);

    float vb = sqrt(tar.x * tar.x + tar.y * tar.y);

    float linVel = this->linVelPID(error, this->config, 100);
    IC(linVel);
    // sah.x = linVel * std::cos((moveDir)*M_PI / 180);
    // sah.y = linVel * std::sin((moveDir)*M_PI / 180);

    output.x = linVel * cos(atan2(errY, errX));
    output.y = linVel * sin(atan2(errY, errX));
    IC(output.x, output.y);

    // sah.x = linVel * std::cos((moveDir - yaw * 180 / M_PI) * M_PI / 180);
    // sah.y = linVel * std::sin((moveDir - yaw * 180 / M_PI) * M_PI / 180);
    output.theta = outTheta * M_PI / 180;
    IC(output.theta);

    coba(output, &ret);

    return output;
}

void PID::coba(Point2D &out, Point2D *ret)
{
    static float vbuffer[2];
    float delta_v[2];

    // IC(vbuffer[0], vbuffer[1]);
    // IC(delta_v[0], delta_v[1]);

    delta_v[0] = out.x - vbuffer[0];
    delta_v[1] = out.y - vbuffer[1];

    float r = sqrt(delta_v[0] * delta_v[1] + delta_v[1] * delta_v[1]);
    float theta = atan2(delta_v[1], delta_v[0]);

    float acceleration = 100;
    // IC(r, theta, acceleration);
    if (r > acceleration)
        r = acceleration;

    // IC(cos(theta), sin(theta));

    vbuffer[0] += r * cos(theta);
    vbuffer[1] += r * sin(theta);

    // printf("Velocity datas: %f %f %f\n", v_buffer[0], v_buffer[1], data->vel_th);
    // IC(vbuffer[0], vbuffer[1]);
    ret->x = vbuffer[0];
    ret->y = vbuffer[1];

    // IC(out.x, out.y, out.theta);
    // IC(ret->x, ret->y);
}
