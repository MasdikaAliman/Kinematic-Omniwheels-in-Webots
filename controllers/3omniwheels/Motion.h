#ifndef MOTION_H
#define MOTION_H

// #include "Kinematic.h"
#include "def.h"
#include "icecream.hpp"
#include "vector"

class PID
{
private:
    float min_out, max_out, min_integral, max_integral;
    float output_speed;
    float proportional, integral, derivative, prev_error;
    float kp, ki, kd;
    const float time_pid = 0.01;

public:
    PID()
    {
        min_out = 0, max_out = 0,
        min_integral = 0, max_integral = 0;
        integral = 0, derivative = 0;
        prev_error = 0;
    };

    float PID_calculate(float error, float min_max)
    {

        min_out = min_integral = -min_max;
        max_out = max_integral = min_max;

        float proportional = kp * error;
        integral += ki * error * time_pid;
        derivative = kd * (error - prev_error) / time_pid;

        if (integral > max_integral)
            integral = max_integral;
        else if (integral < min_integral)
            integral = min_integral;

        output_speed = proportional + integral + derivative;
        if (output_speed > max_out)
            output_speed = max_out;
        else if (output_speed < min_out)
            output_speed = min_out;

        prev_error = error;
        return output_speed;
    };

    void setParam(float kp_, float ki_, float kd_)
    {
        kp = kp_;
        ki = ki_;
        kd = kd_;
    }
    void reset()
    {
        integral = 0, derivative = 0;
        prev_error = 0;
        output_speed = 0;
    }
};

class Motion
{

public:
    Motion();

public:
    PID *pid_position, *angular;
    // void PositionAngularControl(Point2D targetPos, Point2D &nowPos, Point2D &outMotor);
    void PositionAngularControl(double &errorX, double &errorY, double &errorTheta, double yaw, Point2D &outMotor);

private:
    std::vector<double> output = {0, 0, 0, 0};
    std::vector<double> error = {0, 0, 0, 0};
    void basicMotion(float vx, float vy, float vtheta, float thetaRobot, Point2D &output);
    void accelControl(Point2D *out, Point2D &data);
};

#endif
