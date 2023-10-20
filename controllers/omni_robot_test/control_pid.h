#ifndef _PID_H
#define _PID_H

#include <sys/time.h>
#include "math.h"
#include "def.h"

struct ketetapan
{
    float kp_lin = 1, ki_lin = 0, kd_lin = 0;
    float kp_rot = 10, ki_rot = 0, kd_rot = 0;
};

class PID
{
public:
    PID();

    ketetapan config;

    int cartesian2polar_angleDegNormalize(float pointAx, float pointAy, float pointBx, float pointBy);
    float linVelPID(float error, ketetapan config, int max_lin_vel);
    float angularPID(float robot_angle, float target_angle, ketetapan config, int max_ang_vel);
    int movementDirection(int robot_angle, int reference_angle);
    Point2D goTo(Point2D pos, Point2D tar);


    float linear_velocity = 0;
    float dec_vel = 0;
    float setKetetapan(float kp, float ki, float kd);
    void reset()
    {
        lin_previous_error = 0;
        lin_integral = 0;
        rot_previous_error = 0;
        rot_integral = 0;
    };

private:

    float lin_previous_error;
    float lin_integral;
    struct timeval lin_start_timer, lin_stop_timer;
    struct timeval accelaration_start, accelaration_stop;

    float rot_previous_error, rot_integral;
    struct timeval rot_start_timer, rot_stop_timer;
};

#endif