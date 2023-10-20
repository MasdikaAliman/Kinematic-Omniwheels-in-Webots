#ifndef DEF_H
#define DEF_H

struct motion
{
    float w1, w2, w3, w4;
};

struct Point2D
{
    Point2D() {}
    Point2D(float x, float y, float z) : x(x), y(y), theta(z) {}
    float x, y, theta;
};


#endif