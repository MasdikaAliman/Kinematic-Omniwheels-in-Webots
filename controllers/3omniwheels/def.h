#ifndef DEF_H
#define DEF_H

struct Point2D
{
    Point2D() {};
    Point2D(float x, float y, float z) : x(x), y(y), theta(z) {}
    float x, y, theta;
};

struct motion_data
{
    motion_data() : w1(0), w2(0), w3(0) {}
    float w1, w2, w3;
};


#endif