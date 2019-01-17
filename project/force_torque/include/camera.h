#pragma once

#include <Eigen/Dense>
#include "traces.h"


class Camera
{

public:

    Camera();
    Camera(const Eigen::Vector3d &pos, const Eigen::Vector3d &dir, const Eigen::Vector3d &up);
    Eigen::Vector3d getEye();
    Eigen::Vector3d getUp();
    Eigen::Vector3d getCenter();
    void rotateX(double alpha);
    void rotateY(double alpha);
    void moveForward(double d);
    void moveBack(double d);
    void moveLeft(double d);
    void moveRight(double d);
    Eigen::Matrix3d getRMatrix(double alpha, const Eigen::Vector3d &v);
private:
    Eigen::Vector3d _up;
    Eigen::Vector3d _dir;
    Eigen::Vector3d _pos;

};


