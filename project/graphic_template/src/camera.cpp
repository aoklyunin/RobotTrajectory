#include "camera.h"


Camera::Camera()
{
    _dir << 1, 0, 0;
    _pos << 0, 0, 0;
    _up << 0, 0, 1;
}

Eigen::Vector3d Camera::getEye()
{
    return _dir + _pos;
}
Eigen::Vector3d Camera::getUp()
{
    return _up;
}
Eigen::Vector3d Camera::getCenter()
{
    return _pos;
}
void Camera::rotateX(double alpha)
{

    Eigen::Matrix3d R = getRMatrix(alpha, _up);
    _dir = R * _dir;

    Eigen::Matrix3d Rup = getRMatrix(alpha, Eigen::Vector3d(0, 0, 1));
    _up = Rup * _up;
}

void Camera::rotateY(double alpha)
{
    double cDir = _dir.dot(Eigen::Vector3d(0, 0, 1));

    // forbid to look strictly upward and strictly down

   // if ((cDir < 0.7 && alpha < 0) || (cDir > -0.7 && alpha > 0)) {

    Eigen::Vector3d r = _up.cross(_dir).normalized();
    Eigen::Matrix3d R = getRMatrix(alpha, r);

        _up = R * _up;
        _dir = R * _dir;
   // }

}
void Camera::moveRight(double d)
{
    Eigen::Vector3d r = _up.cross(_dir).normalized();
    _pos += r * d;
}
void Camera::moveLeft(double d)
{
    Eigen::Vector3d r = _up.cross(_dir).normalized();
    _pos -= r * d;
}
void Camera::moveForward(double d)
{
    _pos = _pos - _dir * d;
}
void Camera::moveBack(double d)
{
    _pos = _pos + _dir * d;
}
Camera::Camera(const Eigen::Vector3d &pos, const Eigen::Vector3d &dir, const Eigen::Vector3d &up)
{
    _pos = pos;
    _dir = dir;
    _up = up;
}


// получаем матрицу поворота
Eigen::Matrix3d Camera::getRMatrix(double alpha, const Eigen::Vector3d &v)
{
    double x = v(0);
    double y = v(1);
    double z = v(2);

    Eigen::Matrix3d R;
    R << cos(alpha) + (1 - cos(alpha)) * x * x,
        (1 - cos(alpha)) * x * y - (sin(alpha)) * z,
        (1 - cos(alpha)) * x * z + (sin(alpha)) * y,

        (1 - cos(alpha)) * y * x + (sin(alpha)) * z,
        cos(alpha) + (1 - cos(alpha)) * y * y,
        (1 - cos(alpha)) * y * z - (sin(alpha)) * x,

        (1 - cos(alpha)) * z * x - (sin(alpha)) * y,
        (1 - cos(alpha)) * z * y + (sin(alpha)) * x,
        cos(alpha) + (1 - cos(alpha)) * z * z;

    return R;
}
