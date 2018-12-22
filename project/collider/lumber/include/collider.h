#pragma once

#include "solid3_classes.h"

#include <Eigen/Dense>


#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-variable"
#include <MT_Quaternion.h>
#pragma GCC diagnostic pop


class Collider
{
public:
    Collider() = default;
    virtual ~Collider() = default;

    virtual void init(std::vector<std::vector<std::string>> groupedModelPaths) = 0;

    virtual float getDistance(std::vector<Eigen::Matrix4d> matrices) = 0;

    virtual void paint(std::vector<Eigen::Matrix4d> matrices,bool onlyRobot) = 0;
    virtual bool isCollided(std::vector<Eigen::Matrix4d> matrices) = 0;
    virtual bool isCollided(std::vector<Eigen::Matrix4d> matrices, int robotNum) = 0;
    virtual std::vector<float> getPoints(std::vector<Eigen::Matrix4d> matrices) = 0;

    virtual  std::vector<double> getBoxChords(unsigned long objNum,std::vector<Eigen::Matrix4d> matrices) = 0;

    std::string getPrefix()
    {
        return _prefix;
    };



protected:
    bool _isSingleRobot{};

    std::string _prefix;

    std::vector<std::vector<unsigned long>> _robotIndexes;

    std::vector<float> _points;

};


