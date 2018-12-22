#pragma once

#include <scene_description.h>

#include "urdf/model.h"

#include <Eigen/Dense>

#include <utility>
#include <vector>
#include <string>

struct URDFJoint
{
    Eigen::Matrix4d selfTransform;
    Eigen::Matrix4d linkTransform;
    urdf::Vector3 axis;
    bool isFixed;
    double jointAngle;
    URDFJoint(Eigen::Matrix4d selfTransform,
                         Eigen::Matrix4d linkTransform,
                         const urdf::Vector3 &axis,
                         bool isFixed,
                         double jointAngle)
        : selfTransform(std::move(selfTransform)), linkTransform(std::move(linkTransform)), axis(axis), isFixed(isFixed), jointAngle(
        jointAngle)
    {}

    Eigen::Matrix4d getTransformMatrix();
};

class URDFSceneDescription: public SceneDescription
{

public:


    URDFSceneDescription() = default;
    ~URDFSceneDescription() override = default;


private:

    // list of transform matrices for all links
    std::vector<Eigen::Matrix4d> _getTrasformMatrixies(std::vector<double> state) override;
  Eigen::Matrix4d _getLastTrasformMatrix(std::vector<double> state) override;

    void _loadFromFile(std::string path) override;

    void _addURDFJoints(std::vector<std::shared_ptr<urdf::Joint>> jointList,
                        std::map<std::string, urdf::LinkSharedPtr> linkMap);

    Eigen::Matrix4d _getMatrixFromPose(urdf::Pose pose);

    std::vector<URDFJoint> _joints;


};