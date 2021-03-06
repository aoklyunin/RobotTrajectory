#pragma once
#include "scene_description.h"

#include <Eigen/Dense>

#include <vector>
#include <string>


class TemplateSceneDescription: public SceneDescription
{

public:

    TemplateSceneDescription() = default;
    virtual ~TemplateSceneDescription() = default;

    // list of transform matrices for all links
    virtual std::vector<Eigen::Vector3d> getAxes(std::vector<double> state) override;

private:


    virtual void _loadFromFile(std::string path) override;

    // list of transform matrices for all links
    virtual std::vector<Eigen::Matrix4d> _getTrasformMatrixies(std::vector<double> state) override;

    virtual Eigen::Matrix4d _getLastTrasformMatrix(std::vector<double> state) override;

    virtual std::vector<double> _getMassCenters() override;

    virtual std::vector<Eigen::Matrix3d> _getInertias() override;


};