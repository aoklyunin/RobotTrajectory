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
    std::vector<Eigen::Matrix4d> getTrasformMatrices(std::vector<double> joints) override;

    void addModel(std::string path, std::shared_ptr<Eigen::Matrix4d> localMatrix = nullptr)override;

private:

    void _init(std::string &&description_file_path) override;

};