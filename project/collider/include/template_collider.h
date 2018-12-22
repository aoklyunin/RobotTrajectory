#pragma once

#include <collider.h>
#include <solid3_classes.h>

#include <MT_Quaternion.h>
#include <vector>
#include <memory>


class TemplateCollider: public Collider
{
public:

    TemplateCollider() = default;
    ~TemplateCollider() = default;

    void init(std::vector<std::vector<std::string>> groupedModelPaths);

    void paint(std::vector<Eigen::Matrix4d> matrices) override;
    bool isCollided(std::vector<Eigen::Matrix4d> matrices) override;
    bool isCollided(std::vector<Eigen::Matrix4d> vector, int robotNum) override;
};