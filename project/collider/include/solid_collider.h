#pragma once


#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-variable"
#include <MT_Quaternion.h>
#pragma GCC diagnostic pop

#include "collider.h"
#include <vector>

#include <memory>


class SolidCollider: public Collider
{
public:

    SolidCollider() = default;
    ~SolidCollider() override;

    void init(std::vector<std::vector<std::string>> groupedModelPaths) override;

    void paint(std::vector<Eigen::Matrix4d> matrices,bool onlyRobot) override;
    float getDistance(std::vector<Eigen::Matrix4d> matrices) override;

    bool isCollided(std::vector<Eigen::Matrix4d> matrices) override;
    bool isCollided(std::vector<Eigen::Matrix4d> matrices, int robotNum) override;

    std::vector<float>  getPoints(std::vector<Eigen::Matrix4d> matrices) override;

    std::vector<double> getBoxChords(unsigned long objNum,std::vector<Eigen::Matrix4d> matrices) override;


    void setTransformMatrices(std::vector<Eigen::Matrix4d> matrices);

    const std::vector<std::shared_ptr<Solid3Object>> &getLinks() const;

private:

    bool isCollided();

    static double *_eigenToDouble(Eigen::Matrix4d m);
    static void _displayBox(const MT_Point3 &min, const MT_Point3 &max);

    DT_SceneHandle _scene;
    std::vector<std::shared_ptr<Solid3Object>> _links;

    std::vector<std::vector<std::shared_ptr<Solid3Object>>> _groupedLinks;


    std::vector<std::shared_ptr<Collider>> singleRobotColliders;
    std::vector<Eigen::Matrix4d> getSingleRobotmatrixList(std::vector<Eigen::Matrix4d> matrices, int robotNum);

};
