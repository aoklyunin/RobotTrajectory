#pragma once

#include <mutex>
#include <solid_collider.h>

class SolidSyncCollider: public Collider
{
public:

    const static int MAX_THREAD_CNT = 60;

    explicit SolidSyncCollider(unsigned int threadCnt);

    ~SolidSyncCollider() override = default;

    void init(std::vector<std::vector<std::string>> groupedModelPaths) override;

    void paint(std::vector<Eigen::Matrix4d> matrices,bool onlyRobot) override;
    bool isCollided(std::vector<Eigen::Matrix4d> matrices) override;
    bool isCollided(std::vector<Eigen::Matrix4d> matrices, int robotNum) override;
    float getDistance(std::vector<Eigen::Matrix4d> matrices) override;

    std::vector<float> getPoints(std::vector<Eigen::Matrix4d> matrices) override;
    std::vector<double> getBoxChords(unsigned long objNum,std::vector<Eigen::Matrix4d> matrices) override;

private:

    unsigned int _threadCnt;

    std::vector<std::shared_ptr<SolidCollider>> colliders;

    std::mutex colliderMutexes[MAX_THREAD_CNT];

};
