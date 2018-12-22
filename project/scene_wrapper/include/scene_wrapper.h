#pragma once

#include "scene_description.h"

#include <Eigen/Dense>
#include <vector>
#include <cstdlib>
#include <cmath>
#include <memory>


class SceneWrapper
{
public:

    static double myRound(double v, int digitCnt);
    static std::vector<double> diffStates(std::vector<double> a, std::vector<double> b);
    static std::vector<double> sumStates(std::vector<double> a, std::vector<double> b);
    static std::vector<double> multState(std::vector<double> a, double b);
    static double getStateDistance(std::vector<double> a, std::vector<double> b);
    static double getMaxStateChordDist(std::vector<double> a, std::vector<double> b);
    static void dispState(std::vector<double> joints, const char *caption);
    static void errState(std::vector<double> joints, const char *caption);

    SceneWrapper() = default;
    ~SceneWrapper() = default;

    const std::vector<std::shared_ptr<SceneDescription>> &getSceneDescriptions() const;
    const std::vector<std::vector<double>> getGroupedTranslation() const;
    const std::vector<std::vector<double>> getGroupedRotation() const;
    const std::vector<std::vector<double>> getGroupedScale() const;
    const std::vector<std::vector<double>> getGroupedPose() const;
    const std::vector<std::vector<std::string>> getGroupedModelPaths() const;

    void setGroupedTranslation(std::vector<std::vector<double>> groupedTranslation);
    void setGroupedRotation(std::vector<std::vector<double>> groupedRotation);
    void setGroupedScale(std::vector<std::vector<double>> groupedScale);
    void setGroupedPose(std::vector<std::vector<double>> groupedPose);

    void setTranslation(const std::vector<double> &translation);
    void setRotation(const std::vector<double> &srotation);
    void setScale(const std::vector<double> &scale);
    void setPose(const std::vector<double> &pose);

    unsigned long addModel(std::string path);
    unsigned long addModel(std::string path, std::vector<double> &pose);

    void saveScene(std::string path);
    void deleteModel(unsigned long robotNum);
    void buildFromFile(std::string path);

    std::shared_ptr<SceneDescription> getSingleSceneDescription(unsigned long robotNum);
    std::vector<std::vector<unsigned long>> getActuratorIndexRanges();

    const unsigned long getRobotCnt() const;

    unsigned long getActuatorCnt();
    const bool isSingleRobot() const;
    std::vector<std::shared_ptr<Actuator>> getActuators();
    const std::vector<std::shared_ptr<Link>> getLinks() const;

    bool isStateEnabled(std::vector<double> state);
    bool isStateEnabled(std::vector<double> state, unsigned long robotNum);

    std::vector<double> getRandomState();

    const std::string getScenePath() const;

    std::vector<double> getSingleRobotState(
        const std::vector<double> &state,
        unsigned long robotNum
    );

    std::vector<Eigen::Matrix4d> getTrasformMatrices(std::vector<double> state);
    std::vector<double> getEndEffectorPoses(std::vector<double> state);
    std::vector<double> getAllPoses(std::vector<double> state);

private :

    std::vector<std::vector<unsigned long>> _actuatorIndexRanges;
    unsigned int _actuatorCnt;

    std::vector<std::shared_ptr<SceneDescription>> _sceneDescriptions;

    std::string _path;
};


