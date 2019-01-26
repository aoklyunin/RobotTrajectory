#pragma once

#include <memory>
#include <vector>
#include "scene_wrapper.h"
#include "collider.h"

class ForceTorque{
public:
    void paint();
    void prepareTick();
    void tick();
    bool isReady();

    ForceTorque(std::shared_ptr<SceneWrapper>);

    std::shared_ptr<SceneWrapper> getSceneWrapper();

    void setState(std::vector<double> state);

    void writeReport(char * path);

    void calculateDynamic();

    void calculateKinematic();
    void setFlgPlay(bool flgPlay);

private:
    std::shared_ptr<SceneWrapper> _sceneWrapper;

    std::vector<double> _state;
    std::vector<Eigen::Vector3d> _velocities;
    std::vector<Eigen::Vector3d> _accelerations;
    std::vector<double> _torques;
    std::vector<Eigen::Vector3d> _forces;

    // tm, state, torque
    std::vector<std::vector<double>> log;

    std::shared_ptr<Collider> _collider;

    Eigen::Vector3d G;

    bool _flgPlay;
};