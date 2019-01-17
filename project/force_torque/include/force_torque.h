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

private:
    std::shared_ptr<SceneWrapper> _sceneWrapper;

    std::vector<double> _state;
    std::vector<double> _toque;

    // tm, state, torque
    std::vector<std::vector<double>> log;

    std::shared_ptr<Collider> _collider;
};