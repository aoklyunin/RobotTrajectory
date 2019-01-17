#include "force_torque.h"
#include "solid_collider.h"

void ForceTorque::paint() {

    _collider->paint(_sceneWrapper->getTrasformMatrices(_state), false);

}

void ForceTorque::prepareTick() {

}

void ForceTorque::tick() {

}

ForceTorque::ForceTorque(std::shared_ptr<SceneWrapper> sceneWrapper) {
    _sceneWrapper = sceneWrapper;
    _state = sceneWrapper->getRandomState();
    _collider = std::make_shared<SolidCollider>();
    _collider->init(sceneWrapper->getGroupedModelPaths());
}

std::shared_ptr<SceneWrapper> ForceTorque::getSceneWrapper() {
    return _sceneWrapper;
}

bool ForceTorque::isReady() {
    return true;
}

void ForceTorque::setState(std::vector<double> state) {
    _state = state;
}