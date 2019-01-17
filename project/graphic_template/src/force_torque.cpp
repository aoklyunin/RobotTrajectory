#include "force_torque.h"
#include "solid_collider.h"
#include "../../force_torque/include/force_torque.h"


void GraphicTemplate::paint() {

    _collider->paint(_sceneWrapper->getTrasformMatrices(_state), false);

}

void GraphicTemplate::prepareTick() {

}

void GraphicTemplate::tick() {

}

GraphicTemplate::GraphicTemplate(std::shared_ptr<SceneWrapper> sceneWrapper) {
    _sceneWrapper = sceneWrapper;
    _state = sceneWrapper->getRandomState();
    _collider = std::make_shared<SolidCollider>();
    _collider->init(sceneWrapper->getGroupedModelPaths());
}

std::shared_ptr<SceneWrapper> GraphicTemplate::getSceneWrapper() {
    return _sceneWrapper;
}

bool GraphicTemplate::isReady() {
    return true;
}

void GraphicTemplate::setState(std::vector<double> state) {
    _state = state;
}

