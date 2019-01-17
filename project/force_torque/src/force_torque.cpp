#include "force_torque.h"
#include "solid_collider.h"
#include <iostream>
#include <fstream>

void ForceTorque::paint() {

    _collider->paint(_sceneWrapper->getTrasformMatrices(_state), false);

}

void ForceTorque::prepareTick() {

}

void ForceTorque::tick() {
    for (auto &sd :_sceneWrapper->getSceneDescriptions()){

    }
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

void ForceTorque::writeReport(char *path) {
    std::ofstream myfile;
    myfile.open (path);
    myfile << "Writing this to a file.\n";
    myfile.close();
    info_msg("writeReport works");
}
