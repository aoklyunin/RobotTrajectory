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
    calculateDynamic();
    calculateKinematic();
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

void ForceTorque::calculateDynamic(){
    for (auto &sd :_sceneWrapper->getSceneDescriptions()){
        auto linkPositions = sd->getAllLinkPositions(_state);
        auto massCenterPositions = sd->getAllLinkMassCenterPositions(_state);
        //info_msg(massCenterPositions.size());
        for (int i=0;i<sd->getLinkCnt();i++){
            info_msg(
                    massCenterPositions.at(i*4)," ",
                    massCenterPositions.at(i*4+1)," ",
                    massCenterPositions.at(i*4+2)," ",
                    massCenterPositions.at(i*4+3)
            );
        }
    }
}

void ForceTorque::calculateKinematic(){
    for (auto &sd :_sceneWrapper->getSceneDescriptions()){

    }
}
