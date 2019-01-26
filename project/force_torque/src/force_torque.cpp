#include "force_torque.h"
#include "solid_collider.h"
#include <iostream>
#include <fstream>
#include <include/force_torque.h>


void ForceTorque::paint() {

    _collider->paint(_sceneWrapper->getTrasformMatrices(_state), false);

}

void ForceTorque::prepareTick() {
    _accelerations.clear();
    _velocities.clear();
    for (auto &sd :_sceneWrapper->getSceneDescriptions()) {
        for (int i = 0; i < sd->getLinkCnt() + 1; i++) {
            _accelerations.emplace_back(0.0);
            _velocities.emplace_back(0.0);
            _torques.emplace_back(0.0);
        }
    }
}

void ForceTorque::tick(double dt) {
    //info_msg(_flgPlay);
    calculateDynamic();
    if (_flgPlay)
        calculateKinematic(dt);
}

ForceTorque::ForceTorque(std::shared_ptr<SceneWrapper> sceneWrapper) {
    _sceneWrapper = sceneWrapper;
    _state = sceneWrapper->getRandomState();
    _collider = std::make_shared<SolidCollider>();
    _collider->init(sceneWrapper->getGroupedModelPaths());
    G = Eigen::Vector3d({0.0, 0.0, 9.8});
}

std::shared_ptr<SceneWrapper> ForceTorque::getSceneWrapper() {
    return _sceneWrapper;
}

bool ForceTorque::isReady() {
    return true;
}

void ForceTorque::setState(std::vector<double> state) {
    _state = state;
    for (double &acc:_accelerations) {
        acc = 0;
    }
    for (double &vel:_velocities) {
        vel = 0;
    }
}

void ForceTorque::writeReport(char *path) {
    std::ofstream myfile;
    myfile.open(path);
    myfile << "Writing this to a file.\n";
    myfile.close();
    info_msg("writeReport works");
}

void ForceTorque::calculateDynamic() {
    SceneWrapper::dispState(_state,"state");
    //info_msg("ForceTorque::calculateDynamic()");
    for (auto &sd :_sceneWrapper->getSceneDescriptions()) {
        auto linkPositions = sd->getAllLinkPositions(_state);
        auto massCenterPositions = sd->getAllLinkMassCenterPositions(_state);
        auto inertias = sd->getInertias();

        std::vector<Eigen::Vector3d> torques;

        _forces.clear();
//        _accelerations.clear();
//        _velocities.clear();


        for (int i = 0; i < sd->getLinkCnt() + 1; i++) {
            _forces.emplace_back(Eigen::Vector3d({0.0, 0.0, 0.0}));
            torques.emplace_back(Eigen::Vector3d({0.0, 0.0, 0.0}));
//            _accelerations.emplace_back(Eigen::Vector3d({0.0, 0.0, 0.0}));
//            _velocities.emplace_back(Eigen::Vector3d({0.0, 0.0, 0.0}));
            _torques.emplace_back(0.0);
        }


        for (int i = sd->getLinkCnt() - 1; i >= 0; i--) {
            Eigen::Vector3d rNext;
            if (i == sd->getLinkCnt() - 1) {
                rNext = Eigen::Vector3d({0.0, 0.0, 0.0});
            } else {
                rNext = Eigen::Vector3d({
                                                massCenterPositions.at(i * 4 + 1),
                                                massCenterPositions.at(i * 4 + 2),
                                                massCenterPositions.at(i * 4 + 3),
                                        })
                        - Eigen::Vector3d({
                                                  linkPositions.at((i + 1) * 3),
                                                  linkPositions.at((i + 1) * 3 + 1),
                                                  linkPositions.at((i + 1) * 3 + 2)
                                          });
            }

            Eigen::Vector3d rPrev = Eigen::Vector3d({
                                                            massCenterPositions.at(i * 4 + 1),
                                                            massCenterPositions.at(i * 4 + 2),
                                                            massCenterPositions.at(i * 4 + 3),
                                                    })
                                    - Eigen::Vector3d({
                                                              linkPositions.at(i * 3),
                                                              linkPositions.at(i * 3 + 1),
                                                              linkPositions.at(i * 3 + 2)
                                                      });

            _forces.at(i) = _forces.at(i + 1) + massCenterPositions.at(i * 4) * (/*_accelerations.at(i)+*/ G);
            torques.at(i) = torques.at(i + 1) + _forces.at(i).cross(rPrev) - _forces.at(i + 1).cross(rNext)/* +
                            inertias.at(i) * _accelerations.at(i) +
                            _accelerations.at(i).cross(inertias.at(i) * _velocities.at(i))*/;


            //info_msg("next: ", rNext(0), " ", rNext(1), " ", rNext(2),
            //        " prev: ", rPrev(0), " ", rPrev(1), " ", rPrev(2));

        }

        auto axises = sd->getAxes(_state);


        // info_msg("ax size: ",axises.size()," l cnt: ",sd->getLinkCnt());
        int tPos = 0;
        _torques.clear();
        _inertias.clear();
        for (int i = 0; i < sd->getLinkCnt(); i++) {
            if (axises.at(i).norm() > 0.0001) {
                double proj = torques.at(i).dot(axises.at(i)) / axises.at(i).norm();
                _torques.emplace_back(proj);
                _inertias.emplace_back(sd->getLinks().at(i)->inertia.maxCoeff());
                //info_msg(proj);
                // info_msg("proj: ", i, " ", proj," tn: ",torques.at(i).norm()," an: ",axis.at(i+1).norm());
                //  info_msg("axis: ", axis.at(i+1));
                // info_msg("torques: ", torques.at(i));
            }
        }
        SceneWrapper::dispState(_torques, "torques");
    }


}

void ForceTorque::calculateKinematic(double dt) {
    for (auto &sd :_sceneWrapper->getSceneDescriptions()) {
        for (int i = 0; i < _state.size(); i++) {
            _accelerations.at(i) = _torques.at(i) / _inertias.at(i);
            _state.at(i) += _velocities.at(i) * dt + _accelerations.at(i) * dt * dt / 2;
            _velocities.at(i) += _accelerations.at(i) * dt;
        }
    }
}

void ForceTorque::setFlgPlay(bool flgPlay) {
    if (_flgPlay && !flgPlay) {
        for (double &acc:_accelerations) {
            acc = 0;
        }
        for (double &vel:_velocities) {
            vel = 0;
        }
    }
    _flgPlay = flgPlay;
}
