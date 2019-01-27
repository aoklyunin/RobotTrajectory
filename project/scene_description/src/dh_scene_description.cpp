#include "dh_scene_description.h"


#include <fstream>
#include <traces.h>

void DHSceneDescription::_loadFromFile(std::string path) {

    Json::Reader reader;
    Json::Value obj;

    std::ifstream ifs(path.c_str(), std::ios_base::binary);
    std::string content((std::istreambuf_iterator<char>(ifs)),
                        (std::istreambuf_iterator<char>()));

    assert(!content.empty());
    reader.parse(content, obj); // reader can also read strings
    info_msg("Scene name: ", obj["name"].asString());

    const Json::Value &dhs = obj["dh"]; // array of characters

    for (auto dh : dhs) {
        double theta = dh["theta"].asDouble()/180*3.14;
        double d = dh["d"].asDouble();
        double alpha = dh["alpha"].asDouble()/180*3.14;
        double a = dh["a"].asDouble();
        _addDH(theta, d, alpha, a);
    }

}

void DHSceneDescription::_addDH(double theta, double d, double alpha, double a) {
    std::vector<double> dhParams({theta, d, alpha, a});
    _dhs.emplace_back(dhParams);
}

Eigen::Matrix4d DHSceneDescription::_getDHMatrix(std::vector<double> dh) {
    double theta = dh.at(0);
    double d = dh.at(1);
    double alpha = dh.at(2);
    double a = dh.at(3);
    double data[16] = {
            cos(theta), -sin(theta) * cos(alpha), sin(theta) * sin(alpha), a * cos(theta),
            sin(theta), cos(theta) * cos(alpha), -cos(theta) * sin(alpha), a * sin(theta),
            0, sin(alpha), cos(alpha), d,
            0, 0, 0, 1
    };
    return Eigen::Matrix4d(data);
}

std::vector<Eigen::Matrix4d> DHSceneDescription::_getTrasformMatrixies(std::vector<double> state) {

    // info_msg("urdf scene description: get Transform matrices local");

    Eigen::Matrix4d transformMatrix = *getStartMatrix();

    //  info_msg("get transform matrices local");
//    info_msg(transformMatrix);

    std::vector<Eigen::Matrix4d> matrices;

    unsigned int jointPos = 0;

    //info_msg(_joints.size());
    for (auto & dh: _dhs) {
        transformMatrix = transformMatrix * _getDHMatrix(dh);

        matrices.emplace_back(transformMatrix);

    }


    return matrices;
}

Eigen::Matrix4d DHSceneDescription::_getLastTrasformMatrix(std::vector<double> state) {
    if (_dhs.size() == 0)
        return Eigen::Matrix4d::Identity();

    // info_msg("urdf scene description: get Transform matrices local");

    Eigen::Matrix4d transformMatrix = *getStartMatrix();

    //  info_msg("get transform matrices local");
//    info_msg(transformMatrix);

    std::vector<Eigen::Matrix4d> matrices;

    unsigned int jointPos = 0;

    //info_msg(_joints.size());
    for (auto & dh: _dhs) {
        transformMatrix = transformMatrix * _getDHMatrix(dh);

        matrices.emplace_back(transformMatrix);
    }

    return transformMatrix;
}

std::vector<double> DHSceneDescription::_getMassCenters() {
    return std::vector<double>();
}

std::vector<Eigen::Matrix3d> DHSceneDescription::_getInertias() {
    return std::vector<Eigen::Matrix3d>();
}

std::vector<Eigen::Vector3d> DHSceneDescription::getAxes(std::vector<double> state) {
    return std::vector<Eigen::Vector3d>();
}