#include "scene_wrapper.h"
#include <urdf_scene_description.h>

#include <solid_collider.h>
#include <fstream>
#include <json/json.h>

double SceneWrapper::myRound(double v, int digitCnt)
{
    if (digitCnt == 0)
        return v;

    int m = 1;
    for (unsigned int i = 0; i < std::abs(digitCnt); i++) {
        m *= 10;
    }

    if (digitCnt < 0) {
        return round(v * m) / m;
    }

    return round(v / m) * m;

}

std::vector<double> SceneWrapper::diffStates(std::vector<double> a, std::vector<double> b)
{
    std::vector<double> result;
    for (unsigned int i = 0; i < std::min(a.size(), b.size()); i++) {
        result.push_back(a.at(i) - b.at(i));
    }
    return result;
}

std::vector<double> SceneWrapper::sumStates(std::vector<double> a, std::vector<double> b)
{
    std::vector<double> result;
    for (unsigned int i = 0; i < std::min(a.size(), b.size()); i++) {
        result.push_back(a.at(i) + b.at(i));
    }
    return result;
}

std::vector<double> SceneWrapper::multState(std::vector<double> a, double b)
{
    std::vector<double> result;
    for (double v:a) {
        result.push_back(v * b);
    }
    return result;
}

double SceneWrapper::getStateDistance(std::vector<double> a, std::vector<double> b)
{
    //   info_msg(" SceneWrapper::getStateDistance");
    assert(a.size() == b.size());

    std::vector<double> diff = diffStates(a, b);

    double sum = 0;
    for (auto chord:diff) {
        sum += chord * chord;
    }
    return sqrt(sum);
}

double SceneWrapper::getMaxStateChordDist(std::vector<double> a, std::vector<double> b)
{
    assert(a.size() == b.size());
    double maxDist = 0;
    for (unsigned long i = 0; i < a.size(); i++) {
        double tmpDist = std::abs(a.at(i) - b.at(i));
        if (tmpDist > maxDist)
            maxDist = tmpDist;
    }

    return maxDist;
}

void SceneWrapper::dispState(std::vector<double> joints, const char *caption)
{
    std::string msg = caption;
    msg += ":{";
    char buf[256];
    for (double joint: joints) {
        sprintf(buf, "%.3f ,", joint);
        msg += buf;
    }
    msg = msg.substr(0, msg.size() - 2) + "}";
    info_msg(msg);
}

void SceneWrapper::errState(std::vector<double> joints, const char *caption)
{
    std::string msg = caption;
    msg += ":{";
    char buf[256];
    for (double joint: joints) {
        sprintf(buf, "%.3f ,", joint);
        msg += buf;
    }
    msg = msg.substr(0, msg.size() - 2) + "}";
    err_msg(msg);
}

const std::vector<std::shared_ptr<SceneDescription>> &SceneWrapper::getSceneDescriptions() const
{
    return _sceneDescriptions;
}

const std::vector<std::vector<double>> SceneWrapper::getGroupedTranslation() const
{
    std::vector<std::vector<double>> result;
    for (const auto &sceneDescription:_sceneDescriptions) {
        std::vector<double> localTranslations = sceneDescription->getTranslation();
        result.emplace_back(localTranslations);
    }
    return result;

}
const std::vector<std::vector<double>> SceneWrapper::getGroupedRotation() const
{
    std::vector<std::vector<double>> result;

    for (const auto &sceneDescription:_sceneDescriptions) {
        std::vector<double> localRotations = sceneDescription->getRotation();
        result.emplace_back(localRotations);

    }
    return result;

}

const unsigned long SceneWrapper::getRobotCnt() const
{
    unsigned long robotCnt = 0;
    for (const auto &sceneDescription:_sceneDescriptions) {
        if (sceneDescription->getActuatorCnt() != 0)
            robotCnt++;
    }
    return robotCnt;
}

const std::vector<std::vector<double>> SceneWrapper::getGroupedScale() const
{
    std::vector<std::vector<double>> result;
    for (const auto &sceneDescription:_sceneDescriptions) {
        std::vector<double> localScales = sceneDescription->getScale();
        result.emplace_back(localScales);
    }
    return result;
}

const std::vector<std::vector<double>> SceneWrapper::getGroupedPose() const
{
    std::vector<std::vector<double>> result;
    for (const auto &sceneDescription:_sceneDescriptions) {
        std::vector<double> localStartScales = sceneDescription->getPose();
        result.emplace_back(localStartScales);
    }
    return result;
}

const std::vector<std::vector<std::string>> SceneWrapper::getGroupedModelPaths() const
{
    std::vector<std::vector<std::string>> result;
    for (const std::shared_ptr<SceneDescription> &sceneDescription:_sceneDescriptions) {
        result.emplace_back(sceneDescription->getModelPaths());
    };
    return result;
}

void SceneWrapper::setGroupedTranslation(std::vector<std::vector<double>> groupedTranslation)
{
    assert(groupedTranslation.size() == _sceneDescriptions.size());
    for (unsigned int i = 0; i < groupedTranslation.size(); i++) {
        _sceneDescriptions.at(i)->setTranslation(groupedTranslation.at(i));
    }
}
void SceneWrapper::setGroupedRotation(std::vector<std::vector<double>> groupedRotation)
{
    assert(groupedRotation.size() == _sceneDescriptions.size());
    for (unsigned int i = 0; i < groupedRotation.size(); i++) {
        _sceneDescriptions.at(i)->setRotation(groupedRotation.at(i));
    }
}

void SceneWrapper::setGroupedScale(std::vector<std::vector<double>> groupedScale)
{
    assert(groupedScale.size() == _sceneDescriptions.size());
    for (unsigned int i = 0; i < groupedScale.size(); i++) {
        _sceneDescriptions.at(i)->setScale(groupedScale.at(i));
    }
};


void SceneWrapper::setGroupedPose(std::vector<std::vector<double>> groupedPose)
{
    assert(groupedPose.size() == _sceneDescriptions.size());
    for (unsigned int i = 0; i < groupedPose.size(); i++) {
        _sceneDescriptions.at(i)->setPose(groupedPose.at(i));
    }
};

unsigned long SceneWrapper::addModel(std::string path)
{
    info_msg("scene description: add model");

    std::shared_ptr<SceneDescription> sceneDescription = std::make_shared<URDFSceneDescription>();
    sceneDescription->loadFromFile(path);
    _sceneDescriptions.emplace_back(sceneDescription);


    std::vector<unsigned long> range{
        _actuatorCnt,
        sceneDescription->getActuatorCnt() + _actuatorCnt - 1
    };

    _actuatorIndexRanges.emplace_back(range);
    _actuatorCnt += sceneDescription->getActuatorCnt();

    return _sceneDescriptions.size() - 1;
}

unsigned long SceneWrapper::addModel(std::string path, std::vector<double> &pose)
{
    unsigned long robotNum = addModel(std::move(path));
    _sceneDescriptions.at(robotNum)->setPose(pose);
    return robotNum;
}

void SceneWrapper::saveScene(std::string path)
{
    info_msg("scene description save scene, path ", path);
    Json::Value json;

    json["name"] = "saved scene";

    Json::Value modelArr;

    for (unsigned int i = 0; i < _sceneDescriptions.size(); i++) {
        const auto &sceneDescription = _sceneDescriptions.at(i);
        for (unsigned int j = 0; j < 3; j++) {
            modelArr[i]["pos"][j] = sceneDescription->getTranslation().at(j);
            modelArr[i]["rpy"][j] = sceneDescription->getRotation().at(j);
            modelArr[i]["scale"][j] = sceneDescription->getScale().at(j);
        }
        modelArr[i]["model"] = sceneDescription->getPath();
    }
    json["dynamicObjects"] = modelArr;
    info_msg("complete");

    std::ofstream myfile;
    myfile.open(path);
    myfile << json.toStyledString();
    myfile.close();

    info_msg("complete");
}

void SceneWrapper::deleteModel(unsigned long robotNum)
{
    _sceneDescriptions.erase(_sceneDescriptions.begin() + robotNum);
}

void SceneWrapper::buildFromFile(std::string path)
{
    _path = path;
    _sceneDescriptions.clear();

    Json::Reader reader;
    Json::Value obj;

    std::ifstream ifs(path.c_str(), std::ios_base::binary);
    std::string content((std::istreambuf_iterator<char>(ifs)),
                        (std::istreambuf_iterator<char>()));

    assert(!content.empty());
    reader.parse(content, obj); // reader can also read strings
    info_msg("Scene name: ", obj["name"].asString());

    const Json::Value &dos = obj["dynamicObjects"]; // array of characters

    for (auto d : dos) {
        std::string URDFPath = d["model"].asString();
        std::shared_ptr<Eigen::Matrix4d> localMatrix = std::make_shared<Eigen::Matrix4d>();

        std::vector<double> pose{
            d["pos"][0].asDouble(),
            d["pos"][1].asDouble(),
            d["pos"][2].asDouble(),
            d["rpy"][0].asDouble(),
            d["rpy"][0].asDouble(),
            d["rpy"][0].asDouble(),
            d["scale"][0].asDouble(),
            d["scale"][0].asDouble(),
            d["scale"][0].asDouble()
        };

        addModel(URDFPath, pose);
    }

}

std::vector<std::vector<unsigned long>> SceneWrapper::getActuratorIndexRanges()
{
    return _actuatorIndexRanges;
}

std::shared_ptr<SceneDescription> SceneWrapper::getSingleSceneDescription(unsigned long robotNum)
{
    assert(_sceneDescriptions.size() > robotNum);
    return _sceneDescriptions.at(robotNum);
}

unsigned long SceneWrapper::getActuatorCnt()
{
    unsigned long actuatorCnt = 0;
    for (const auto &sceneDescription:_sceneDescriptions) {
        actuatorCnt += sceneDescription->getActuatorCnt();
    }
    return actuatorCnt;
}

const bool SceneWrapper::isSingleRobot() const
{
    int robotCnt = 0;
    for (const auto &sceneDescription:_sceneDescriptions) {
        if (sceneDescription->getActuatorCnt() > 0)
            robotCnt++;
    }
    //info_msg("robotCnt ", robotCnt);
    return robotCnt == 1;
}

std::vector<std::shared_ptr<Actuator>> SceneWrapper::getActuators()
{
    std::vector<std::shared_ptr<Actuator>> result;
    for (const auto &sceneDescription:_sceneDescriptions) {
        std::vector<std::shared_ptr<Actuator>> localActuators = sceneDescription->getActuators();
        result.insert(result.end(), localActuators.begin(), localActuators.end());
    }
    return result;
}

const std::vector<std::shared_ptr<Link>> SceneWrapper::getLinks() const
{

    std::vector<std::shared_ptr<Link>> links;
    for (auto sceneDescription:_sceneDescriptions) {
        std::vector<std::shared_ptr<Link>> localLinks = sceneDescription->getLinks();
        links.insert(links.end(), localLinks.begin(), localLinks.end());
    }
    return links;

}

bool SceneWrapper::isStateEnabled(std::vector<double> state, unsigned long robotNum)
{

    return _sceneDescriptions.at(robotNum)->isStateEnabled(std::move(state));
}

bool SceneWrapper::isStateEnabled(std::vector<double> state)
{
    assert(_actuatorIndexRanges.size() == _sceneDescriptions.size());
    assert(_actuatorIndexRanges.size() == _sceneDescriptions.size());

    for (unsigned long i = 0; i < _actuatorIndexRanges.size(); i++) {
        std::vector<double> localState(
            state.begin() + _actuatorIndexRanges.at(i).front(),
            state.begin() + _actuatorIndexRanges.at(i).back() + 1
        );
        if (!isStateEnabled(localState, i))
            return false;
    }

    return true;
}

std::vector<double> SceneWrapper::getSingleRobotState(
    const std::vector<double> &state,
    unsigned long robotNum)
{
    std::vector<unsigned long> range = _actuatorIndexRanges.at(robotNum);

    std::vector<double> singleRobotState;

    for (unsigned long i = range.front(); i <= range.back(); i++) {
        singleRobotState.emplace_back(state.at(i));
    }

    return singleRobotState;
}

const std::string SceneWrapper::getScenePath() const
{
    return _path;
}

std::vector<double> SceneWrapper::getRandomState()
{
    //info_msg("scene wrapper: get random state");
    std::vector<double> state;
    do {
        std::vector<double> joints;
        state.clear();
        for (const auto &actuator : getActuators()) {
            double max = actuator->maxAngle;
            double min = actuator->minAngle;
            double f = (double) rand() / RAND_MAX;
            state.push_back(myRound(min + f * (max - min), -3));
        }

    }
    while (!isStateEnabled(state));
    return state;
}

void SceneWrapper::setTranslation(const std::vector<double> &translation)
{

    assert(translation.size() / 3 == _sceneDescriptions.size());
    for (unsigned int i = 0; i < _sceneDescriptions.size(); i++) {
        _sceneDescriptions.at(i)->setTranslation(std::vector<double>(
            translation.begin() + i * 3, translation.begin() + (i + 1) * 3
        ));
    }

}
void SceneWrapper::setRotation(const std::vector<double> &rotation)
{
    assert(rotation.size() / 3 == _sceneDescriptions.size());
    for (unsigned int i = 0; i < _sceneDescriptions.size(); i++) {
        _sceneDescriptions.at(i)->setRotation(std::vector<double>(
            rotation.begin() + i * 3, rotation.begin() + (i + 1) * 3
        ));
    }

}

void SceneWrapper::setScale(const std::vector<double> &scale)
{
    assert(scale.size() / 3 == _sceneDescriptions.size());
    for (unsigned int i = 0; i < _sceneDescriptions.size(); i++) {
        _sceneDescriptions.at(i)->setScale(std::vector<double>(
            scale.begin() + i * 3, scale.begin() + (i + 1) * 3
        ));
    }
}

std::vector<Eigen::Matrix4d> SceneWrapper::getTrasformMatrices(std::vector<double> state)
{
    //   info_msg("SceneWrapper::getTrasformMatrices");
    assert(!state.empty());
    std::vector<Eigen::Matrix4d> matrices;
    unsigned long Apos = 0;
    int i = 0;
    for (const auto &sceneDescription:_sceneDescriptions) {
        //info_msg("loop ", i++);
        assert(sceneDescription);
        const unsigned long actuatorCnt = sceneDescription->getActuatorCnt();

        // info_msg("aCnt ", actuatorCnt, " pos ", Apos);
        // info_msg(state.size());

        std::vector<double> localState;
        if (actuatorCnt > 0) {
            localState = std::vector<double>(state.begin() + Apos, state.begin() + Apos + actuatorCnt);
        }
        // info_msg("get local state ", localState.size());
        std::vector<Eigen::Matrix4d> localMatrices = sceneDescription->getTrasformMatrixies(localState);
        // info_msg("localMatrix ", localMatrices.size());
        matrices.insert(matrices.end(), localMatrices.begin(), localMatrices.end());

        Apos += actuatorCnt;
        // info_msg("loop end");
        // }
    }
    return matrices;
}

std::vector<double> SceneWrapper::getEndEffectorPoses(std::vector<double> state)
{
    std::vector<double> poses;
    for (unsigned long i = 0; i < _sceneDescriptions.size(); i++) {
        if (_sceneDescriptions.at(i)->getActuatorCnt() > 0) {
            auto localState = getSingleRobotState(state, i);
            auto pos = _sceneDescriptions.at(i)->getEndEffectorPos(state);
            poses.insert(poses.end(), pos.begin(), pos.end());
        }
    }
    return poses;
}

std::vector<double> SceneWrapper::getAllPoses(std::vector<double> state)
{
    std::vector<double> poses;
    for (unsigned long i = 0; i < _sceneDescriptions.size(); i++) {
        if (_sceneDescriptions.at(i)->getActuatorCnt() > 0) {
 //           info_msg("SceneWrapper::getAllLinkPositions: ",i);
            auto localState = getSingleRobotState(state, i);
   //         dispState(localState, "local state");
            auto pos = _sceneDescriptions.at(i)->getAllLinkPositions(localState);
            poses.insert(poses.end(), pos.begin(), pos.end());
        }
    }
    return poses;
}
