#include "path_finder.h"
#include <iostream>
#include "traces.h"
#include <json/json.h>
#include <fstream>
#include <utility>
#include <solid_collider.h>


const std::string PathFinder::errorLabels[10] = {
    "collision in checking path",
    "reached max node cnt",
    "reached max discret cnt",
    "can not find path",
    "can not find free point near start",
    "can not find free point near end",
    "locking offsets",
    "unit offsets",
    "star chords are disabled",
    "end chords are disabled"
};

void PathFinder::updateCollider()
{
    _collider->init(_sceneWrapper->getGroupedModelPaths());
}

// show path_finding
void PathFinder::savePathToFile(std::vector<std::vector<double>> path, std::string filename)
{

    std::string jsonRepresentation =
        PathFinder::getJSONRepresentation(std::move(path)).toStyledString();

    std::ofstream ofs;
    ofs.open(filename.c_str(), std::ios::out | std::ios::binary);
    ofs.write(jsonRepresentation.c_str(), jsonRepresentation.length());
    ofs.close();

}

// returns json time based path_finding description
Json::Value PathFinder::getJSONRepresentation(std::vector<std::vector<double>> path)
{
    std::vector<double> timestamps;
    std::vector<std::shared_ptr<Actuator>> actuators = _sceneWrapper->getActuators();

    timestamps.push_back(0.0);
    double curTime = 0;

    for (unsigned int i = 1; i < path.size(); i++) {
        std::vector<double> delta = SceneWrapper::diffStates(path.at(i), path.at(i - 1));
        double maxTime = 0;
        for (unsigned int j = 0; j < delta.size(); j++) {
            double t = delta.at(j) / actuators.at(j)->maxVelocity;
            if (t > maxTime)
                maxTime = t;
        }
        curTime += maxTime;
        timestamps.push_back(curTime);
    }

    Json::Value json;

    Json::Value pointArr;

    for (unsigned int i = 0; i < path.size(); i++) {
        Json::Value point;
        point["ts"] = timestamps.at(i);
        Json::Value state;
        for (unsigned int j = 0; j < path.at(i).size(); j++) {
            state[j] = path.at(i).at(j);
        }
        point["state"] = state;
        pointArr[i] = point;
    }
    json["states"] = pointArr;

    return json;

}

bool PathFinder::_checkPath(std::vector<double> prevPoint, std::vector<double> nextPoint)
{
    // count of checks
    double checkStep = 1.0 / CHECK_CNT;

    std::vector<double> delta = SceneWrapper::multState(
        SceneWrapper::diffStates(std::move(nextPoint), prevPoint),
        checkStep
    );

    // forward direction: find first bad point(do not check last point)
    std::vector<double> currentPoint = prevPoint;
    for (int i = 0; i < CHECK_CNT; i++) {
        if (!isStateEnabled(currentPoint)) {
            return false;
        }
        currentPoint = SceneWrapper::sumStates(currentPoint, delta);
    }
    return true;
}

// show path_finding
void PathFinder::showPath(std::vector<std::vector<double>> path)
{
    for (const auto &state:path) {
        SceneWrapper::dispState(state, "path_finding node");
    }
}

// constructor with arguments:
// robot - a SceneWrapper class object needs for distance, limit and collider calculation
// discretCnt - count of grid nodes in one dimention
PathFinder::PathFinder(std::shared_ptr<SceneWrapper> sceneWrapper,
                       bool showTrace)
{
    _ready = false;
    assert(sceneWrapper);
    _collider = std::make_shared<SolidCollider>();
    _collider->init(sceneWrapper->getGroupedModelPaths());
    _sceneWrapper = sceneWrapper;
    _showTrace = showTrace;
    _actuators = sceneWrapper->getActuators();

    for (unsigned long i=0;i<_sceneWrapper->getActuatorCnt();i++){
        _startState.emplace_back(0.0);
        _endState.emplace_back(0.0);
    }
}

std::vector<std::vector<double>> PathFinder::_splitPath(std::vector<std::vector<double>> points, unsigned long partCnt)
{

    double step = 1.0 / partCnt;

    info_msg(step);

    std::vector<std::vector<double>> path;

    for (unsigned int i = 1; i < points.size(); i++) {
        std::vector<double> prevPoint = points.at(i - 1);
        std::vector<double> nextPoint = points.at(i);
        std::vector<double> delta = SceneWrapper::multState(SceneWrapper::diffStates(nextPoint, prevPoint), step);

        for (unsigned int j = 0; j < partCnt; j++) {
            std::vector<double> s = SceneWrapper::multState(delta, j);
            path.push_back(SceneWrapper::sumStates(prevPoint, s));
        }

    }

    path.push_back(*std::prev(points.end()));


    return path;
}

const std::shared_ptr<SceneWrapper> &PathFinder::getSceneWrapper() const
{
    return _sceneWrapper;
}
const std::shared_ptr<Collider> &PathFinder::getCollider() const
{
    return _collider;
}

double PathFinder::getColliderDistance(std::vector<double> state)
{
    return _collider->getDistance(_sceneWrapper->getTrasformMatrices(state));
}

bool PathFinder::checkCollision(std::vector<double> state)
{
    return _collider->isCollided(_sceneWrapper->getTrasformMatrices(state));
}

std::string PathFinder::getColliderPrefix()
{
    return "";
}

bool PathFinder::isStateEnabled(std::vector<double> state)
{
    //info_msg("PathFinder::isStateEnabled");
    if (!_sceneWrapper->isStateEnabled(state)) {
        info_msg("state is disabled by scene wrapper");
        return false;
    }

//    if (_collider->isCollided(_sceneWrapper->getTrasformMatrices(state))){
//        info_msg("collsion");
//    }
    return !_collider->isCollided(_sceneWrapper->getTrasformMatrices(state));
}

std::vector<double> PathFinder::getBoxChords(unsigned long objNum, std::vector<double> state)
{
    return _collider->getBoxChords(objNum, _sceneWrapper->getTrasformMatrices(state));
}
const std::vector<double> &PathFinder::getState() const
{
    return _state;
}
void PathFinder::setState(const std::vector<double> &_state)
{
    PathFinder::_state = _state;
}
bool PathFinder::isReady() const
{
    return _ready;
}
void PathFinder::setReady(bool ready)
{
    _ready = ready;
}
void PathFinder::paint(std::vector<double> state, bool onlyRobot)
{
    assert(!state.empty());

   // info_msg("path finder paint");
    //info_msg(state.size());
    _collider->paint(_sceneWrapper->getTrasformMatrices(state), onlyRobot);
}

// returns all ranges with prohibit states
bool PathFinder::checkPath(std::vector<double> prevPoint, std::vector<double> nextPoint)
{
    const static int CHECK_CNT = 100; // count of checks

    double checkStep = 1.0 / CHECK_CNT;

    std::vector<double> delta = SceneWrapper::multState(
        SceneWrapper::diffStates(std::move(nextPoint), prevPoint),
        checkStep
    );

    // forward direction: find first bad point(do not check last point)
    std::vector<double> currentPoint = prevPoint;
    for (int i = 0; i < CHECK_CNT; i++) {
        if (!isStateEnabled(currentPoint)) {
            return false;
        }
        currentPoint = SceneWrapper::sumStates(currentPoint, delta);
    }
    return true;
}

// returns all ranges with prohibit states
int PathFinder::checkPath(std::vector<std::vector<double>> points)
{
    for (unsigned int i = 1; i < points.size(); i++) {
        std::vector<double> prevPoint = points.at(i - 1);
        std::vector<double> nextPoint = points.at(i);
        if (!checkPath(prevPoint, nextPoint)) {
            //  info_msg("Collided");
            SceneWrapper::dispState(prevPoint, "prev");
            SceneWrapper::dispState(nextPoint, "next");
            return i - 1;
        }
    }
    return NO_ERROR;
}

std::vector<double> PathFinder::getRandomState()
{
    //info_msg("scene wrapper: get random state");
    std::vector<double> state;
    do {
        state = _sceneWrapper->getRandomState();
    }
    while (!isStateEnabled(state));
    return state;
}

double PathFinder::getPathTimeDuraion()
{
    assert(!_path.empty());

    return _path.size() - 1;
}

std::vector<double> PathFinder::getStateFromPath(double tm)
{
    assert(tm >= 0);
    //info_msg("LocalPathFinder::getStateFromPath");

    auto pos = (unsigned long) tm;

    auto prevState = _path.at(pos);
    auto nextState = _path.at(pos + 1);

    auto delta = SceneWrapper::diffStates(nextState, prevState);
    return SceneWrapper::sumStates(
        prevState,
        SceneWrapper::multState(delta, tm - pos)
    );

}
std::vector<double> &PathFinder::getStartState()
{
    return _startState;
}


std::vector<double> &PathFinder::getEndState()
{
    return _endState;
}

// find path_finding between two state points
std::vector<std::vector<double>>
PathFinder::findPath(const std::vector<double> &startState, const std::vector<double> &endState, int &errorCode)
{
    _startState = startState;
    _endState = endState;

    prepareTick(startState, endState);

    std::vector<double> actualState;
    std::string logMsg;

    while (!tick(actualState, logMsg)) {};

    buildPath();

    errorCode = _errorCode;

    return _path;
}
int PathFinder::getErrorCode() const
{
    return _errorCode;
}
const std::vector<std::vector<double>> &PathFinder::getPath() const
{
    return _path;
}
