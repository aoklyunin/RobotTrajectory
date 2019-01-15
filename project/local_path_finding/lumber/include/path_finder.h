#pragma once

#include <memory>
#include <unordered_map>
#include <set>
#include <utility>
#include <unordered_set>
#include <scene_wrapper.h>
#include <collider.h>
#include <json/json.h>


// main class for finding path_finding
class PathFinder
{
public:


    std::vector<std::vector<double>>
    findPath(const std::vector<double> &startState, const std::vector<double> &endState, int &errorCode);

    const static int CHECK_CNT = 10;

    static const int NO_ERROR = -1;
    static const int ERROR_COLLISION_IN_CHECKING = 0;
    static const int ERROR_REACHED_MAX_NODE_CNT = 1;
    static const int ERROR_REACHED_MAX_DISCRET_CNT = 2;
    static const int ERROR_CAN_NOT_FIND_PATH = 3;
    static const int ERROR_CAN_NOT_FIND_FREE_START_POINT = 4;
    static const int ERROR_CAN_NOT_FIND_FREE_END_POINT = 5;
    static const int ERROR_LOCKING_OFFSETS = 6;
    static const int ERROR_UNIT_OFFSETS = 7;
    static const int ERROR_START_CHORDS_ARE_DISABLED = 8;
    static const int ERROR_END_CHORDS_ARE_DISABLED = 9;

    static const std::string errorLabels[10];

    explicit PathFinder(std::shared_ptr<SceneWrapper> sceneWrapper,
                        bool showTrace, bool sync=false);

    virtual ~PathFinder() = default;

    virtual void prepareFindTick(const std::vector<double> &startState, const std::vector<double> &endState) = 0;
    virtual bool findTick(std::vector<double> &state, std::string &logMsg)  = 0;
    virtual void buildPath()  = 0;

    static void showPath(std::vector<std::vector<double>> path);
    Json::Value getJSONRepresentation(std::vector<std::vector<double>> path);
    void savePathToFile(std::vector<std::vector<double>> path, std::string filename);

    const std::shared_ptr<SceneWrapper> &getSceneWrapper() const;

    const std::shared_ptr<Collider> &getCollider() const;
    std::vector<double> getBoxChords(unsigned long objNum, std::vector<double> state);
    void updateCollider();
    bool checkCollision(std::vector<double> state);

    bool checkPath(std::vector<double> prevPoint, std::vector<double> nextPoint);
    int checkPath(std::vector<std::vector<double>> points);

    std::vector<double> getRandomState();

    std::vector<double> getStateFromPath(double tm);
    double getPathTimeDuraion();

    void setState(const std::vector<double> &_state);
    const std::vector<double> &getState() const;

    bool isReady() const;
    void setReady(bool ready);

    void paint(std::vector<double> state,bool onlyRobot);

    std::vector<double> & getStartState();
    std::vector<double> & getEndState();

    int getErrorCode() const;
    const std::vector<std::vector<double>> &getPath() const;

    double getCalculationTimeInSeconds() const;

protected:

    int _errorCode = NO_ERROR;

    std::vector<std::shared_ptr<Actuator>> _actuators;

    std::shared_ptr<SceneWrapper> _sceneWrapper;


    static std::vector<std::vector<double>> _splitPath(std::vector<std::vector<double>> points, unsigned long partCnt);

    bool _showTrace;

    bool _checkPath(std::vector<double> prevPoint, std::vector<double> nextPoint);

    double getColliderDistance(std::vector<double> state);

    std::shared_ptr<Collider> _collider;

    std::string getColliderPrefix();

    // flag is wrapper _ready for work
    // it loads scene description and stl model for collider
    // it can take some time
    bool _ready = false;

    bool isStateEnabled(std::vector<double> state);

    std::vector<double> _state;

    std::vector<double> _endState;
    std::vector<double> _startState;
    std::vector<std::vector<double>> _path;

    std::chrono::time_point<std::chrono::system_clock> _startTime;

    double _calculationTimeInSeconds;

};
