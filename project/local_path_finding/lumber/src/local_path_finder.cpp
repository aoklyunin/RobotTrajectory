#include "local_path_finder.h"

#include <fstream>
#include <local_path_finder.h>


bool LocalPathFinder::_checkUnitOffsets(std::vector<int> &offsetList)
{
    for (int offset:offsetList) {
        if (std::abs(offset) > 1)
            return false;
    }
    return true;
}

int LocalPathFinder::_checkLockingOffsets(std::vector<int> &offsetList, unsigned int maxRepeatCnt)
{
    if (_checkUnitOffsets(offsetList)) {
        _nonModifiedCnt = 0;
        return NO_ERROR;
    }

    if (_prevCheckingOffsetList.empty()) {
        _nonModifiedCnt = 0;
        _prevCheckingOffsetList = offsetList;
        return NO_ERROR;
    }

    bool flgModify = false;
    for (unsigned int i = 0; i < offsetList.size(); i++) {
        if (std::abs(offsetList.at(i)) > 1) {
            if (offsetList.at(i) != _prevCheckingOffsetList.at(i))
                flgModify = true;
        }
    }
    _prevCheckingOffsetList = offsetList;

    if (!flgModify)
        _nonModifiedCnt++;
    else {
        _nonModifiedCnt = 0;
        return NO_ERROR;
    }

    if (_nonModifiedCnt > maxRepeatCnt) {
        _nonModifiedCnt = 0;

        int findPos =
            static_cast<int>(std::distance(offsetList.begin(),
                                           std::max_element(offsetList.begin(),
                                                            offsetList.end()))
            );

        unsigned long pos = 0;
        if (!_sceneWrapper->isSingleRobot() && findPos >= 0) {
            auto sceneDescriptions = _sceneWrapper->getSceneDescriptions();
            for (int unsigned j = 0; j < sceneDescriptions.size(); j++) {
                const auto &sceneDescription = sceneDescriptions.at(j);
                unsigned long b = pos;
                unsigned long e = pos + sceneDescription->getActuatorCnt();
                if (pos >= b && pos < e) {
                    return j;
                }
                pos += sceneDescription->getActuatorCnt();
            }
        }
        else {
            return findPos;
        }
    }
    return NO_ERROR;
}

std::vector<unsigned long> LocalPathFinder::_getOrderedOffsetIndexesMultiRobot(
    std::vector<int> &offsetList
)
{
    struct RobotStruct
    {
        RobotStruct(unsigned int index, std::vector<int> &chords)
        {
            this->index = index;
            this->chords = chords;
        }
        unsigned int index;
        std::vector<int> chords;
        bool operator<(const RobotStruct &obj) const
        {
            //info_msg(this->chords.size(), " ", obj.chords.size());

            assert(this->chords.size() == obj.chords.size());
            int aMax = std::abs(*this->chords.begin());
            int bMax = std::abs(*obj.chords.begin());
            for (int chord:this->chords) {
                if (std::abs(chord) > aMax)
                    aMax = std::abs(chord);
            }
            for (int chord:obj.chords) {
                if (std::abs(chord) > bMax)
                    bMax = std::abs(chord);
            }
            //     info_msg("asum:",aSum," bSum",bSum);
            return aMax > bMax;
        }
    };

    std::vector<RobotStruct> robotsStructs;

    unsigned long pos = 0;
    unsigned long j = 0;
    for (const auto &sceneDescription: _sceneWrapper->getSceneDescriptions()) {
        std::vector<int> tmp =
            std::vector<int>(offsetList.begin() + pos, offsetList.begin() + pos + sceneDescription->getActuatorCnt());
        robotsStructs.emplace_back(j, tmp);
        pos += sceneDescription->getActuatorCnt();
        j++;
    }

    unsigned long robotCnt = robotsStructs.size();

    std::sort(robotsStructs.begin(), robotsStructs.end());

    std::vector<unsigned long> result;
    for (unsigned long k = 0; k < robotCnt; k++) {
        //info_msg(robotsStructs.at(j).index);
        for (int i = 0; i < 6; i++) {
            unsigned long statePos = robotsStructs.at(k).index * 6 + i;
            result.push_back(statePos * 2);
            result.push_back(statePos * 2 + 1);
        }
    }

    return result;
}

std::vector<unsigned long> LocalPathFinder::_getSizeOrderedOffsetIndexesSingleRobot(std::vector<int> &offsetList)
{
    struct OffsetStruct
    {
        OffsetStruct(unsigned int index, int offset)
        {
            this->index = index;
            this->offset = offset;
        }
        unsigned int index;
        int offset;
        bool operator<(const OffsetStruct &obj) const
        {
            return std::abs(this->offset) < std::abs(obj.offset);
        }
    };

    std::vector<OffsetStruct> offsetStructs;

    for (unsigned int i = 0; i < offsetList.size(); i++) {
        offsetStructs.emplace_back(i, offsetList.at(i));
    }

    std::sort(offsetStructs.begin(), offsetStructs.end());

    std::vector<unsigned long> zeroOffsets;
    std::vector<unsigned long> directOffsets;
    std::vector<unsigned long> inDirectOffsets;

    for (OffsetStruct offsetStruct:offsetStructs) {
        if (std::abs(offsetStruct.offset) >= 1) {
            if (offsetStruct.offset > 0) {
                directOffsets.push_back(offsetStruct.index * 2 + 1);
                inDirectOffsets.insert(inDirectOffsets.begin(), offsetStruct.index * 2);
            }
            else {
                directOffsets.push_back(offsetStruct.index * 2);
                inDirectOffsets.insert(inDirectOffsets.begin(), offsetStruct.index * 2 + 1);
            }
        }
        else {
            zeroOffsets.push_back(offsetStruct.index * 2);
            zeroOffsets.push_back(offsetStruct.index * 2 + 1);
        }
    }
    directOffsets.insert(directOffsets.end(), zeroOffsets.begin(), zeroOffsets.end());
    directOffsets.insert(directOffsets.end(), inDirectOffsets.begin(), inDirectOffsets.end());
    return directOffsets;
}

std::vector<unsigned long> LocalPathFinder::_getOrderedOffsetIndexes(std::vector<int> &offsetList)
{
    //  info_msg("get size ordered offset indexes");
    if (!_sceneWrapper->isSingleRobot())
        return _getOrderedOffsetIndexesMultiRobot(offsetList);
    else
        return _getSizeOrderedOffsetIndexesSingleRobot(offsetList);

}

void LocalPathFinder::_init(std::shared_ptr<SceneWrapper> sceneWrapper,
                            bool showTrace,
                            unsigned int maxOpenSetSize,
                            unsigned int discretCnt,
                            unsigned int maxNodeCnt,
                            unsigned int kG,
                            unsigned int kH,
                            unsigned int kC,
                            unsigned int kD
)
{

    assert(sceneWrapper);
    _kG = kG;
    _kH = kH;
    _kC = kC;
    _kD = kD;


    _sceneWrapper = sceneWrapper;
    _showTrace = showTrace;
    _discretCnt = discretCnt;
    _maxOpenSetSize = maxOpenSetSize;
    _actuators = sceneWrapper->getActuators();
    _groupedSteps.clear();
    _steps.clear();
    _maxNodeCnt = maxNodeCnt;


    _actuatorRanges = _sceneWrapper->getActuratorIndexRanges();


    for (const auto &sceneDescription:sceneWrapper->getSceneDescriptions()) {
        std::vector<double> localSteps;
        for (const auto &actuator:sceneDescription->getActuators()) {
            double step = (actuator->maxAngle - actuator->minAngle) / discretCnt;
            localSteps.emplace_back(step);
            _steps.emplace_back(step);
            //info_msg("step ",step," delta ",(actuator->maxAngle - actuator->minAngle));
        }
        _groupedSteps.emplace_back(localSteps);
    }

    _startState.empty();
    _endState.empty();
    for (unsigned long i = 0; i < _sceneWrapper->getActuatorCnt(); i++) {
        _startState.emplace_back(0.0);
        _endState.emplace_back(0.0);
    }

    _offsetList.clear();
    //_groupedFullOffsetList.clear();
    _groupedOffsetList.clear();

    _buildOffsetList();

    info_msg("inited; dCnt: ", _discretCnt, " kG=",
             _kG, " kH=", _kH, " kC=", _kC, " kD=", _kD);

    _ready = true;
}

// constructor with arguments:
// robot - a SceneWrapper class object needs for distance, limit and collider calculation
// discretCnt - count of grid nodes in one dimention
LocalPathFinder::LocalPathFinder(std::shared_ptr<SceneWrapper> sceneWrapper,
                                 bool showTrace,
                                 unsigned int maxOpenSetSize,
                                 unsigned int discretCnt,
                                 unsigned int maxNodeCnt,
                                 unsigned int maxLockingRepeatCnt,
                                 unsigned int kG,
                                 unsigned int kH,
                                 unsigned int kC,
                                 unsigned int kD)
    : PathFinder(sceneWrapper, showTrace), _maxLockingRepeatCnt(maxLockingRepeatCnt)
{
    _discretCnt = discretCnt;
    _init(std::move(sceneWrapper),
          showTrace,
          maxOpenSetSize,
          discretCnt,
          maxNodeCnt,
          kG, kH, kC, kD);
}

// build offset list only with one-chord offset
void LocalPathFinder::_buildOffsetList()
{
    _offsetList.clear();
    std::vector<int> tmpVec;
    for (unsigned int i = 0; i < _sceneWrapper->getActuatorCnt(); i++)
        tmpVec.push_back(0);

    for (unsigned int i = 0; i < _sceneWrapper->getActuatorCnt(); i++) {
        _offsetStandartIndexes.push_back(i * 2);
        _offsetStandartIndexes.push_back(i * 2 + 1);
        tmpVec.at(i) = 1;
        _offsetList.push_back(tmpVec);
        tmpVec.at(i) = -1;
        _offsetList.push_back(tmpVec);
        tmpVec.at(i) = 0;
    }

    for (const auto &sceneDescription: _sceneWrapper->getSceneDescriptions()) {
        std::vector<std::vector<int>> localOffsetList;
        std::vector<int> tmpVec2;
        for (unsigned int i = 0; i < sceneDescription->getActuatorCnt(); i++)
            tmpVec.push_back(0);

        for (unsigned int i = 0; i < sceneDescription->getActuatorCnt(); i++) {
            tmpVec.at(i) = 1;
            localOffsetList.push_back(tmpVec2);
            tmpVec.at(i) = -1;
            localOffsetList.push_back(tmpVec2);
            tmpVec.at(i) = 0;
        }
        _groupedOffsetList.emplace_back(localOffsetList);
    }

}

// add new node
std::shared_ptr<PathNode>
LocalPathFinder::_createNode(std::vector<int> &chords,
                             double g,
                             double h,
                             double c,
                             double d,
                             std::shared_ptr<PathNode> parent)
{
    info_msg("LocalPathFinder::_createNode");
    std::shared_ptr<PathNode> pathNodePtr = std::make_shared<PathNode>(chords, parent, g, h, c, d);
    _openSet.insert(PathNodePtr(pathNodePtr));
    return pathNodePtr;
}

// build code of positive integer chords
unsigned long LocalPathFinder::_getChordCode(std::vector<int> &chords)
{
    assert(chords.size() == _sceneWrapper->getActuatorCnt());
    unsigned long code = 0;
    int m = 1;
    for (int chord : chords) {
        code += m * chord;
        m *= _discretCnt;
    }
    return code;
}

// move node from set of open nodes to the set of closed
void LocalPathFinder::_moveNodeFromOpenedToClosed(std::shared_ptr<PathNode> node)
{
    long code = _getChordCode(node->chords);
    _closedCodeSet.insert(code);
    std::multiset<PathNodePtr>::iterator it;

    // we need find element directly, because our comparator compare only g value,
    // but here we need compare chords, not g value
    for (it = _openSet.begin(); it != _openSet.end() && !_isEqual(it->ptr->chords, node->chords); it++);
    if (it != _openSet.end())
        _openSet.erase(it);

}

// find node in set of closed nodes
bool LocalPathFinder::_findNodeInClosedList(std::vector<int> &chords)
{
    long code = _getChordCode(chords);
    return (_closedCodeSet.end() != _closedCodeSet.find(code));
}

// find node in set of opened nodes
bool LocalPathFinder::_findNodeInOpenedList(std::vector<int> &chords)
{
    for (const PathNodePtr &node:_openSet)
        if (_isEqual(node.ptr->chords, chords))
            return true;
    return false;
}

// convert chords to state
std::vector<double> LocalPathFinder::chordToState(std::vector<int> &chords) const {
    std::vector<double> state;
    for (unsigned int i = 0; i < chords.size(); i++) {
        state.push_back(chords.at(i) * _steps.at(i) + _actuators.at(i)->minAngle);
    }
    return state;
}

std::vector<int> LocalPathFinder::_findFreePoint(std::vector<int> chords,
                                                 unsigned long pos,
                                                 unsigned long maxPos,
                                                const std::vector<double> & startState)
{
    // info_msg("pos: ",pos," maxPos: ",maxPos);

    if (pos > maxPos)
        return std::vector<int>();

    std::vector<int> tmpChords = chords;
    for (int i = -1; i <= 1; i++) {
        tmpChords.at(pos) = chords.at(pos) + i;
//        info_msg("pos: ",pos," i: ",i);
        //   dispChords(tmpChords, "tmp chords");
        //  SceneWrapper::dispState(chordToState(tmpChords), "tmp state");
        if (_checkChords(tmpChords) && _checkPath(chordToState(tmpChords),startState)) {
            // info_msg("chords fucking good");
            return tmpChords;
        }
        else {
            if (pos < chords.size() - 1) {
                auto newChords = _findFreePoint(tmpChords, pos + 1, maxPos,startState);
                if (!newChords.empty()) {
                    return newChords;
                }
            }
        }
    }

    return std::vector<int>();
}

// convert state to chords
std::vector<int> LocalPathFinder::stateToChords(std::vector<double> state)
{

    std::vector<int> chords;
    for (unsigned int i = 0; i < state.size(); i++) {
        double v = state.at(i);
        chords.push_back((int) ((v - _actuators.at(i)->minAngle) / _steps.at(i)));
        //info_msg(i," ",_steps.at(i));
    }

    // info_msg(_actuatorRanges.size());

    std::vector<bool> robotReady;
    for (unsigned long i = 0; i < _sceneWrapper->getRobotCnt(); i++)
        robotReady.emplace_back(false);

    int readyRobotCnt = 0;
    int prevReadyRobotCnt = -1;
    // info_msg(_sceneWrapper->getRobotCnt());
    while (prevReadyRobotCnt != readyRobotCnt) {
        prevReadyRobotCnt = readyRobotCnt;
        for (unsigned long i = 0; i < _sceneWrapper->getRobotCnt(); i++) {
            if (!robotReady.at(i)) {
                auto newChords = _findFreePoint(chords,
                                                _actuatorRanges.at(i).front(),
                                                _actuatorRanges.at(i).back(),
                                                state
                );
                if (newChords.empty()) {
                    // info_msg("new chords are empty");
                    continue;
                }
                readyRobotCnt++;
                robotReady.at(i) = true;
                for (unsigned long j = _actuatorRanges.at(i).front();
                     j <= _actuatorRanges.at(i).back();
                     j++
                    ) {
                    chords.at(j) = newChords.at(j);
                }
            }
        }
    }

    if (readyRobotCnt == _sceneWrapper->getRobotCnt()) {
        // info_msg("OK");
        return chords;
    }


    //info_msg("Partly Error");
    return std::vector<int>();

}

// check if choords are enabled
bool LocalPathFinder::_checkChords(std::vector<int> chords)
{
    // info_msg(chords.size(), " ", _sceneWrapper->getActuatorCnt());
    assert(chords.size() == _sceneWrapper->getActuatorCnt());
    for (int chord : chords) {
        if (chord < 0 || chord >= _discretCnt) {
//            info_msg("chords are disabled by path finder");
//            info_msg(_discretCnt," ",chord);
            return false;
        }
    }
    return isStateEnabled(chordToState(chords));
}

// check if choords are enabled
bool LocalPathFinder::_checkChords(std::vector<int> chords, unsigned int robotNum)
{
    //   info_msg(chords.size() , " ", _dimension);

    for (int chord : chords) {
        if (chord < 0 || chord >= (int) _discretCnt) {
            //info_msg("chords are disabled");
            return false;
        }
    }
    info_msg("begin state ");
    std::vector<double> state = _chordToState(chords, robotNum);
    SceneWrapper::dispState(state, "check state");

    return _sceneWrapper->isStateEnabled(state, robotNum);
}

// find distance method
double LocalPathFinder::_findDistance(std::vector<int> &a, std::vector<int> &b)
{
    return _findChordSpaceDistance(a, b);
}

double LocalPathFinder::_findWeightDistance(std::vector<int> &a, std::vector<int> &b)
{
    //info_msg("LocalPathFinder::_findWeightDistance");
    assert(a.size() == b.size());

    std::vector<double> posesA = _sceneWrapper->getAllPoses(
        chordToState(a)
    );
    std::vector<double> posesB = _sceneWrapper->getAllPoses(
        chordToState(b)
    );
//    dispChords(a,"chordsA");
//    dispChords(b,"chordsB");

//    SceneWrapper::dispState(chordToState(a),"stateA");
//    SceneWrapper::dispState(chordToState(b),"stateB");
//    SceneWrapper::dispState(posesA,"posesA");
//    SceneWrapper::dispState(posesB,"posesB");

    assert(posesA.size() == posesB.size());

    double sum = 0;
    for (unsigned long i = 0; i < posesA.size(); i++) {
        // info_msg(posesA.at(i) - posesB.at(i));
        sum += std::abs(posesA.at(i) - posesB.at(i));
    }

    //info_msg("sum: ",sum);

    return sum;
}

// find distance between two points in chords space
double LocalPathFinder::_findChordSpaceDistance(std::vector<int> &a, std::vector<int> &b)
{
//
//    dispChords(a,"a chords");
//    dispChords(b,"b chords");
    assert(a.size() == b.size());
    double sum = 0;
    unsigned long size = a.size();
    for (unsigned long i = 0; i < size; i++) {
        sum += (b.at(i) - a.at(i)) * (b.at(i) - a.at(i));
    }
    return sum;
}

// check if two set of chord are equal
bool LocalPathFinder::_isEqual(std::vector<int> &a, std::vector<int> &b)
{
//    dispChords(a,"a");
//    dispChords(b,"b");
    assert(a.size() == b.size());
    unsigned long size = a.size();
    for (unsigned int i = 0; i < size; i++) {
        if (a.at(i) != b.at(i))
            return false;
    }
    return true;
}

// sum sets of chords
std::vector<int> LocalPathFinder::_sumChords(const std::vector<int> &a,
                                             const std::vector<int> &b)
{
    assert(a.size() == b.size());
    std::vector<int> result;
    unsigned long size = a.size();
    for (unsigned int i = 0; i < size; i++) {
        result.push_back(a.at(i) + b.at(i));
    }
    return result;
}
// diff sets of chords
std::vector<int> LocalPathFinder::_diffChords(const std::vector<int> &a,
                                              const std::vector<int> &b)
{
    assert(a.size() == b.size());
    std::vector<int> result;
    unsigned long size = a.size();
    for (unsigned int i = 0; i < size; i++) {
        result.emplace_back(a.at(i) - b.at(i));
    }
    return result;
}

std::vector<int> LocalPathFinder::multChord(const std::vector<int> &a,
                                            int scalar)
{
    std::vector<int> result;
    unsigned long size = a.size();
    for (unsigned int i = 0; i < size; i++) {
        result.emplace_back(a.at(i) * scalar);
    }
    return result;
}

// service function for displaying chords vector
void LocalPathFinder::dispChords(std::vector<int> joints, const char *caption, bool flgDisp)
{
    if (flgDisp) {
        assert(!joints.empty());
        std::string msg = caption;
        msg += ":{";
        char buf[256];
        for (int joint: joints) {
            sprintf(buf, "%d ,", joint);
            msg += buf;
        }
        msg = msg.substr(0, msg.size() - 2) + "}";
        info_msg(msg);
    }
}

// service function for displaying chords vector
void LocalPathFinder::dispChords(std::vector<unsigned int> joints, const char *caption, bool flgDisp)
{
    if (flgDisp) {
        assert(!joints.empty());
        std::string msg = caption;
        msg += ":{";
        char buf[256];
        for (unsigned int &joint: joints) {
            sprintf(buf, "%d ,", joint);
            msg += buf;
        }
        msg = msg.substr(0, msg.size() - 2) + "}";
        info_msg(msg);
    }
}

std::vector<double> LocalPathFinder::_chordToState(std::vector<int> chords, unsigned long robotNum)
{
    assert(robotNum >= 0);
    dispChords(chords, "convert");
    std::vector<double> state;
    for (unsigned int i = 0; i < 6; i++) {
        state.push_back(chords.at(i) * _steps.at(i) + _actuators.at(i)->minAngle);
    }
    return state;
}

int LocalPathFinder::_getCollisionDistance(std::vector<int> &chords)
{

    struct VectorHash
    {
        size_t operator()(const std::vector<int> &v) const
        {
            std::hash<int> hasher;
            size_t seed = 0;
            for (int i : v) {
                seed ^= hasher(i) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
            }
            return seed;
        }
    };
    using MySet = std::unordered_set<std::vector<int>, VectorHash>;

    MySet openSet{chords};

    int level = 0;
    MySet newOpenSet;

    while (level < 20) {
        newOpenSet.clear();
        for (auto &newChords:openSet) {
            if (!_checkChords(newChords))
                return level;
            for (auto &offset:_offsetList) {
                newOpenSet.insert(_sumChords(newChords, offset));
            }
        }
        openSet.clear();
        openSet.insert(newOpenSet.begin(), newOpenSet.end());
        level++;
    }
    return level;
}

std::shared_ptr<PathNode> LocalPathFinder::getNeighborPtr(
    std::vector<int> newChords, std::shared_ptr<PathNode> currentNode,
    std::vector<int> &endChords)
{
    if (_checkChords(newChords)) {
        if (_findNodeInClosedList(newChords) || _findNodeInOpenedList(newChords)) {
            //info_msg("finded in lists");
            return nullptr;
        }

        // sum=g+h-c
        double g; // distance
        double h; // heuristic
        double c; // collision distance
        double d; // collision distance

        _getPathNodeWeight(newChords, currentNode, endChords, g, h, c, d);

        return std::make_shared<PathNode>(newChords, currentNode, g, h, c, d);

    }
    else {
        return nullptr;
    }
}

bool LocalPathFinder::findTick(std::vector<double> &state, std::string &logMsg)
{
    // info_msg("findTick");

    if (_openSet.empty()) {
        _errorCode = ERROR_CAN_NOT_FIND_PATH;
        return true;
    }

    std::shared_ptr<PathNode> currentNode = _openSet.begin()->ptr;

    _loopNodes.emplace_back(currentNode);

    if (_isEqual(currentNode->chords, _endChords)) {
        _errorCode = NO_ERROR;
        return true;
    }

    state = chordToState(currentNode->chords);

    assert(_checkChords(currentNode->chords));
    _moveNodeFromOpenedToClosed(currentNode);
    std::vector<int> deltaChords = _diffChords(_endChords, currentNode->chords);

    int problemRobotPos = _checkLockingOffsets(deltaChords, _maxLockingRepeatCnt);

    if (problemRobotPos >= 0) {
        // errorCode = problemRobotPos;
        _errorCode = ERROR_LOCKING_OFFSETS;
        return false;
    }

    if (_closedCodeSet.size() > _maxNodeCnt) {
        info_msg("reached max node cnt");
        _errorCode = ERROR_REACHED_MAX_NODE_CNT;
        return false;
    }
    logMsg = currentNode->toString();
    if (_showTrace) {

        //   dispChords(currentNode->chords, "current node");
        info_msg(currentNode->toString(),
                 " openSet:",
                 _openSet.size(),
                 " closedSet: ",
                 _closedCodeSet.size());
        dispChords(deltaChords, "delta node");
        SceneWrapper::dispState(chordToState(currentNode->chords), "actual state");
    }

    auto endNode = _forEachNode(currentNode, _endChords);

    if (endNode) {
        _endNode = endNode;
        return true;
    }
    return false;
}

std::vector<std::vector<double>> LocalPathFinder::getFindingPoses() const {
    std::vector<std::vector<double>> findingPoses;
    for(auto & node:_loopNodes){
        findingPoses.emplace_back(chordToState(node->chords));
    }
    return  findingPoses;
}

void LocalPathFinder::prepareFindTick(const std::vector<double> &startState,
                                      const std::vector<double> &endState)
{
    _startState = startState;
    _endState = endState;

    _loopNodes.clear();

    info_msg("LocalPathFinder::prepareFindTick");

    SceneWrapper::dispState(startState, "startState");
    SceneWrapper::dispState(endState, "endState");

    _startChords = stateToChords(startState);
    _endChords = stateToChords(endState);

    if (_startChords.empty()) {
        _errorCode = ERROR_CAN_NOT_FIND_FREE_START_POINT;
        return;
    }
    if (_endChords.empty()) {
        _errorCode = ERROR_CAN_NOT_FIND_FREE_END_POINT;
        return;
    }

    assert(_checkChords(_startChords));
    assert(_checkChords(_endChords));

    info_msg("converted with discretCnt: ", _discretCnt);

    dispChords(_startChords, "start chords");
    dispChords(_endChords, "end chords");

    _startStateFromChords = chordToState(_startChords);
    _endStateFromChords = chordToState(_endChords);
    SceneWrapper::dispState(_startStateFromChords, "startStateFromChords");
    SceneWrapper::dispState(_endStateFromChords, "endStateFromChords");

    if (_isEqual(_startChords, _endChords)) {
        assert(false);
    }

    _openSet.clear();
    _closedCodeSet.clear();
    _prevCheckingOffsetList.clear();
    _oneNonUnitOffsetCnt = 0;
    _nonModifiedCnt = 0;


    // create start node
    std::shared_ptr<PathNode> start =
        _createNode(_startChords,
                    _findDistance(_startChords, _endChords) * _kG,
                    0,
                    _getCollisionDistance(_startChords) * _kC,
                    0,
                    std::shared_ptr<PathNode>());

    if (!_checkChords(_startChords)) {
        //errorCode = ERROR_START_CHORDS_ARE_DISABLED;
        assert(false);
    }
    if (!_checkChords(_endChords)) {
        //errorCode = ERROR_END_CHORDS_ARE_DISABLED;
        assert(false);
    }
    info_msg("prepared");
}

void LocalPathFinder::buildPath()
{
    assert(_endNode);

    _path.clear();

    // move between nodes in reverse order: from end node to start node
    while (_endNode) {
        // always insert new node at the begin of list
        std::vector<double> state = chordToState(_endNode->chords);
        _path.insert(_path.begin(), state);
        _endNode = _endNode->parent;
    }

    // add real start and end points
    _path.insert(_path.begin(), _startStateFromChords);
    _path.insert(_path.begin(), _startState);
    _path.emplace_back(_endStateFromChords);
    _path.emplace_back(_endState);
}

const std::vector<double> &LocalPathFinder::getSteps() const
{
    return _steps;
}
const std::vector<double> &LocalPathFinder::getStartStateFromChords() const
{
    return _startStateFromChords;
}

const std::vector<double> &LocalPathFinder::getEndStateFromChords() const
{
    return _endStateFromChords;
}



