#include "global_path_finder.h"

#include <tra_star_path_finder.h>
#include <ordered_tra_star_path_finder.h>
#include "traces.h"

std::vector<std::vector<double>> GlobalPathFinder::_getCollisionPair(
    const std::vector<std::vector<double>> &points, bool &flgError)
{

    info_msg("GlobalPathFinder::_getCollisionPair");

    flgError = false;
    std::vector<double> beginPoint = *points.begin();

    if (!isStateEnabled(beginPoint)) {
        flgError = true;
        info_msg("begin point is disabled");
        return std::vector<std::vector<double>>();
    }

    std::vector<double> beginPointLoop = beginPoint;

    unsigned int lastCheckedPoint = 0;


    for (unsigned int i = 1; i < points.size(); i++) {
        // info_msg("suffix loop");
        std::vector<double> prevPoint = points.at(i - 1);
        std::vector<double> nextPoint = points.at(i);
        if (checkPath(prevPoint, nextPoint)) {
            beginPointLoop = prevPoint;
            break;
        }
        else {
            lastCheckedPoint = i;
            beginPath.push_back(prevPoint);
        }
    }

    info_msg("begin point finded");

    if (lastCheckedPoint == points.size() - 1) {
        return std::vector<std::vector<double>>();
    }

    std::vector<double> endPoint = points.back();
    std::vector<double> endPointLoop = endPoint;

    if (!isStateEnabled(endPointLoop)) {
        flgError = true;
        info_msg("end point is disabled");
        return std::vector<std::vector<double>>();
    }

    // backward direction
    endPath.insert(endPath.begin(), endPoint);

    assert(!points.empty());

    for (unsigned long i = points.size() - 1; i > lastCheckedPoint; i--) {
        std::vector<double> prevPoint = points.at(i);
        std::vector<double> nextPoint = points.at(i - 1);
        if (checkPath(prevPoint, nextPoint)) {
            endPointLoop = prevPoint;
            break;
        }
        else {
            endPath.insert(endPath.begin(), prevPoint);
        }
    }
    info_msg("end point finded");
    return std::vector<std::vector<double>>{beginPointLoop, endPointLoop};
}

bool GlobalPathFinder::findTick(std::vector<double> &state, std::string &logMsg)
{
    assert(_localPathFinder);
    if (_localPathFinder->findTick(state, logMsg)) {
        _ready = false;
        _errorCode = _localPathFinder->getErrorCode();
        logMsg = "new local path finder";
        if (_errorCode == NO_ERROR) {
            _localPathFinder->buildPath();
            _path = _localPathFinder->getPath();
            showPath(_path);
            info_msg("check path");
            std::vector<std::vector<double>> collisionPair = _getCollisionPair(_path, flgError);

            if (flgError) {
                _errorCode = ERROR_CAN_NOT_FIND_PATH;
                err_msg("error geting collisionPair");
                _ready = true;
                return false;
            }
            else {
                if (collisionPair.empty()) {
                    info_msg("collision pair is empty");
                    _errorCode = NO_ERROR;
                    _ready = true;
                    return true;
                }
            }

            std::vector<double> beforeCollisionPoint = collisionPair.at(0);
            std::vector<double> afterCollisionPoint = collisionPair.at(1);
            _startState = beforeCollisionPoint;
            _endState = afterCollisionPoint;

            SceneWrapper::dispState(beforeCollisionPoint, "before collision point");
            SceneWrapper::dispState(afterCollisionPoint, "after collision point");

            do {
                _minDiscretCnt += DISCRET_STEP;
            //    _kD += K_D_STEP;

                _localPathFinder = std::make_shared<OrderedAStarPathFinder>(
                    _sceneWrapper,
                    _showTrace,
                    OPEN_SET_SIZE,
                    _minDiscretCnt,
                    MAX_NODE_CNT,
                    MAX_LOCKING_CNT,
                    K_G, K_H, K_C, K_D
                );
                _localPathFinder->prepareFindTick(beforeCollisionPoint, afterCollisionPoint);
            }
            while (_localPathFinder->getErrorCode() != NO_ERROR);

            _startState = _localPathFinder->getStartState();
            _endState = _localPathFinder->getEndState();

            _ready = true;

        }
        else {
            _ready = true;
            return true;
        }
    }

    return false;
}

void GlobalPathFinder::prepareFindTick(const std::vector<double> &startState,
                                       const std::vector<double> &endState)
{
    _kD = K_D;
    _startState = startState;
    _endState = endState;
    _flgLocalPathFinding = false;

    _minDiscretCnt = START_MIN_DISCRET_CNT;

    assert(isStateEnabled(startState));
    assert(isStateEnabled(endState));
    do {
        _localPathFinder = std::make_shared<OrderedAStarPathFinder>(
            _sceneWrapper,
            _showTrace,
            OPEN_SET_SIZE,
            _minDiscretCnt,
            MAX_NODE_CNT,
            MAX_LOCKING_CNT,
            K_G, K_H, 0, K_D
        );

        _localPathFinder->prepareFindTick(startState, endState);
        _minDiscretCnt += DISCRET_STEP;
        //_kD += K_D_STEP;
    }
    while (_localPathFinder->getErrorCode() != NO_ERROR);

    _startState = _localPathFinder->getStartState();
    _endState = _localPathFinder->getEndState();
    beginPath.clear();
    endPath.clear();
    middle.clear();
    prevBeforeCollisionPoint.clear();
    prevAfterCollisionPoint.clear();
    _flgRepeated = false;

    _flgLocalPathFinding = true;
    _ready = true;
}

void GlobalPathFinder::buildPath()
{

//    std::vector<std::vector<double>> result = beginPath;
//
//    _errorCode = NO_ERROR;

}
