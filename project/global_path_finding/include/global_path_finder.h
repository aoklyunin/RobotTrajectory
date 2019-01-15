#pragma once

#include "scene_wrapper.h"
#include "path_finder.h"

#include <memory>
#include <unordered_map>
#include <set>
#include <utility>
#include <unordered_set>
#include <local_path_finder.h>


class GlobalPathFinder: public PathFinder
{
public:
    const unsigned int START_MIN_DISCRET_CNT = 5;
    const unsigned int DISCRET_STEP = 1;
    const unsigned int MAX_DISCRET_CNT = 52;
    const unsigned int MAX_KH = 0;
    const unsigned int OPEN_SET_SIZE = 5000;
    const unsigned int MAX_NODE_CNT = 3000;
    const unsigned int MAX_LOCKING_CNT = 600;

    const unsigned int K_G = 1;
    const unsigned int K_D = 1;

    const unsigned int K_C = 10;

    const unsigned int K_H = 0;

    const unsigned int K_D_STEP = 2;

    explicit GlobalPathFinder(std::shared_ptr<SceneWrapper> sceneWrapper,
                           bool showTrace)
        :
        PathFinder(sceneWrapper,
                   showTrace), _startDist(0.0), _minStep(0.001), _startDiscretCnt(10)
    {};


    bool findTick(std::vector<double> &state, std::string &logMsg) override;

    void prepareFindTick(const std::vector<double> &startState,
                         const std::vector<double> &endState) override;

    void buildPath() override;


protected:

    unsigned int _minDiscretCnt;
    std::vector<std::vector<double>> beginPath;
    std::vector<std::vector<double>> endPath;

    double _startDist;
    double _minStep;

    unsigned int _startDiscretCnt;

    bool flgError;

    std::vector<std::vector<double>> _getCollisionPair(
        const std::vector<std::vector<double>> &points,
        bool &flgError
    );

    std::vector<double> prevBeforeCollisionPoint;
    std::vector<double> prevAfterCollisionPoint;

    std::vector<double> middle;

    bool _flgRepeated;

    bool _flgLocalPathFinding;

    std::shared_ptr<PathFinder> _localPathFinder;

    unsigned int _kD;
};
