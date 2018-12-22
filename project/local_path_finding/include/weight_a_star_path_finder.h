#pragma once

#include <tra_star_path_finder.h>

#include <memory>
#include <unordered_map>
#include <set>
#include <utility>
#include <unordered_set>

#include <future>
#include "scene_wrapper.h"


class WeightAStarPathFinder: public AStarPathFinder
{
public:
    explicit WeightAStarPathFinder(const std::shared_ptr<SceneWrapper> &sceneWrapper,
                             bool showTrace,
                             unsigned int maxOpenSetSize,
                             unsigned int startDiscretCnt,
                             unsigned int maxNodeCnt,
                             unsigned int maxLockingRepeatCnt,
                             unsigned int kG,
                             unsigned int kH,
                             unsigned int kC,
                                   unsigned int kD
    )
        :
        AStarPathFinder(sceneWrapper,
                        showTrace,
                        maxOpenSetSize,
                        startDiscretCnt,
                        maxNodeCnt,
                        maxLockingRepeatCnt,
                        kG,kH,kC,kD)
    {};


protected:

    void _getPathNodeWeight(std::vector<int> newChords,
                            std::shared_ptr<PathNode> currentNode,
                            std::vector<int> &endChords,
                            double &g, double &h, double &c, double &d) override;

};
