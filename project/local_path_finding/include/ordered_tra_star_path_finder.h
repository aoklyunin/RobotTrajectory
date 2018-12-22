#pragma once

#include <memory>
#include <unordered_map>
#include <set>
#include <utility>
#include <unordered_set>
#include <local_path_finder.h>
#include <future>
#include "scene_wrapper.h"
#include "tra_star_path_finder.h"


class OrderedAStarPathFinder: public AStarPathFinder
{
public:
    explicit OrderedAStarPathFinder(std::shared_ptr<SceneWrapper> sceneWrapper,
                                    bool showTrace,
                                    unsigned int maxOpenSetSize,
                                    unsigned int startDiscretCnt,
                                    unsigned int maxNodeCnt,
                                    unsigned int maxLockingRepeatCnt,
                                    unsigned int kG,
                                    unsigned int kH,
                                    unsigned int kC,
                                    unsigned int kD)
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

    std::shared_ptr<PathNode> _forEachNode(
        std::shared_ptr<PathNode> currentNode,
        std::vector<int> &endChords
    ) override;

};
