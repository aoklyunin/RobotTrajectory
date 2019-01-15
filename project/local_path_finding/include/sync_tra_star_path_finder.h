#pragma once

#include <memory>
#include <unordered_map>
#include <set>
#include <utility>
#include <unordered_set>
#include <local_path_finder.h>
#include <future>
#include <solid_collider.h>
#include <solid_sync_collider.h>
#include "scene_wrapper.h"
#include "tra_star_path_finder.h"


class SyncAStarPathFinder: public AStarPathFinder
{
public:
    explicit SyncAStarPathFinder(std::shared_ptr<SceneWrapper> sceneWrapper,
                                 bool showTrace,
                                 unsigned int maxOpenSetSize,
                                 unsigned int startDiscretCnt,
                                 unsigned int maxNodeCnt,
                                 unsigned int maxLockingRepeatCnt,
                                 unsigned int kG,
                                 unsigned int kH,
                                 unsigned int kC,
                                 unsigned int kD,
                                 unsigned int threadCnt)
        :
        AStarPathFinder(std::move(sceneWrapper),
                        showTrace,
                        maxOpenSetSize,
                        startDiscretCnt,
                        maxNodeCnt,
                        maxLockingRepeatCnt,
                        kG, kH, kC,kD)
    {
        _collider = std::make_shared<SolidSyncCollider>(threadCnt);
        _collider->init(sceneWrapper->getGroupedModelPaths());

        std::vector<std::vector<int>> group;
        int cnt = 0;
        for (auto & offset:_offsetList){
            group.emplace_back(offset);
            cnt++;
            if (cnt>=threadCnt){
                _groupedOffsetList.emplace_back(group);
                group.clear();
                cnt = 0;
            }

        }
    };

protected:

    std::shared_ptr<PathNode> _forEachNode(
        std::shared_ptr<PathNode> currentNode,
        std::vector<int> &endChords
    ) override;

    std::vector<std::vector<std::vector<int>>> _groupedOffsetList;
};
