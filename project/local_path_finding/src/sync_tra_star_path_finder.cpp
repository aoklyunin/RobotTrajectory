#include "sync_tra_star_path_finder.h"
#include "traces.h"
#include <thread>
#include <future>
#include <functional>
#include <utility>
#include <tbb/tbb.h>


void getNeighbor(std::promise<std::shared_ptr<PathNode>> prm,
                 std::vector<int> newChords,
                 std::shared_ptr<PathNode> currentNode,
                 std::vector<int> &endChords, SyncAStarPathFinder *pointer)
{

    std::shared_ptr<PathNode> ptr = pointer->getNeighborPtr(
        std::move(newChords),
        std::move(currentNode),
        endChords
    );
    //std::shared_ptr<PathNode> ptr = nullptr;
    prm.set_value_at_thread_exit(ptr);
}


std::shared_ptr<PathNode>
SyncAStarPathFinder::_forEachNode(std::shared_ptr<PathNode> currentNode, std::vector<int> &endChords)
{
    std::vector<std::future<std::shared_ptr<PathNode>>> futures;
    std::vector<std::promise<std::shared_ptr<PathNode>>> promises;
    std::vector<std::thread> threads;

    for (const auto &offset:_offsetList) {
        std::vector<int> newChords = _sumChords(currentNode->chords, offset);
        std::shared_ptr<PathNode> newNode = getNeighborPtr(newChords, currentNode, endChords);

        std::promise<std::shared_ptr<PathNode>> promise;
        futures.push_back(promise.get_future());

        std::thread
            thread(getNeighbor, std::move(promise), std::move(newChords), currentNode, std::ref(endChords), this);
        thread.detach();
        threads.push_back(std::move(thread));
    }


    for (unsigned int i = 0; i < _offsetList.size(); i++) {
        std::shared_ptr<PathNode> newNode = futures.at(i).get();

        if (newNode) {

            if (_isEqual(newNode->chords, endChords)) {
                return newNode;
            }

            _openSet.insert(PathNodePtr(newNode));
            if (_maxOpenSetSize != 0 && _openSet.size() > _maxOpenSetSize)
                _openSet.erase(std::prev(_openSet.end()));

            continue;
        }

    }
    return std::shared_ptr<PathNode>();
}
