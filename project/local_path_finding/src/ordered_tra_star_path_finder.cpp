#include "ordered_tra_star_path_finder.h"


std::shared_ptr<PathNode> OrderedAStarPathFinder::_forEachNode(
    std::shared_ptr<PathNode> currentNode,
    std::vector<int> &endChords
)
{
    // info_msg("for each node");
    std::vector<int> chordsOffset = _diffChords(endChords, currentNode->chords);

    std::vector<unsigned long> sizeOrderedOffsetIndexes;
    if (_checkUnitOffsets(chordsOffset)) {
        sizeOrderedOffsetIndexes = _offsetStandartIndexes;
    }
    else {
        sizeOrderedOffsetIndexes = _getOrderedOffsetIndexes(chordsOffset);
    }

    unsigned long listSize = sizeOrderedOffsetIndexes.size();

    for (unsigned int i = 0; i < listSize; i++) {
        std::vector<int> &offset = _offsetList.at(sizeOrderedOffsetIndexes.at(i));

        std::vector<int> newChords = _sumChords(currentNode->chords, offset);

        std::shared_ptr<PathNode> newNode = getNeighborPtr(newChords, currentNode, endChords);

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
    return nullptr;
}

