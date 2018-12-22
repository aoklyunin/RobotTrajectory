#include "tra_star_path_finder.h"


std::shared_ptr<PathNode> AStarPathFinder::_forEachNode(
    std::shared_ptr<PathNode> currentNode,
    std::vector<int> &endChords
)
{
    for (const std::vector<int> &offset:_offsetList) {
        //  dispChords(offset,"local offset");
        std::vector<int> newChords = _sumChords(currentNode->chords, offset);
        std::shared_ptr<PathNode> newNode =
            getNeighborPtr(newChords, currentNode, endChords);

        if (newNode) {
            //  info_msg("new node");
            if (_isEqual(newNode->chords, endChords)) {
                return newNode;
            }

            //  info_msg(newNode->toString());

            _openSet.insert(PathNodePtr(newNode));
            if (_maxOpenSetSize != 0 && _openSet.size() > _maxOpenSetSize) {
                _openSet.erase(std::prev(_openSet.end()));
            }

        }

    }
    return nullptr;
}

void AStarPathFinder::_getPathNodeWeight(std::vector<int> newChords,
                                         std::shared_ptr<PathNode> currentNode,
                                         std::vector<int> &endChords,
                                         double &g, double &h, double &c, double &d)
{
    if (_kD == 0) {
        d = 0;
    }
    else {
        d = _findWeightDistance(newChords, endChords) * _kD;
//        double newDistance = _findWeightDistance(newChords, endChords);
//        double prevDistance = _findWeightDistance(currentNode->chords, endChords);
//        if (newDistance>prevDistance){
//            d=currentNode->d-_kD;
//        }else{
//            d=currentNode->d+_kD;
//        }
//        if (d < 50)
//            d = 0;
    }

    if (_kG == 0)
        g = 0;
    else {
        g = 0;
        for (unsigned int i = 0; i < newChords.size(); i++) {
            //     info_msg(i," ",(newChords.size()-i)*std::abs(newChords.at(i) - endChords.at(i)));
            g += std::abs(newChords.at(i) - endChords.at(i));
        }
        g = g * _kG;
        // g = g*g;
        //g = _findDistance(newChords, endChords) * _kG;

    }
    if (_kH == 0)
        h = 0;
    else
        h = currentNode->h + _kH;

    if (_kC == 0) {
        c = 0;
    }
    else {
        int cd = _getCollisionDistance(newChords);
        c = (double)(cd * _kC);
    }
}
