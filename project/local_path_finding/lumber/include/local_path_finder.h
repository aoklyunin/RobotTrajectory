#pragma once

#include <memory>
#include <unordered_map>
#include <set>
#include <utility>
#include <unordered_set>
#include <scene_wrapper.h>
#include "path_finder.h"

#include <iostream>

// node for path_finding finding algorithm
class PathNode
{
public:


    PathNode(const std::vector<int> &chords,
             std::shared_ptr<PathNode> parent,
             double g, double h, double c, double d)
        : chords(chords), parent(parent),
          g(g), h(h), c(c), d(d), chordCnt(chords.size())
    {
        sum = g + h + c + d;

        if (!parent)
            order = 0;
        else
            order = parent->order + 1;
    }

    std::string toString()
    {
        std::string result = "{";
        char buf[256];
        sprintf(buf, "sum:%.3f, g:%.3f, h:%.3f, c:%.3f, d:%.3f", sum, g, h, c, d);
        result += buf;
        return result + "}";
    }

    unsigned long chordCnt;
    std::vector<int> chords;
    std::shared_ptr<PathNode> parent;
    double g;
    double h;
    double c;
    double d;
    double sum;

    unsigned int order;

};

// node ptr for sorted set of open nodes
class PathNodePtr
{
public:
    explicit PathNodePtr(std::shared_ptr<PathNode> ptr)
        : ptr(std::move(ptr))
    {}
    std::shared_ptr<PathNode> ptr;
    bool operator<(const PathNodePtr &obj) const
    {
        return (this->ptr->sum < obj.ptr->sum);
    }
};

// main class for finding path_finding
class LocalPathFinder: public PathFinder
{
public:

    explicit LocalPathFinder(std::shared_ptr<SceneWrapper> sceneWrapper,
                             bool showTrace,
                             unsigned int maxOpenSetSize,
                             unsigned int discretCnt,
                             unsigned int maxNodeCnt,
                             unsigned int maxLockingRepeatCnt,
                             unsigned int kG,
                             unsigned int kH,
                             unsigned int kC,
                             unsigned int kD

    );

    ~LocalPathFinder() override = default;

    static void dispChords(std::vector<int> joints, const char *caption, bool flgDisp = true);
    static void dispChords(std::vector<unsigned int> joints, const char *caption, bool flgDisp = true);
    static std::vector<int> multChord(const std::vector<int> &a, int scalar);

    std::shared_ptr<PathNode>
    getNeighborPtr(std::vector<int> newChords, std::shared_ptr<PathNode> currentNode, std::vector<int> &endChords);

    // service methods for convesion between chords space and state space
    std::vector<double> chordToState(std::vector<int> &chords);
    std::vector<int> stateToChords(std::vector<double> state);

    void prepareTick(const std::vector<double> &startState, const std::vector<double> &endState) override;
    bool tick(std::vector<double> &state, std::string &logMsg) override;
    void buildPath() override;

    const std::vector<double> &getStartStateFromChords() const;
    const std::vector<double> &getEndStateFromChords() const;

    const std::vector<double> &getSteps() const;

protected:


    std::shared_ptr<PathNode> _endNode;

    unsigned int _kG;
    unsigned int _kH;
    unsigned int _kC;
    unsigned int _kD;

    virtual std::shared_ptr<PathNode> _forEachNode(
        std::shared_ptr<PathNode> currentNode,
        std::vector<int> &endChords
    ) = 0;

    virtual void _getPathNodeWeight(std::vector<int> newChords,
                                    std::shared_ptr<PathNode> currentNode,
                                    std::vector<int> &endChords,
                                    double &g, double &h, double &c, double &d) = 0;

    std::vector<unsigned long> _getOrderedOffsetIndexesMultiRobot(std::vector<int> &offsetList);

    std::vector<std::shared_ptr<Actuator>> _actuators;

    std::vector<unsigned long> _offsetStandartIndexes;

    std::vector<std::vector<std::vector<int>>> _groupedOffsetList;
    std::vector<std::vector<int>> _offsetList;

    std::vector<std::vector<double>> _groupedSteps;

    unsigned int _maxOpenSetSize;
    unsigned int _maxLockingRepeatCnt;

    std::shared_ptr<SceneWrapper> _sceneWrapper;

    std::vector<unsigned long> _getSizeOrderedOffsetIndexesSingleRobot(std::vector<int> &offsetList);

    int _checkLockingOffsets(std::vector<int> &offsetList, unsigned int maxRepeatCnt);
    bool _checkUnitOffsets(std::vector<int> &offsetList);

    unsigned int _oneNonUnitOffsetCnt;
    std::vector<int> _prevCheckingOffsetList;
    unsigned int _nonModifiedCnt;

    std::vector<int> _findFreePoint(std::vector<int> chords,
                                    unsigned long pos,
                                    unsigned long maxPos,
                                    const std::vector<double> &startState);

    // variables and methods for working with nodes
    std::unordered_set<long> _closedCodeSet;
    std::multiset<PathNodePtr> _openSet;
    unsigned long _getChordCode(std::vector<int> &chords);
    void _buildOffsetList();

    std::shared_ptr<PathNode>
    _createNode(std::vector<int> &chords, double g, double h, double c, double d, std::shared_ptr<PathNode> parent);
    void _moveNodeFromOpenedToClosed(std::shared_ptr<PathNode> node);
    bool _findNodeInClosedList(std::vector<int> &chords);
    bool _findNodeInOpenedList(std::vector<int> &chords);

    unsigned int _discretCnt;

    std::vector<double> _steps;
    int _getCollisionDistance(std::vector<int> &chords);

    bool _checkChords(std::vector<int> chords);
    double _findDistance(std::vector<int> &a, std::vector<int> &b);

    // service functions for working with int chords
    static bool _isEqual(std::vector<int> &a, std::vector<int> &b);
    static std::vector<int> _sumChords(const std::vector<int> &a, const std::vector<int> &b);
    static std::vector<int> _diffChords(const std::vector<int> &a, const std::vector<int> &b);

    std::vector<unsigned long> _getOrderedOffsetIndexes(std::vector<int> &offsetList);

    unsigned int _maxNodeCnt;

    std::vector<int> _startChords;
    std::vector<int> _endChords;

    double _findWeightDistance(std::vector<int> &a, std::vector<int> &b);
private:


    std::vector<double> _startStateFromChords;
    std::vector<double> _endStateFromChords;
    // variables and methods for path_finding finding
    double _findChordSpaceDistance(std::vector<int> &a, std::vector<int> &b);

    bool _checkChords(std::vector<int> chords, unsigned int robotNum);
    std::vector<double> _chordToState(std::vector<int> chords, unsigned long robotNum);

    void _init(std::shared_ptr<SceneWrapper> sceneWrapper,
               bool showTrace,
               unsigned int maxOpenSetSize,
               unsigned int discretCnt,
               unsigned int maxNodeCnt,
               unsigned int kG,
               unsigned int kH,
               unsigned int kC,
               unsigned int kD
    );

    std::vector<std::vector<unsigned long>> _actuatorRanges;

};
