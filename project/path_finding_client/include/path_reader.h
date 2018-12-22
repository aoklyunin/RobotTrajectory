#pragma one

#include <string>
#include <scene_wrapper.h>
#include <path_finder.h>


struct PathState
{

    PathState(const std::vector<double> &_state, double _ts)
    {
        state = _state;
        ts = _ts;
    }
    std::vector<double> state;
    double ts;

    void disp()
    {
        info_msg("path state size: ", state.size());
        SceneWrapper::dispState(state, "state");
        info_msg("ts ", ts);
    }
};

class PathReader
{

public:
    PathReader(double timeScale)
    { _timeScale = timeScale; };
    ~PathReader() = default;

    void init(std::string path);

    std::shared_ptr<PathFinder> getPathFinder()
    {
        return _pathFinder;
    }

    const std::vector<std::vector<PathState>> &getPathStates() const;

    std::vector<double> getState(double tm, unsigned long expNum);

    double getDuration(unsigned long expNum);

    unsigned long getExpCnt()
    { return _pathStates.size(); };

private:
    std::shared_ptr<PathFinder> _pathFinder;
    std::vector<std::vector<PathState>> _pathStates;
    double _timeScale;
};

