#include <fstream>
#include <solid_collider.h>
#include <tra_star_path_finder.h>
#include "path_reader.h"

void PathReader::init(std::string path)
{

    Json::Reader reader;
    Json::Value obj;

    std::ifstream ifs((path).c_str(), std::ios_base::binary);
    std::string content((std::istreambuf_iterator<char>(ifs)),
                        (std::istreambuf_iterator<char>()));

    reader.parse(content, obj); // reader can also read strings
    std::string scenePath = obj["scene"].asString();

    info_msg("Scene path: ", scenePath);

    const Json::Value &data = obj["data"]; // array of characters

    int i = 0;


    _pathStates.clear();

    for (Json::Value path:data) {

        info_msg("test num ", ++i);
        std::vector<PathState> oneExpPathStates;
        for (Json::Value record: path["states"]) {
            //info_msg("get states");
            std::vector<double> state;
            for (Json::Value chord:record["state"]) {
                //info_msg("get chord");
                state.emplace_back(chord.asDouble());
            }
            double ts = record["ts"].asDouble();
            SceneWrapper::dispState(state, "readed state");
            PathState ps(state, ts);
            ps.disp();
            oneExpPathStates.emplace_back(ps);

        }
        _pathStates.emplace_back(oneExpPathStates);
    }

    std::shared_ptr<SceneWrapper> sceneWrapper = std::make_shared<SceneWrapper>();
    sceneWrapper->buildFromFile(scenePath);

    _pathFinder = std::make_shared<AStarPathFinder>(sceneWrapper, true, 100, 10, 4000, 25, 1, 1, 0,2);

    std::cout << scenePath << std::endl;
}

const std::vector<std::vector<PathState>> &PathReader::getPathStates() const
{
    return _pathStates;
}

std::vector<double> PathReader::getState(double tm, unsigned long expNum)
{
    //  info_msg("get state tm=",tm);

//    info_msg(_pathStates.size());

    // info_msg("get state path reader");
    assert (!_pathStates.empty());
    assert (!_pathStates.at(expNum).empty());

    // info_msg("exp path size: ",_pathStates.at(expNum).size());

    for (unsigned int i = 0; i < _pathStates.at(expNum).size(); i++) {

        PathState &pathState = _pathStates.at(expNum).at(i);
        // info_msg("path state geted");

        // info_msg(pathState.ts," ",tm);
        if (tm > pathState.ts * _timeScale) {
            //    info_msg("greater");
            continue;
        }
        if (std::abs(pathState.ts * _timeScale - tm) < 0.001) {
            //        info_msg("equals");
            return pathState.state;
        }
        // tm<ps.ts
        assert(i >= 1);
        //   info_msg("pathState size ",pathState.state.size());

        PathState &prevState = _pathStates.at(expNum).at(i - 1);

        //  info_msg("prevState size ",prevState.state.size());
//        info_msg(i);
//        SceneWrapper::dispState(prevState.state,"prev state");
//        SceneWrapper::dispState(pathState.state,"path state");


        double delta = pathState.ts * _timeScale - prevState.ts * _timeScale;

        //  info_msg("delta=",delta);

        std::vector<double> deltaChord = SceneWrapper::diffStates(pathState.state, prevState.state);

        std::vector<double> speed = SceneWrapper::multState(deltaChord, 1 / delta);
        auto resultState =
            SceneWrapper::sumStates(prevState.state, SceneWrapper::multState(speed, tm - prevState.ts * _timeScale));
        //  info_msg("result state size ",resultState.size());
        return resultState;
    }
    return _pathStates.at(expNum).back().state;
}

double PathReader::getDuration(unsigned long expNum)
{
    assert (!_pathStates.empty());

    return (_pathStates.at(expNum).back().ts - _pathStates.at(expNum).front().ts) * _timeScale;
}

