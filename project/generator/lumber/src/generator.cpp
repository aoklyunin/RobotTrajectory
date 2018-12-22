#include <climits>
#include <utility>
#include <traces.h>
#include <path_finder.h>
#include <urdf_scene_description.h>
#include <solid_collider.h>
#include "tra_star_path_finder.h"

#include "ordered_tra_star_path_finder.h"
#include "global_path_finder.h"
#include "generator.h"


std::string
Generator::buildStat(std::vector<std::vector<double>> startPoints,
                     std::vector<std::vector<double>> endPoints,
                     std::vector<double> seconds,
                     std::vector<bool> isValid)
{
    //info_msg(startPoints.size()," ", endPoints.size() ," ", seconds.size() ," ", isValid.size());

    // assert(startPoints.size() == endPoints.size() == seconds.size() == isValid.size());

    Json::Value json;


    for (unsigned int i = 0; i < startPoints.size(); i++) {
        Json::Value record;
        for (unsigned int j = 0; j < startPoints.at(i).size(); j++)
            record["start"][j] = startPoints.at(i).at(j);
        for (unsigned int j = 0; j < endPoints.at(i).size(); j++)
            record["end"][j] = endPoints.at(i).at(j);

        record["time"] = seconds.at(i);
        record["isValid"] = isValid.at(i) ? 1 : 0;
        json[i] = record;
    }

    Json::Value result;

    info_msg("scene path ", _pathFinder->getSceneWrapper()->getScenePath());

    char buf[PATH_MAX + 1]; /* not sure about the "+ 1" */
    char *res = realpath(_pathFinder->getSceneWrapper()->getScenePath().c_str(), buf);
    if (res) {
        result["scene"] = buf;
    }
    else {
        perror("realpath");
        exit(EXIT_FAILURE);
    }

    result["data"] = json;

    long validCnt = std::count(_isValid.begin(), _isValid.end(), true);
    long nonValidCnt = std::count(_isValid.begin(), _isValid.end(), false);
    long expCnt = _isValid.size();

    Json::Value agregated;

    agregated["validCnt"] = (int) validCnt;
    agregated["nonValidCnt"] = (int) nonValidCnt;
    agregated["expCnt"] = (int) expCnt;

    result["agregated"] = agregated;

    return result.toStyledString();

}

Json::Value Generator::generateRoutes(unsigned int testCnt)
{
    assert(_pathFinder);
    assert(_pathFinder->getSceneWrapper());


    Json::Value json;
    _startPoints.clear();
    _endPoints.clear();
    _secondsList.clear();
    _isValid.clear();

    for (int i = 0; i < testCnt; i++) {

        std::vector<double> start = _pathFinder->getRandomState();
        std::vector<double> end = _pathFinder->getRandomState();

        json[i] = testPathFinding(start, end);
    }

    Json::Value result;

    char buf[PATH_MAX + 1]; /* not sure about the "+ 1" */
    char *res = realpath(_pathFinder->getSceneWrapper()->getScenePath().c_str(), buf);
    if (res) {
        result["scene"] = buf;
    }
    else {
        perror("realpath");
        exit(EXIT_FAILURE);
    }

    result["data"] = json;

    return result;
}

void Generator::writeResult(std::string routePath, std::string reportPath, unsigned int testCnt)
{
    // info_msg("write results");

    auto result = generateRoutes(testCnt);
    std::ofstream myfile;
    myfile.open(routePath);
    myfile << result.toStyledString();
    myfile.close();

//    info_msg("write report");

    if (!reportPath.empty()) {
        std::string stat = buildStat(_startPoints, _endPoints, _secondsList, _isValid);
        myfile.open(reportPath);
        myfile << stat;
        myfile.close();
    }
}

Generator::Generator(std::shared_ptr<PathFinder> pathFinder)
{
    srand(time(NULL));
    _pathFinder = std::move(pathFinder);
}

Generator::Generator(std::string scenePath, std::string algorithm, bool trace)
{
    srand(time(NULL));

    std::shared_ptr<SceneDescription> sceneDescription;

    std::shared_ptr<Collider> collider;

    std::shared_ptr<SceneWrapper> sceneWrapper = std::make_shared<SceneWrapper>();
    sceneWrapper->buildFromFile(scenePath);


    if (algorithm == "a_star")
        _pathFinder = std::make_shared<AStarPathFinder>(sceneWrapper, trace, 30, 10, 5000, 25, 1, 0, 0, 2);
    else if (algorithm == "ordered_a_star")
        _pathFinder = std::make_shared<OrderedAStarPathFinder>(sceneWrapper, trace, 30, 10, 5000, 25, 1, 1, 0, 2);
    else if (algorithm == "tra")
        _pathFinder = std::make_shared<AStarPathFinder>(sceneWrapper,  trace, 3000, 10, 5000, 600, 3, 0, 0, 1);

}

Json::Value Generator::testPathFinding(std::vector<double> start, std::vector<double> end)
{
    _startPoints.emplace_back(start);
    _endPoints.emplace_back(end);

    info_msg("test path finding");
    Json::Value json;

    using namespace std::chrono;

    int errorCode = PathFinder::NO_ERROR;

    auto startTime = high_resolution_clock::now();
    std::vector<std::vector<double>> path = _pathFinder->findPath(start, end, errorCode);

    auto endTime = high_resolution_clock::now();
    double seconds = (double) duration_cast<milliseconds>(endTime - startTime).count() / 1000;
    info_msg("pf took ", seconds, " seconds, error code: ", errorCode);

    _secondsList.emplace_back(seconds);


    bool isPathValid;
    if (errorCode == PathFinder::NO_ERROR) {
        //isPathValid = _pathFinder->checkPath(path) == PathFinder::NO_ERROR;
        isPathValid = true;
        json = _pathFinder->getJSONRepresentation(path);
        if (!isPathValid)
            errorCode = PathFinder::ERROR_COLLISION_IN_CHECKING;
    }
    else {
        isPathValid = false;
        json = _pathFinder->getJSONRepresentation(
            std::vector<std::vector<double>>{
                start, end
            }
        );
    }

    _isValid.emplace_back(isPathValid);
//    if (!isPathValid) {
//        err_msg("path is not valid");
//        SceneWrapper::dispState(start,"start state");
//        SceneWrapper::dispState(start,"end state");
//    }
    _errorCodes.emplace_back(errorCode);

    return json;
}

void Generator::showErrorPaths()
{
    bool flgError = false;
    for (bool isValid:_isValid)
        if (!isValid) flgError = true;

    if (flgError) {
        err_msg("non valid paths:");

        for (unsigned int i = 0; i < _startPoints.size(); i++) {
            if (!_isValid.at(i)) {
                assert(_errorCodes.at(i) != PathFinder::NO_ERROR);
                SceneWrapper::errState(_startPoints.at(i), " startPoint");
                SceneWrapper::errState(_endPoints.at(i), " endPoint");
                err_msg(" ", PathFinder::errorLabels[_errorCodes.at(i)]);
            }
        }
    }
    else {
        info_msg("all paths are valid");
    }
}
