

#include <scene_wrapper.h>
#include <traces.h>
#include <solid_collider.h>

#include <chrono>
#include <path_finder.h>
#include <global_path_finder.h>

std::shared_ptr<SceneWrapper> sceneWrapper;

std::shared_ptr<PathFinder> pathFinder;


void testPath(std::vector<double> &start, std::vector<double> &end)
{
    using namespace std::chrono;

    auto startTime = high_resolution_clock::now();

    info_msg("test begin");
    int errorCode = -1;
    std::vector<std::vector<double>> path = pathFinder->findPath(start, end, errorCode);
    PathFinder::showPath(path);

    assert(!path.empty());

    assert(errorCode == PathFinder::NO_ERROR);

    assert(pathFinder->checkPath(path) == PathFinder::NO_ERROR);

    assert(SceneWrapper::getStateDistance(start, path.front()) < 0.0001);
    assert(SceneWrapper::getStateDistance(end, path.back()) < 0.0001);
    assert(!path.empty());

    auto endTime = high_resolution_clock::now();
    auto seconds = duration_cast<milliseconds>(endTime - startTime).count() / 1000;
    info_msg(seconds, " seconds");

    assert (errorCode == PathFinder::NO_ERROR);

}

void test1()
{
    info_msg("test 1");
    std::vector<double> start
        {-2.967, -0.855, 0.314, -1.937, -1.676, -3.665};
    std::vector<double> end
        {1.187, -0.035, -1.131, 0.000, 0.419, 3.665};
    testPath(start, end);
}
void test2()
{
    info_msg("test 2");
    std::vector<double> start
        {1.397 ,-3.276 ,0.165 ,1.935 ,-0.102 ,1.925};
    std::vector<double> end
        {0.670 ,0.560 ,1.761 ,-0.916 ,-0.482 ,2.184};
    testPath(start, end);
}

void test3()
{
    info_msg("test 3");
    std::vector<double> start
        {-2.548 ,-3.182 ,-1.391 ,2.705 ,-1.300 ,-1.551};
    std::vector<double> end
        {-1.051 ,0.319 ,-0.334 ,0.203 ,-1.719 ,5.273};
    testPath(start, end);
}


int main()
{
    info_msg("test speed tra finder");

    std::shared_ptr<SceneWrapper> sceneWrapper = std::make_shared<SceneWrapper>();
    sceneWrapper->buildFromFile("../../../config/murdf/demo_scene2.json");

    pathFinder = std::make_shared<GlobalPathFinder>(sceneWrapper, true);

  //  test1();
    test2();
    test3();

    info_msg("complete");
    return 0;
}
