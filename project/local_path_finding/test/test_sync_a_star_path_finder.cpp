

#include <scene_wrapper.h>
#include <traces.h>
#include <solid_sync_collider.h>
#include <path_finder.h>
#include <sync_tra_star_path_finder.h>


int main()
{
    srand(time(NULL));

    std::shared_ptr<SceneWrapper> sceneWrapper = std::make_shared<SceneWrapper>();
    sceneWrapper->buildFromFile("../../../config/murdf/4robots.json");

    std::shared_ptr<PathFinder>
        pathFinder = std::make_shared<SyncAStarPathFinder>(sceneWrapper, false, 100, 10, 1000, 25, 1, 0, 0, 2);

    std::vector<double> start
        {-2.372, -2.251, 1.977, 0.031, 1.885, 5.093, -2.043, -0.717, -0.893, 0.307, 0.687, -0.148, 0.723, 0.667, -1.421,
         -2.498, 1.934, -4.705, -2.144, -2.477, 1.529, 0.919, 1.333, 2.003};
    std::vector<double> end
        {0.262, -3.238, 1.314, 2.603, -0.827, -3.604, -1.641, -0.440, 1.958, 1.606, 1.474, -4.645, -2.421, -0.583,
         0.134, -0.834, 2.049, -4.375, -2.353, -2.529, 0.148, -0.707, 0.145, -2.702};

    int errorCode = -1;
    std::vector<std::vector<double>> path = pathFinder->findPath(start, end, errorCode);
    pathFinder->showPath(path);
    assert (errorCode == PathFinder::NO_ERROR);

    return 0;
}
