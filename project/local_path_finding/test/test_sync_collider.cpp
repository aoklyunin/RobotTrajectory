

#include <scene_wrapper.h>
#include <traces.h>
#include <urdf_scene_description.h>
#include <solid_sync_collider.h>
#include "tra_star_path_finder.h"


int main()
{
    info_msg("sync collider");
    srand(time(NULL));


    std::shared_ptr<SceneWrapper> sceneWrapper = std::make_shared<SceneWrapper>();
    sceneWrapper->buildFromFile("../../../config/murdf/demo_scene.json");


    std::shared_ptr<PathFinder>
        pathFinder = std::make_shared<AStarPathFinder>(sceneWrapper, false, 100, 10, 1000, 25, 1, 1, 0, 2);

    std::vector<double> start{-0.017, 0.618, -0.011, -0.970, -0.106, 1.309};
    std::vector<double> end{1.883, 0.341, 0.232, 1.081, 1.366, 5.234};

    int errorCode = -1;
    std::vector<std::vector<double>> path = pathFinder->findPath(start, end, errorCode);
    pathFinder->showPath(path);
    assert (errorCode = PathFinder::NO_ERROR);

    return 0;
}
