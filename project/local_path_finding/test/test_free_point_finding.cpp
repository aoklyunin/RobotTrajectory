//
// Created by alex on 01.10.18.
//

#include <scene_wrapper.h>
#include <path_finder.h>
#include <solid_collider.h>
#include <local_path_finder.h>
#include <ordered_tra_star_path_finder.h>

std::shared_ptr<SceneWrapper> sceneWrapper;

std::shared_ptr<SceneDescription> sceneDescription;

std::shared_ptr<LocalPathFinder> pathFinder;

std::shared_ptr<Collider> collider;

int main()
{
    srand(time(NULL));

    info_msg("test free point finding");

    std::shared_ptr<SceneWrapper> sceneWrapper = std::make_shared<SceneWrapper>();
    sceneWrapper->buildFromFile("../../../config/murdf/4robots.json");

    pathFinder = std::make_shared<OrderedAStarPathFinder>(
        sceneWrapper, true, 30, 10, 1000, 100, 1, 1, 0, 2
    );

    int nonValidCnt = 0;

    for (unsigned int i = 0; i < 100; i++) {
        auto state = sceneWrapper->getRandomState();
        auto chords = pathFinder->stateToChords(state);
        if (chords.empty()) {
            SceneWrapper::dispState(state, "random state");
            info_msg("NON VALID");
            nonValidCnt++;
        }
        else
            info_msg("VALID");
    }

    info_msg("non valid: ", nonValidCnt);

    return 0;
}