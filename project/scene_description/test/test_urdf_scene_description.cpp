#include <scene_description.h>
#include <urdf_scene_description.h>
#include <traces.h>

int main()
{
    std::shared_ptr<SceneDescription> sc = std::make_shared<URDFSceneDescription>();
    sc->loadFromFile("../../../config/urdf/kuka_six.urdf");

    info_msg(sc->getActuatorCnt());
    info_msg(sc->getLinkCnt());


    for (auto path:sc->getModelPaths()){
        info_msg(path);
    }
    return 0;
}