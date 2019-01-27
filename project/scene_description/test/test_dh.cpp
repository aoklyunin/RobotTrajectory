


#include <scene_description.h>
#include <urdf_scene_description.h>
#include <dh_scene_description.h>
#include <traces.h>


const int TEST_CNT = 1;

int main(){
    std::shared_ptr<SceneDescription> urdf = std::make_shared<URDFSceneDescription>();
    urdf->loadFromFile("../../../config/urdf/ur10.urdf");

    std::shared_ptr<SceneDescription> dh = std::make_shared<DHSceneDescription>();
    dh->loadFromFile("../../../config/dh/ur10.json");

    for (int i=0;i<TEST_CNT;i++){
        auto state = urdf->getRandomState();
        auto urdfPoses = urdf->getAllLinkPositions(state);
        auto dhPoses = dh->getAllLinkPositions(state);
        info_msg(urdfPoses.size()," ",dhPoses.size());
    }
    return 0;
}