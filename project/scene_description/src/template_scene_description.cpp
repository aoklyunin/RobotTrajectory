#include "template_scene_description.h"


/*
 * need to fill:
 *
 *
 *
 *
 */
void TemplateSceneDescription::_init(std::string &&description_file_path)
{
    /*
     *  you need to populate:
     *   _joints - this is the all joints in the system
     *   _actuatorIndexes - this indexes of actuators(variable)
     *   _jointCnt - count of all joints
     *   _actuatorCnt - count of actuators
     *   maxAcceleraions - maximum accelereations of actuators
     *   maxVelocities - maximum velocities of actuators
     *   linkmatrices - this is relative matrices of all joints(objects)
     *   jointLimit - it's a two-dimentioned list oof limits. First demention
     *   depends on count of actuators, second dimention is two: first is
     *   lower current actuator limit, second - heigher
     */


}

Eigen::Matrix4d TemplateSceneDescription::getTransformMatrix(const std::vector<double> &joints)
{
    Eigen::Matrix4d transformMatrix = Eigen::Matrix<double, 4, 4>::Identity();
    /*
     *  here you need to write code, what returns transform matrix of end-effector
     */
    return transformMatrix;
}
