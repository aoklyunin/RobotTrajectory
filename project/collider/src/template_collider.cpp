#include "template_collider.h"
#include <stl_reader.h>


void TemplateCollider::init(std::vector<std::vector<std::string>> groupedModelPaths)
{
    /*
     *  here ypu need process list of models, starting at in directory with prefix path
     */
}

void TemplateCollider::paint(std::vector<Eigen::Matrix4d> matrices)
{
    /*
     *  you need paint your objects with OpenGL, using transform matrices
     *
     */

}


bool isCollided(std::vector<Eigen::Matrix4d> matrices){
    /*
     * check collision by matrices
     */
}

bool isCollided(std::vector<Eigen::Matrix4d> matrices, int robotNum){
    /*
     * check collision only for one robot
     * matrices in array are only for this robot
     */
}
