#include "solid_collider.h"
#include <stl_reader.h>


void SolidCollider::init(std::vector<std::vector<std::string>> groupedModelPaths)
{

   // info_msg("sc: init ",groupedModelPaths.size());
    _links.clear();
    _robotIndexes.clear();
    singleRobotColliders.clear();

    _isSingleRobot = groupedModelPaths.size() == 1;
    _groupedLinks.clear();

    int pos = 0;


    //info_msg("solid collider init ", groupedModelPaths.size());
    for (std::vector<std::string> modelPaths:groupedModelPaths) {
//        for(auto p:modelPaths){
//            info_msg(p);
//        }
        std::vector<std::shared_ptr<Solid3Object>> localLinks;
        if (!_isSingleRobot) {
            std::vector<unsigned long> range{
                static_cast<unsigned long>(pos), modelPaths.size() + pos - 1
            };
            _robotIndexes.emplace_back(range);
            pos += modelPaths.size();
        }
        for (const std::string &path: modelPaths) {
            //info_msg(path);
            std::vector<float> points = read_stl(path);
            std::shared_ptr<Solid3Object> ptr = std::make_shared<Solid3Object>(new StlSolid3Object(points),modelPaths.size()!=1);
            _links.emplace_back(ptr);
            localLinks.emplace_back(ptr);
        }
        _groupedLinks.emplace_back(std::move(localLinks));
    }

    _scene = DT_CreateScene();
    for (std::shared_ptr<Solid3Object> &obj:_links) {
        DT_AddObject(_scene, obj->getHandle());
    }
    if (!_isSingleRobot) {
        //info_msg(groupedModelPaths.size(), " size _groupedModelPaths");
        for (const auto &modelPaths:groupedModelPaths) {
            std::vector<std::vector<std::string>> tmpPath{modelPaths};
            std::shared_ptr<Collider> singleRobotCollider = std::make_shared<SolidCollider>();
            singleRobotCollider->init(tmpPath);
            singleRobotColliders.emplace_back(singleRobotCollider);
        }
    }

}

// convert Eigen Matrix to double array
double *SolidCollider::_eigenToDouble(Eigen::Matrix4d m)
{
    double *matrix = new double[16];
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++)
            matrix[4 * i + j] = m(i, j);
    }
    return matrix;

};

void SolidCollider::setTransformMatrices(std::vector<Eigen::Matrix4d> matrices)
{

    // info_msg("m begin");
    // info_msg(_sceneDescriptions.size()," ",matrices.size());
    // info_msg(_links.size(), " ", matrices.size());
    assert(_links.size() == matrices.size());
    const unsigned long itCnt = _links.size();
    for (unsigned long i = 0; i < itCnt; i++) {
        //info_msg(matrices.at(i));
        double *position = _eigenToDouble(matrices.at(i).transpose());
        DT_SetMatrixd(_links.at(i)->getHandle(), position);
        delete[] position;
    }
    //info_msg("m end");
}

void SolidCollider::paint(std::vector<Eigen::Matrix4d> matrices,bool onlyRobot)
{
//    info_msg(" SolidCollider::paint ",onlyRobot);

    setTransformMatrices(matrices);
    for (int i = 0; i < _links.size(); i++) {
        //glColor3f(1.0f, 1.0f / _links.size() * i, 1.0f / _links.size() * i);
        _links.at(i)->paint(onlyRobot);
    }

//    glDisable(GL_DEPTH_TEST);
//    glDisable(GL_LIGHTING);
//
//    MT_Point3 min;
//    MT_Point3 max;
//    for (std::shared_ptr<Solid3Object> &obj:_links) {
//        DT_GetBBox(obj->getHandle(), min, max);
//        _displayBox(min, max);
//    }
//
//    glEnable(GL_LIGHTING);
//    glEnable(GL_DEPTH_TEST);

}

std::vector<double> SolidCollider::getBoxChords(unsigned long objNum, std::vector<Eigen::Matrix4d> matrices)
{
    setTransformMatrices(matrices);
    //info_msg(objNum," ", _groupedLinks.size());
    assert(objNum < _groupedLinks.size());

    // info_msg("get box chords");

    //  info_msg(objNum, " ", _groupedLinks.size());

    MT_Point3 min;
    MT_Point3 max;
    DT_GetBBox(_groupedLinks.at(objNum).front()->getHandle(), min, max);
    std::vector<double> startMinMax = {min.x(), min.y(), min.z(), max.x(), max.y(), max.z()};

    //  info_msg("links size ", _links.size());

    // info_msg("grouped links size ", _groupedLinks.at(objNum).size());
    for (auto &link:_groupedLinks.at(objNum)) {
        //info_msg("grouped links loop");
        DT_GetBBox(link->getHandle(), min, max);
        std::vector<double> minMax = {min.x(), min.y(), min.z(), max.x(), max.y(), max.z()};

        for (unsigned int i = 0; i < 3; i++) {
            if (startMinMax.at(i) > minMax.at(i))
                startMinMax.at(i) = minMax.at(i);
        }

        for (unsigned int i = 3; i < 6; i++) {
            if (startMinMax.at(i) < minMax.at(i))
                startMinMax.at(i) = minMax.at(i);
        }
    }
    return startMinMax;
}

void SolidCollider::_displayBox(const MT_Point3 &min, const MT_Point3 &max)
{
    glColor3f(0.0f, 1.0f, 1.0f);
    glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
    glBegin(GL_QUAD_STRIP);
    glVertex3d(min[0], min[1], min[2]);
    glVertex3d(min[0], min[1], max[2]);
    glVertex3d(max[0], min[1], min[2]);
    glVertex3d(max[0], min[1], max[2]);
    glVertex3d(max[0], max[1], min[2]);
    glVertex3d(max[0], max[1], max[2]);
    glVertex3d(min[0], max[1], min[2]);
    glVertex3d(min[0], max[1], max[2]);
    glVertex3d(min[0], min[1], min[2]);
    glVertex3d(min[0], min[1], max[2]);
    glEnd();
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
}

bool SolidCollider::isCollided()
{
    MT_Point3 cp1;
    bool flgCollision = false;
    if (_isSingleRobot) {
        for (int i = 0; i < _links.size(); i++)
            for (int j = 0; j < _links.size(); j++)
                if (std::abs(i - j) > 1
                    && DT_GetCommonPoint(_links.at(i)->getHandle(), _links.at(j)->getHandle(), cp1)) {
                    //info_msg("single robot collided: ",i," ",j);
                    flgCollision = true;
                }
    }
    else {
       // info_msg(_links.size());
        for (unsigned long i = 0; i < _links.size(); i++) {
            if (flgCollision)
                break;
            for (unsigned long j = 0; j < _links.size(); j++) {
                if (i == j)
                    continue;

                bool neighbors = false;
                for (std::vector<unsigned long> robotRange:_robotIndexes) {
                   // info_msg(robotRange.at(0), " ", robotRange.at(1));
                    if (i >= robotRange.at(0) &&
                        i <= robotRange.at(1) &&
                        j >= robotRange.at(0) &&
                        j <= robotRange.at(1) &&
                        (i == j + 1 || i == j - 1 || i == j)
                        ) {
                        neighbors = true;
                        break;
                    }
                }
              //  info_msg("i:", i, " j:", j, neighbors ? " neigbors" : " not neigbors");

                if (!neighbors && DT_GetCommonPoint(_links.at(i)->getHandle(), _links.at(j)->getHandle(), cp1)) {
                   // info_msg(i, " ", j);
                  //  info_msg("multi robot collided: ",i," ",j);
                    flgCollision = true;
                    break;
                }
            }
        }
    }

    return flgCollision;
}

SolidCollider::~SolidCollider()
{
    DT_DestroyScene(_scene);
}

bool SolidCollider::isCollided(std::vector<Eigen::Matrix4d> matrices)
{
        //info_msg("is collided");
    setTransformMatrices(std::move(matrices));



    bool ic = isCollided();
    //info_msg(ic);
    return ic;
}

std::vector<Eigen::Matrix4d>

SolidCollider::getSingleRobotmatrixList(std::vector<Eigen::Matrix4d> matrices, int robotNum)
{
    return std::vector<Eigen::Matrix4d>(matrices.begin() + robotNum * 7, matrices.begin() + (robotNum + 1) * 7);
}

bool SolidCollider::isCollided(std::vector<Eigen::Matrix4d> matrices, int robotNum)
{
    //  info_msg("is collided called");
    assert(robotNum >= 0);
    assert(robotNum < singleRobotColliders.size());
    // info_msg(matrices.size());

    std::vector<Eigen::Matrix4d> singleRobotmatrixList = getSingleRobotmatrixList(matrices, robotNum);

    //info_msg(singleRobotmatrixList.size());
    //info_msg(matrices.at(2));
    return singleRobotColliders.at(robotNum)->isCollided(singleRobotmatrixList);
}

std::vector<float> SolidCollider::getPoints(std::vector<Eigen::Matrix4d> matrices)
{
    //info_msg("get points");
    setTransformMatrices(std::move(matrices));

    std::vector<float> pointList;
    for (auto &link:_links) {
        std::vector<float> points = link->getFullPointsList();
//        for(unsigned int i=0;i<points.size()/3;i++) {
//            Eigen::Vector4d pos(
//                points.at(i*3),
//                points.at(i*3+1),
//                points.at(i*3+2),
//                1
//            );
//           // pos = link->
//        }
        pointList.insert(pointList.begin(), points.begin(), points.end());
    }
    return pointList;
}
const std::vector<std::shared_ptr<Solid3Object>> &SolidCollider::getLinks() const
{
    return _links;
}

float SolidCollider::getDistance(std::vector<Eigen::Matrix4d> matrices)
{
    setTransformMatrices(std::move(matrices));

    float minDistance = 1000000;
    if (_isSingleRobot) {
        for (int i = 0; i < _links.size(); i++)
            for (int j = 0; j < _links.size(); j++)
                if (std::abs(i - j) > 1) {
                    MT_Point3 cp1;
                    MT_Point3 cp2;
                    float
                        curDistance = DT_GetClosestPair(_links.at(i)->getHandle(), _links.at(j)->getHandle(), cp1, cp2);
                    if (curDistance < minDistance)
                        minDistance = curDistance;
                }

    }
    else {
        for (long i = 0; i < _links.size(); i++) {
            for (long j = 0; j < _links.size(); j++) {
                if (i == j)
                    continue;

                bool neighbors = false;
                for (std::vector<unsigned long> robotRange:_robotIndexes)
                    if (i >= robotRange.at(0) &&
                        i <= robotRange.at(1) &&
                        j >= robotRange.at(0) &&
                        j <= robotRange.at(1)
                        //  && std::abs(i - j) <= 1
                        ) {
                        neighbors = true;
                        break;
                    }
                if (!neighbors) {
                    MT_Point3 cp1;
                    MT_Point3 cp2;
                    float
                        curDistance = DT_GetClosestPair(_links.at(i)->getHandle(), _links.at(j)->getHandle(), cp1, cp2);
                    if (curDistance < minDistance)
                        minDistance = curDistance;
                }
            }
        }

    }

    assert(minDistance != 100000);
    return minDistance;

}
