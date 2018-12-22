#include "scene_description.h"

#include <traces.h>

#include <utility>


void SceneDescription::_beforeLoadFromFile()
{
    _actuators.clear();
    _links.clear();
}

void SceneDescription::_afterLoadFromFile()
{

}

bool SceneDescription::isStateEnabled(std::vector<double> state)
{

    //info_msg(state.size(), " ", _actuators.size());

    assert(state.size() == _actuators.size());

    // info_msg("scene description: is state enabled");

    for (unsigned int i = 0; i < state.size(); i++) {
        double v = state.at(i);
        double max = _actuators.at(i)->maxAngle;
        double min = _actuators.at(i)->minAngle;
        if (v - min < -0.00001 || v - max > 0.00001) {
            info_msg(i, " ", min, " ", max, " ", v);
            info_msg(i, " ", v - min, " ", v - max);
            return false;
        }
    }

    return true;
}

Eigen::Matrix4d SceneDescription::_getTranslationMatrix(double x, double y, double z)
{
    Eigen::Matrix4d m;
    m << 1, 0, 0, x,
        0, 1, 0, y,
        0, 0, 1, z,
        0, 0, 0, 1;
    return m;
}

Eigen::Matrix4d SceneDescription::_getTranslationMatrix(std::vector<double> &translation)
{
    return _getTranslationMatrix(
        translation.at(0),
        translation.at(1),
        translation.at(2));
}

Eigen::Matrix4d SceneDescription::_getRotationMatrix(double r, double p, double y)
{
    Eigen::Matrix4d m;
    double c1 = cos(r);
    double s1 = sin(r);
    double c2 = cos(p);
    double s2 = sin(p);
    double c3 = cos(y);
    double s3 = sin(y);
    m << c1 * c2, c1 * s2 * s3 - c3 * s1, s1 * s3 + c1 * c3 * s2, 0,
        c2 * s1, c1 * c3 + s1 * s2 * s3, c3 * s1 * s2 - c1 * s3, 0,
        -s2, c2 * s3, c2 * c3, 0,
        0, 0, 0, 1;
    return m;
}

Eigen::Matrix4d SceneDescription::_getRotationMatrix(std::vector<double> &rotation)
{
    return _getRotationMatrix(
        rotation.at(0),
        rotation.at(1),
        rotation.at(2));
}

Eigen::Matrix4d SceneDescription::_getScaleMatrix(double x, double y, double z)
{
    Eigen::Matrix4d m;
    m << x, 0, 0, 0,
        0, y, 0, 0,
        0, 0, z, 0,
        0, 0, 0, 1;
    return m;
}

Eigen::Matrix4d SceneDescription::_getScaleMatrix(std::vector<double> &scale)
{
    return _getScaleMatrix(
        scale.at(0),
        scale.at(1),
        scale.at(2));
}

const std::vector<std::string> SceneDescription::getModelPaths() const
{
    std::vector<std::string> modelPaths;
    for (const auto &link:_links)
        modelPaths.emplace_back(link->model_path);
    return modelPaths;
}

const std::shared_ptr<Eigen::Matrix4d> &SceneDescription::getStartMatrix() const
{
    return _startMatrix;
}

unsigned long SceneDescription::getActuatorCnt() const
{
    return _actuators.size();
}
unsigned long SceneDescription::getLinkCnt() const
{
    return _links.size();
}

Actuator::Actuator(double maxAcceleration,
                   double maxVelocity,
                   double maxAngle,
                   double minAngle,
                   unsigned long jointNum,
                   std::string prevLinkName)
    : maxAcceleration(maxAcceleration), maxVelocity(maxVelocity), maxAngle(maxAngle), minAngle(minAngle), jointNum(
    jointNum), prevLinkName(std::move(prevLinkName))
{}

std::string Actuator::toString() const
{
    char buf[256];
    sprintf(buf, "maxAcceleration=%.1f ", maxAcceleration);
    std::string msg = buf;
    sprintf(buf, "maxVelocity=%.1f ", maxVelocity);
    msg += buf;
    sprintf(buf, "maxAngle=%.3f ", maxAngle);
    msg += buf;
    sprintf(buf, "minAngle=%.1f ", minAngle);
    msg += buf;
    sprintf(buf, "jointNum=%lu ", jointNum);
    msg += buf;
    return msg + " prevLinkName=" + prevLinkName;
}

void SceneDescription::setTranslation(const std::vector<double> &translation)
{
    assert(translation.size() == 3);
    for (unsigned int i = 0; i < 3; i++) {
        _pose.at(i) = translation.at(i);
    }
}

void SceneDescription::setRotation(const std::vector<double> &rotation)
{
    assert(rotation.size() == 3);
    for (unsigned int i = 0; i < 3; i++) {
        _pose.at(i + 3) = rotation.at(i);
    }

}
void SceneDescription::setScale(const std::vector<double> &scale)
{
    assert(scale.size() == 3);
    for (unsigned int i = 0; i < 3; i++) {
        _pose.at(i + 6) = scale.at(i);
    }
}

std::vector<Eigen::Matrix4d> SceneDescription::getTrasformMatrices(std::vector<double> state)
{
    return _getTrasformMatrixies(std::move(state));
}

const std::vector<std::shared_ptr<Actuator>> SceneDescription::getActuators() const
{
    return _actuators;
}

const std::vector<std::shared_ptr<Link>> SceneDescription::getLinks() const
{
    return _links;
}

std::string Link::toString() const
{
    return "model_path=" + model_path + "\nname=" + name;
}

Link::Link(const std::string model_path,
           std::shared_ptr<Eigen::Matrix4d> transformMatrix,
           std::string name
)
    : model_path(model_path),
      transformMatrix(std::move(transformMatrix)),
      name(name)
{}

void SceneDescription::dispActuators(std::vector<std::shared_ptr<Actuator>> actuators)
{
    info_msg("disp actuators");
    for (const std::shared_ptr<Actuator> &actuator:actuators) {
        info_msg(actuator->toString());
    }
}

void SceneDescription::dispLinks(std::vector<std::shared_ptr<Link>> links)
{
    info_msg("disp links");
    for (const std::shared_ptr<Link> &link:links) {
        info_msg(link->toString());
    }
}

const std::vector<double> SceneDescription::getTranslation() const
{
    return std::vector<double>(_pose.begin(), _pose.begin() + 3);
}
const std::vector<double> SceneDescription::getRotation() const
{
    return std::vector<double>(_pose.begin() + 3, _pose.begin() + 6);
}
const std::vector<double> SceneDescription::getScale() const
{
    return std::vector<double>(_pose.begin() + 6, _pose.begin() + 9);
};

SceneDescription::SceneDescription()
{
    _pose = std::vector<double>{0, 0, 0, 0, 0, 0, 1, 1, 1};
}

const std::string SceneDescription::getPath() const
{
    return _path;
}

void SceneDescription::loadFromFile(std::string path)
{
    _beforeLoadFromFile();
    _loadFromFile(std::move(path));
    _afterLoadFromFile();
}

const std::vector<double> SceneDescription::getPose() const
{
    return _pose;
}

void SceneDescription::setPose(const std::vector<double> &pose)
{
    assert(pose.size() == 9);

    _pose = pose;

    _generateStartMatrix();

}

void SceneDescription::_generateStartMatrix()
{
    std::vector<double> translation(_pose.begin(), _pose.begin() + 3);
    std::vector<double> rotation(_pose.begin() + 3, _pose.begin() + 6);
    std::vector<double> scale(_pose.begin() + 6, _pose.begin() + 9);
    _startMatrix = std::make_shared<Eigen::Matrix4d>(
        _getRotationMatrix(rotation) *
            _getTranslationMatrix(translation) *
            _getScaleMatrix(scale)
    );
}
std::vector<Eigen::Matrix4d> SceneDescription::getTrasformMatrixies(std::vector<double> state)
{
    return _getTrasformMatrixies(state);
}

std::vector<double> SceneDescription::getEndEffectorPos(std::vector<double> state)
{
    //  info_msg("SceneDescription::getEndEffectorPos");
    auto tf = _getLastTrasformMatrix(state);
    return getPosition(tf);
}

std::vector<double> SceneDescription::getAllPoses(std::vector<double> state)
{
//    info_msg("SceneWrapper::getAllPoses");
//    std::string msg = "";
//    char buf[256];
//
//    for (double & s:state){
//        sprintf(buf,"%.3f ",s);
//        msg+=buf;
//    }
//    info_msg("incomig state: "+msg);

    auto tfs = _getTrasformMatrixies(state);
  //  info_msg("SceneDescription::getEndEffectorPos ", tfs.size());
    std::vector<double> allPoses;
    for (auto &tf:tfs) {
        auto pos = getPosition(tf);
   //     info_msg(pos.at(0), " ", pos.at(1), " ", pos.at(2));
        allPoses.insert(allPoses.end(), pos.begin(), pos.end());
    }
//    info_msg(allPoses.size());
//    for (double pos:allPoses){
//        info_msg(pos);
//    }
    return allPoses;
}

// get position by transfer function
std::vector<double> SceneDescription::getPosition(Eigen::Matrix4d tf)
{

    std::vector<double> pos{
        tf(0, 3), // STATE_POS_X
        tf(1, 3), // STATE_POS_Y
        tf(2, 3), // STATE_POS_Z
    };
    return pos;
}

std::vector<double> SceneDescription::getFullPosition(std::shared_ptr<Eigen::Matrix4d> matrix)
{
    Eigen::Matrix4d tf = *matrix;
    double tr = tf.trace();

    double w = 1.0 / 2 * sqrt(tr);
    double x = 1.0 / (4 * w) * (tf(2, 1) - tf(1, 2));
    double y = 1.0 / (4 * w) * (tf(0, 2) - tf(2, 0));
    double z = 1.0 / (4 * w) * (tf(1, 0) - tf(0, 1));
    std::vector<double> pos{
        tf(0, 3), // STATE_POS_X
        tf(1, 3), // STATE_POS_Y
        tf(2, 3), // STATE_POS_Z
        w,        // STATE_ORIENT_W
        x,        // STATE_ORIENT_X
        y,        // STATE_ORIENT_Y
        z         // STATE_ORIENT_Z
    };
    return pos;
}


