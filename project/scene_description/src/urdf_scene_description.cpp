#include "urdf_scene_description.h"

#include <traces.h>
#include <fstream>
#include <set>


void URDFSceneDescription::_addURDFJoints(std::vector<std::shared_ptr<urdf::Joint>> jointList,
                                          std::map<std::string, urdf::LinkSharedPtr> linkMap) {

    // info_msg("addURDFJoints size: ", jointList.size());

    std::set<std::string> findedLinkNames;

    for (unsigned int i = 0; i < jointList.size(); i++) {
        auto j = jointList.at(i);
        Eigen::Matrix4d selfTransform = _getMatrixFromPose(j->parent_to_joint_origin_transform);
        urdf::Vector3 axis = j->axis;

        bool isFixed = j->type == urdf::Joint::FIXED;

        if (!isFixed) {
            _actuators.emplace_back(std::make_shared<Actuator>(j->limits->effort,
                                                               j->limits->velocity,
                                                               j->limits->upper,
                                                               j->limits->lower,
                                                               i,
                                                               j->child_link_name
            ));
        }
        findedLinkNames.insert(j->child_link_name);
        auto link = linkMap.at(j->child_link_name);

        assert(link->collision);
        Eigen::Matrix4d linkTransform = _getMatrixFromPose(link->collision->origin);
        auto scale = std::dynamic_pointer_cast<const urdf::Mesh>(link->collision->geometry)->scale;
        //  info_msg("loop scale ",scale.x," ",scale.y," ",scale.z);
        //info_msg(_getScaleMatrix(scale.x,scale.y,scale.z));
        linkTransform = linkTransform * _getScaleMatrix(scale.x, scale.y, scale.z);

        _links.emplace_back(std::make_shared<Link>(
                std::dynamic_pointer_cast<const urdf::Mesh>(link->collision->geometry)->filename,
                std::make_shared<Eigen::Matrix4d>(linkTransform),
                link->name));

        _joints.emplace_back(selfTransform, linkTransform, axis, isFixed, 0.0);
    }

    // info_msg("urdf add URDF JOINTS");

    for (auto &entry : linkMap) {
        if (findedLinkNames.find(entry.first) == findedLinkNames.end()) {
            if (entry.second->collision && entry.second->collision->geometry) {
                Eigen::Matrix4d linkTransform = _getMatrixFromPose(entry.second->collision->origin);
                auto scale = std::dynamic_pointer_cast<const urdf::Mesh>(entry.second->collision->geometry)->scale;
                // info_msg("single scale ",scale.x," ",scale.y," ",scale.z);
                //   info_msg(_getScaleMatrix(scale.x,scale.y,scale.z));
                linkTransform = linkTransform * _getScaleMatrix(scale.x, scale.y, scale.z);

                auto link = std::make_shared<Link>(
                        std::dynamic_pointer_cast<const urdf::Mesh>(entry.second->collision->geometry)->filename,
                        std::make_shared<Eigen::Matrix4d>(linkTransform),
                        entry.second->name
                );
                _links.emplace_back(link);
                _fixedLinks.emplace_back(link);
                // info_msg(entry.first);
            }
        }
    }
}

std::vector<Eigen::Matrix4d> URDFSceneDescription::_getTrasformMatrixies(std::vector<double> state) {
    // info_msg("urdf scene description: get Transform matrices local");

    Eigen::Matrix4d transformMatrix = *getStartMatrix();

    //  info_msg("get transform matrices local");
//    info_msg(transformMatrix);

    std::vector<Eigen::Matrix4d> matrices;

    unsigned int jointPos = 0;

    //info_msg(_joints.size());
    for (unsigned long i = 0; i < _joints.size(); i++) {
        auto jd = _joints.at(i);
        if (!jd.isFixed) {
            jd.jointAngle = state.at(jointPos);
            jointPos++;
        }
        Eigen::Matrix4d tf = transformMatrix * jd.getTransformMatrix() * jd.linkTransform;

        matrices.emplace_back(tf);

        transformMatrix = transformMatrix * jd.getTransformMatrix();
    }

//    info_msg("after main loop");

    for (const auto &link:_fixedLinks) {
        // info_msg("fixed link");
        matrices.emplace_back(transformMatrix * (*link->transformMatrix));
    }

    return matrices;

}

Eigen::Matrix4d URDFSceneDescription::_getMatrixFromPose(urdf::Pose pose) {
    Eigen::Matrix4d m;
    urdf::Vector3 p = pose.position;
    urdf::Rotation r = pose.rotation;
    double a = r.w;
    double b = r.x;
    double c = r.y;
    double d = r.z;
    m << a * a + b * b - c * c - d * c, 2 * b * c - 2 * a * d, 2 * b * d + 2 * a * c, p.x,
            2 * b * c + 2 * a * d, a * a - b * b + c * c - d * d, 2 * c * d - 2 * a * b, p.y,
            2 * b * d - 2 * a * c, 2 * c * d + 2 * a * b, a * a - b * b - c * c + d * d, p.z,
            0, 0, 0, 1;
    return m;
}


void URDFSceneDescription::_loadFromFile(std::string path) {
    // info_msg("urdf scene description: load form file");
    _path = path;
    urdf::Model model;
    if (!model.initFile(path)) {
        info_msg("error reading urdf");
        return;
    }

    std::map<std::shared_ptr<urdf::Joint>, std::shared_ptr<urdf::Joint>> nextJoints;
    std::set<std::shared_ptr<urdf::Joint>> jointSet;
    for (auto joint:model.joints_) {
        jointSet.insert(joint.second);
    }

    for (auto joint1 : model.joints_)
        for (auto joint2: model.joints_) {
            auto p1 = joint1.second;
            auto p2 = joint2.second;
            if (p1 && p2 && p1 != p2 && p1->child_link_name == p2->parent_link_name) {
                jointSet.erase(p2);
                nextJoints.insert(std::pair<std::shared_ptr<urdf::Joint>, std::shared_ptr<urdf::Joint>>(p1, p2));
            }
        }

    assert(jointSet.size() == 1);
    auto currentJoint = *jointSet.begin();

    std::vector<std::shared_ptr<urdf::Joint>> orderedJointList;

    while (nextJoints.find(currentJoint) != nextJoints.end()) {
        orderedJointList.push_back(currentJoint);
        currentJoint = nextJoints.at(currentJoint);
    }

    // _jointCnt = orderedJointList.size();

//    if (orderedJointList.empty()) {
//        boost::shared_ptr<urdf::Joint> firstJoint = *jointSet.begin();
//        auto link = model.links_.at(firstJoint->child_link_name);
//        _links.emplace_back(std::make_shared<Link>(
//            boost::dynamic_pointer_cast<const urdf::Mesh>(link->collision->geometry)->filename, nullptr
//        ));
//    }
//    else {
    _addURDFJoints(orderedJointList, model.links_);
//    }


    // _groupedModelPaths.push_back(_modelPaths);
    _afterLoadFromFile();
}

Eigen::Matrix4d URDFSceneDescription::_getLastTrasformMatrix(std::vector<double> state) {

    if (_joints.size() == 0)
        return Eigen::Matrix4d::Identity();


    Eigen::Matrix4d transformMatrix = *getStartMatrix();

    unsigned int jointPos = 0;

    //info_msg(_joints.size());
    for (unsigned long i = 0; i < _joints.size(); i++) {
        auto jd = _joints.at(i);
        if (!jd.isFixed) {
            jd.jointAngle = state.at(jointPos);
            jointPos++;
        }
        Eigen::Matrix4d tf = transformMatrix * jd.getTransformMatrix() * jd.linkTransform;

        transformMatrix = transformMatrix * jd.getTransformMatrix();
    }

    return transformMatrix;

}

Eigen::Matrix4d URDFJoint::getTransformMatrix() {
    Eigen::Matrix4d rotM;
    double theta = jointAngle;
    double x = axis.x;
    double y = axis.y;
    double z = axis.z;

    rotM << cos(theta) + (1 - cos(theta)) * x * x, (1 - cos(theta)) * x * y - sin(theta) * z, (1 - cos(theta)) * x * z
                                                                                              + sin(theta) * y, 0,
            (1 - cos(theta)) * y * x + sin(theta) * z, cos(theta) + (1 - cos(theta)) * y * y, (1 - cos(theta)) * y * z
                                                                                              - sin(theta) * x, 0,
            (1 - cos(theta)) * z * x - sin(theta) * y, (1 - cos(theta)) * z * y + (sin(theta)) * x, cos(theta)
                                                                                                    + (1 - cos(theta)) *
                                                                                                      z * z, 0,
            0, 0, 0, 1;

    return selfTransform * rotM;
}
