#pragma once


#pragma once

#include <vector>
#include <string>
#include <memory>
#include <Eigen/Dense>


struct Actuator
{
    double maxAcceleration;
    double maxVelocity;
    double maxAngle;
    double minAngle;
    unsigned long jointNum;
    std::string prevLinkName;
    Eigen::Vector3d axis;

    Actuator(double maxAcceleration,
             double maxVelocity,
             double maxAngle,
             double minAngle,
             unsigned long jointNum,
             std::string prevLinkName,
             Eigen::Vector3d axis);
    std::string toString() const;
};

struct Link
{
    Link(const std::string model_path,
         std::shared_ptr<Eigen::Matrix4d> transformMatrix,
         std::string name,
         Eigen::Matrix3d inertia,
         std::vector<double> massCenter
    );

    std::string model_path;
    std::string name;
    std::shared_ptr<Eigen::Matrix4d> transformMatrix;
    std::string toString() const;

    Eigen::Matrix3d inertia;
    std::vector<double> massCenter;


};

class SceneDescription
{

public:

    static void dispActuators(std::vector<std::shared_ptr<Actuator>> actuators);
    static void dispLinks(std::vector<std::shared_ptr<Link>> links);
    static std::vector<double> getPosition(Eigen::Matrix4d);

    void loadFromFile(std::string path);

    virtual ~SceneDescription() = default;
    SceneDescription();

    bool isStateEnabled(std::vector<double> state);

    const std::vector<double> getTranslation() const;
    const std::vector<double> getRotation() const;
    const std::vector<double> getScale() const;
    const std::vector<double> getPose() const;

    void setTranslation(const std::vector<double> &translation);
    void setRotation(const std::vector<double> &rotation);
    void setScale(const std::vector<double> &scale);
    void setPose(const std::vector<double> &pose);

    const std::vector<std::string> getModelPaths() const;

    const std::shared_ptr<Eigen::Matrix4d> &getStartMatrix() const;

    unsigned long getActuatorCnt() const;
    unsigned long getLinkCnt() const;

    const std::vector<std::shared_ptr<Actuator>> getActuators() const;
    const std::vector<std::shared_ptr<Link>> getLinks() const;

    const std::string getPath() const;

    std::vector<Eigen::Matrix4d> getTrasformMatrixies(std::vector<double> state);

    std::vector<double> getEndEffectorPos(std::vector<double> state);

    std::vector<double> getFullPosition(std::shared_ptr<Eigen::Matrix4d> matrix);
    std::vector<double> getAllLinkPositions(std::vector<double> state);
    std::vector<double> getAllLinkMassCenterPositions(std::vector<double> state);
    std::vector<Eigen::Matrix3d> getInertias();

    virtual std::vector<Eigen::Vector3d> getAxes(std::vector<double> state) = 0;

protected:

    virtual void _loadFromFile(std::string path) = 0;
    // list of transform matrices for all links
    virtual std::vector<Eigen::Matrix4d> _getTrasformMatrixies(std::vector<double> state) = 0;
    virtual Eigen::Matrix4d _getLastTrasformMatrix(std::vector<double> state) = 0;
    virtual std::vector<double> _getMassCenters() = 0;

    virtual std::vector<Eigen::Matrix3d> _getInertias() = 0;

    void _beforeLoadFromFile();
    void _afterLoadFromFile();

    std::vector<std::shared_ptr<Actuator>> _actuators;
    std::vector<std::shared_ptr<Link>> _links;
    std::vector<std::shared_ptr<Link>> _fixedLinks;

    static Eigen::Matrix4d _getTranslationMatrix(std::vector<double> &offset);
    static Eigen::Matrix4d _getTranslationMatrix(double x, double y, double z);
    Eigen::Matrix4d _getScaleMatrix(double r, double p, double y);
    Eigen::Matrix4d _getScaleMatrix(std::vector<double> &offset);
    Eigen::Matrix4d _getRotationMatrix(std::vector<double> &rotation);
    Eigen::Matrix4d _getRotationMatrix(double r, double p, double y);

    std::string _path;

private:

    // translation: x,y,z; rotation: r,p,y; scale: x,y,z
    std::vector<double> _pose;

    std::shared_ptr<Eigen::Matrix4d> _startMatrix;

    void _generateStartMatrix();


};