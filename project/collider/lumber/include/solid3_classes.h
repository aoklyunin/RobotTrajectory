#pragma once

#include "traces.h"


#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-variable"
#include <GL/glut.h>
#include <SOLID.h>
#include <MT_Scalar.h>
#include <MT_Point3.h>
#pragma GCC diagnostic pop

#include <vector>
#include <Eigen/Dense>

class Solid3Shape
{
public:
    Solid3Shape(DT_ShapeHandle shape = 0)
        : m_shape(shape)
    {}
    virtual ~Solid3Shape()
    {}

    void setShape(DT_ShapeHandle shape)
    { m_shape = shape; }

    DT_ShapeHandle getShape() const
    { return m_shape; }

    virtual void paint() const = 0;

    virtual std::vector<float> &getFullPointsList() = 0;
    virtual std::vector<double> getMinMax() = 0;
protected:
    DT_ShapeHandle m_shape;
};

class StlSolid3Object: public Solid3Shape
{
public:

    StlSolid3Object(std::vector<float> points)
    {
        _fullPointsList = points;
        triCnt = points.size() / 12;
        //triCnt = 10;
        m_points = new MT_Point3[triCnt * 3];
        for (unsigned int i = 0; i < triCnt; i++) {
            DT_Vector3 point1{points.at(i * 12 + 3), points.at(i * 12 + 4), points.at(i * 12 + 5)};
            DT_Vector3 point2{points.at(i * 12 + 6), points.at(i * 12 + 7), points.at(i * 12 + 8)};
            DT_Vector3 point3{points.at(i * 12 + 9), points.at(i * 12 + 10), points.at(i * 12 + 11)};
            m_points[i * 3].setValue(point1);
            m_points[i * 3 + 1].setValue(point2);
            m_points[i * 3 + 2].setValue(point3);
        }

        m_base = DT_NewVertexBase(m_points, 0);
        setShape(DT_NewComplexShape(m_base));
        for (unsigned int i = 0; i < triCnt; i++) {
            DT_Begin();
            DT_VertexIndex(i * 3);
            DT_VertexIndex(i * 3 + 1);
            DT_VertexIndex(i * 3 + 2);
            DT_End();
        }
        DT_EndComplexShape();
    }

    StlSolid3Object(const char *path)
        : Solid3Shape()
    {
        m_base = DT_NewVertexBase(m_points, 0);
        setShape(DT_NewComplexShape(m_base));


        DT_EndComplexShape();

    }

    virtual void paint() const
    {
        glBegin(GL_TRIANGLES);
        for (DT_Index i = 0; i < triCnt * 3; i++) {
            glVertex3fv(m_points[i]);
        }
        glEnd();
    }

    ~StlSolid3Object()
    {
        DT_DeleteVertexBase(m_base);
        delete[] m_points;
    }

    std::vector<float> &getFullPointsList()
    {
        return _fullPointsList;
    }

    std::vector<double> getMinMax()
    {
        double minX = m_points[0].x();
        double minY = m_points[0].y();
        double minZ = m_points[0].z();
        double maxX = minX;
        double maxY = minY;
        double maxZ = minZ;

        for (DT_Index i = 0; i < triCnt * 3; i++) {
            if (minX > m_points[i].x()) minX = m_points[i].x();
            if (minY > m_points[i].y()) minY = m_points[i].y();
            if (minZ > m_points[i].z()) minZ = m_points[i].z();
            if (maxX < m_points[i].x()) maxX = m_points[i].x();
            if (maxY < m_points[i].y()) maxY = m_points[i].y();
            if (maxZ < m_points[i].z()) maxZ = m_points[i].z();
        }
        return std::vector<double>{minX, minY, minZ, maxX, maxY, maxZ};
    }

private:
    MT_Point3 *m_points;
    unsigned int triCnt;
    DT_VertexBaseHandle m_base;
    std::vector<float> _fullPointsList;
};

class Solid3Object
{
public:
    Solid3Object()
    {}
    Solid3Object(Solid3Shape *shape, bool isRobot, MT_Scalar margin = 0.0f)
        : m_shape(shape),
          m_object(DT_CreateObject(this, shape->getShape())),
          isRobot(isRobot)
    {
        DT_SetMargin(m_object, margin);
    }

    Solid3Object(const Solid3Object &) = delete;
    Solid3Object &operator=(const Solid3Object &) = delete;

    ~Solid3Object()
    {
        DT_DestroyObject(m_object);
        delete m_shape;
    }

    void setShape(DT_ShapeHandle shape)
    {
        m_shape->setShape(shape);
    }

    void paint(bool onlyRobot = false) const
    {

        if (!onlyRobot) {
            double m[16];
            DT_GetMatrixd(m_object, m);

            glPushMatrix();
            glMultMatrixd(m);
            m_shape->paint();
            glPopMatrix();
        }else if (isRobot){
            double m[16];
            DT_GetMatrixd(m_object, m);

            glPushMatrix();
            glMultMatrixd(m);
            m_shape->paint();
            glPopMatrix();
        }
    }

    DT_ObjectHandle getHandle() const
    { return m_object; }

    Solid3Shape *getShape() const
    { return m_shape; }

    std::vector<float> getFullPointsList()
    {
        std::vector<float> shapePointList = m_shape->getFullPointsList();
        std::vector<float> transformedPointList;
        for (unsigned int i = 0; i < shapePointList.size() / 12; i++) {
            for (unsigned int j = 0; j < 3; j++)
                transformedPointList.push_back(shapePointList.at(i * 12 + j));

            double m[16];

            DT_GetMatrixd(m_object, m);
            Eigen::Matrix4d em(m);
            for (unsigned int j = 0; j < 3; j++) {
                Eigen::Vector4d point
                    (shapePointList.at(i * 12 + 3 * (j + 1)),
                     shapePointList.at(i * 12 + 3 * (j + 1) + 1),
                     shapePointList.at(i * 12 + 3 * (j + 1) + 2),
                     1);
                point = em * point;
                for (int k = 0; k < 3; k++)
                    transformedPointList.push_back(point[k]);
            }

        }
        return transformedPointList;
    }

    bool isRobot;

private:
    Solid3Shape *m_shape;
    DT_ObjectHandle m_object;
};