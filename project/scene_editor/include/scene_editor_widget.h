/****************************************************************************
**
** Copyright (C) 2015 The Qt Company Ltd.
** Contact: http://www.qt.io/licensing/
**
** This file is part of the examples of the Qt Toolkit.
**
** $QT_BEGIN_LICENSE:BSD$
** You may use this file under the terms of the BSD license as follows:
**
** "Redistribution and use in source and binary forms, with or without
** modification, are permitted provided that the following conditions are
** met:
**   * Redistributions of source code must retain the above copyright
**     notice, this list of conditions and the following disclaimer.
**   * Redistributions in binary form must reproduce the above copyright
**     notice, this list of conditions and the following disclaimer in
**     the documentation and/or other materials provided with the
**     distribution.
**   * Neither the name of The Qt Company Ltd nor the names of its
**     contributors may be used to endorse or promote products derived
**     from this software without specific prior written permission.
**
**
** THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
** "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
** LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
** A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
** OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
** SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
** LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
** DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
** THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
** (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
** OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE."
**
** $QT_END_LICENSE$
**
****************************************************************************/
#pragma  once

#include <QOpenGLWidget>
#include <QOpenGLFunctions>
#include <QOpenGLVertexArrayObject>
#include <QOpenGLBuffer>
#include <QMatrix4x4>
#include <scene_wrapper.h>
#include <QtWidgets/QSlider>
#include "scene_editor_drawer.h"

QT_FORWARD_DECLARE_CLASS(QOpenGLShaderProgram)


class SceneEditorWidget: public QOpenGLWidget, protected QOpenGLFunctions
{
Q_OBJECT



public:


    explicit SceneEditorWidget(QWidget *parent = 0);
    ~SceneEditorWidget() override;

    QSize minimumSizeHint() const Q_DECL_OVERRIDE;
    QSize sizeHint() const Q_DECL_OVERRIDE;

    void setSliders(std::vector<QSlider *> vector);

    void setPathFinder(std::shared_ptr<PathFinder> pathFinder);


    void doAdd(std::string);
    void doDelete();
    void doPrev();
    void doNext();

public slots:
    void setXRotation(int angle);
    void setYRotation(int angle);
    void setZRotation(int angle);
    void setScrollTranslationX(int angle);
    void setScrollTranslationY(int angle);
    void setScrollTranslationZ(int angle);

    void setScrollRotationX(int angle);
    void setScrollRotationY(int angle);
    void setScrollRotationZ(int angle);

    void setScrollScaleX(int angle);
    void setScrollScaleY(int angle);
    void setScrollScaleZ(int angle);
    void cleanup();

signals:
    void setScrollTranslationXChanged(int angle);
    void setScrollTranslationYChanged(int angle);
    void setScrollTranslationZChanged(int angle);

    void setScrollRotationXChanged(int angle);
    void setScrollRotationYChanged(int angle);
    void setScrollRotationZChanged(int angle);

    void setScrollScaleXChanged(int angle);
    void setScrollScaleYChanged(int angle);
    void setScrollScaleZChanged(int angle);
    void xRotationChanged(int angle);
    void yRotationChanged(int angle);
    void zRotationChanged(int angle);

    void timeChanged(int angle);

protected:
    void initializeGL() Q_DECL_OVERRIDE;
    void paintGL() Q_DECL_OVERRIDE;
    void resizeGL(int width, int height) Q_DECL_OVERRIDE;
    void mousePressEvent(QMouseEvent *event) Q_DECL_OVERRIDE;
    void mouseMoveEvent(QMouseEvent *event) Q_DECL_OVERRIDE;

private:
    void setupVertexAttribs();

    double _scene_scale = 0.3;

    bool m_core;
    int m_xRot;
    int m_yRot;
    int m_zRot;
    QPoint m_lastPos;
    SceneEditorDrawer visualizationDrawer;
    QOpenGLVertexArrayObject m_vao;
    QOpenGLBuffer m_logoVbo;
    QOpenGLShaderProgram *m_program;
    int m_projMatrixLoc;
    int m_mvMatrixLoc;
    int m_normalMatrixLoc;
    int m_lightPosLoc;
    QMatrix4x4 m_proj;
    QMatrix4x4 m_camera;
    QMatrix4x4 m_world;
    bool m_transparent;

  //  std::shared_ptr<PathReader> _pathReader;

    static const int _DELAY_MS=20;

    double _tm;
    bool _flgPlay;

    void _setChanges();

    int _expCnt;
    int _curExpNum;
    void makeChangeExpNum();

    QWidget *_parent;

    std::shared_ptr<PathFinder> _pathFinder;

    unsigned long actualRobotNum;
    void paintBox();

    std::vector<double> _startTranslation;
    std::vector<double> _startRotation;
    std::vector<double> _startScale;

    std::vector<std::vector<double>> _groupedStartTranslation;
    std::vector<std::vector<double>> _groupedStartRotation;
    std::vector<std::vector<double>> _groupedStartScale;

    std::vector<QSlider *> _sliders;


    std::vector<int> _sliderValues;

    void _applySliders();
    void _modifySliders();
    double _sliderToTranslation(int translation);
    double _sliderToRotation(int rotation);
    double _sliderToScale(int scale);
    int _translationToSlider(double translation);
    int _rotationToSlider(double rotation);
    int _scaleToSlider(double scale);
};
