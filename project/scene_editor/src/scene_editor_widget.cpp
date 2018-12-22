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

#include <scene_editor_widget.h>

#include <QMouseEvent>
#include <QOpenGLShaderProgram>
#include <QCoreApplication>
#include <math.h>
#include <iostream>
#include <solid_collider.h>
#include <QtCore/QTimer>
#include <QTimer>
#include <QtWidgets/QSlider>
#include <cmath>
#include <local_path_finder.h>

SceneEditorWidget::SceneEditorWidget(QWidget *parent)
    : QOpenGLWidget(parent),
      m_xRot(0),
      m_yRot(0),
      m_zRot(0),
      m_program(0)
{
    _flgPlay = false;
    //_slider = slider;
    _parent = parent;

    m_core = QCoreApplication::arguments().contains(QStringLiteral("--coreprofile"));
    // --transparent causes the clear color to be transparent. Therefore, on systems that
    // support it, the widget will become transparent apart from the logo.
    m_transparent = QCoreApplication::arguments().contains(QStringLiteral("--transparent"));
    if (m_transparent)
        setAttribute(Qt::WA_TranslucentBackground);
    actualRobotNum = 0;

    for (unsigned int i = 0; i < 9; i++)
        _sliderValues.push_back(0);

    visualizationDrawer.setScale(_scene_scale);
}

SceneEditorWidget::~SceneEditorWidget()
{
    cleanup();
}

QSize SceneEditorWidget::minimumSizeHint() const
{
    return QSize(50, 50);
}

QSize SceneEditorWidget::sizeHint() const
{
    return QSize(400, 400);
}

static void qNormalizeAngle(int &angle)
{
    while (angle < 0)
        angle += 360 * 16;
    while (angle > 360 * 16)
        angle -= 360 * 16;
}

void SceneEditorWidget::setXRotation(int angle)
{

    qNormalizeAngle(angle);
    if (angle != m_xRot) {
        m_xRot = angle;
        emit xRotationChanged(angle);
        update();
    }

}

void SceneEditorWidget::setYRotation(int angle)
{
    qNormalizeAngle(angle);
    if (angle != m_yRot) {
        m_yRot = angle;
        emit yRotationChanged(angle);
        update();
    }
}

void SceneEditorWidget::setZRotation(int angle)
{
    qNormalizeAngle(angle);
    if (angle != m_zRot) {
        m_zRot = angle;
        emit zRotationChanged(angle);
        update();
    }
}

void SceneEditorWidget::cleanup()
{
    makeCurrent();
    m_logoVbo.destroy();
    delete m_program;
    m_program = 0;
    doneCurrent();
}

static const char *vertexShaderSourceCore =
    "#version 150\n"
        "in vec4 vertex;\n"
        "in vec3 normal;\n"
        "out vec3 vert;\n"
        "out vec3 vertNormal;\n"
        "uniform mat4 projMatrix;\n"
        "uniform mat4 mvMatrix;\n"
        "uniform mat3 normalMatrix;\n"
        "void main() {\n"
        "   vert = vertex.xyz;\n"
        "   vertNormal = normalMatrix * normal;\n"
        "   gl_Position = projMatrix * mvMatrix * vertex;\n"
        "}\n";

static const char *fragmentShaderSourceCore =
    "#version 150\n"
        "in highp vec3 vert;\n"
        "in highp vec3 vertNormal;\n"
        "out highp vec4 fragColor;\n"
        "uniform highp vec3 lightPos;\n"
        "void main() {\n"
        "   highp vec3 L = normalize(lightPos - vert);\n"
        "   highp float NL = max(dot(normalize(vertNormal), L), 0.0);\n"
        "   highp vec3 color = vec3(0.39, 1.0, 0.0);\n"
        "   highp vec3 col = clamp(color * 0.2 + color * 0.8 * NL, 0.0, 1.0);\n"
        "   fragColor = vec4(col, 1.0);\n"
        "}\n";

static const char *vertexShaderSource =
    "attribute vec4 vertex;\n"
        "attribute vec3 normal;\n"
        "varying vec3 vert;\n"
        "varying vec3 vertNormal;\n"
        "uniform mat4 projMatrix;\n"
        "uniform mat4 mvMatrix;\n"
        "uniform mat3 normalMatrix;\n"
        "void main() {\n"
        "   vert = vertex.xyz;\n"
        "   vertNormal = normalMatrix * normal;\n"
        "   gl_Position = projMatrix * mvMatrix * vertex;\n"
        "}\n";

static const char *fragmentShaderSource =
    "varying highp vec3 vert;\n"
        "varying highp vec3 vertNormal;\n"
        "uniform highp vec3 lightPos;\n"
        "void main() {\n"
        "   highp vec3 L = normalize(lightPos - vert);\n"
        "   highp float NL = max(dot(normalize(vertNormal), L), 0.0);\n"
        "   highp vec3 color = vec3(0.39, 1.0, 0.0);\n"
        "   highp vec3 col = clamp(color * 0.2 + color * 0.8 * NL, 0.0, 1.0);\n"
        "   gl_FragColor = vec4(col, 1.0);\n"
        "}\n";

void SceneEditorWidget::initializeGL()
{
    // In this example the widget's corresponding top-level window can change
    // several times during the widget's lifetime. Whenever this happens, the
    // QOpenGLWidget's associated context is destroyed and a new one is created.
    // Therefore we have to be prepared to clean up the resources on the
    // aboutToBeDestroyed() signal, instead of the destructor. The emission of
    // the signal will be followed by an invocation of initializeGL() where we
    // can recreate all resources.
    connect(context(), &QOpenGLContext::aboutToBeDestroyed, this, &SceneEditorWidget::cleanup);

    initializeOpenGLFunctions();
    glClearColor(0, 0, 0, m_transparent ? 0 : 1);

    m_program = new QOpenGLShaderProgram;
    m_program->addShaderFromSourceCode(QOpenGLShader::Vertex, m_core ? vertexShaderSourceCore : vertexShaderSource);
    m_program
        ->addShaderFromSourceCode(QOpenGLShader::Fragment, m_core ? fragmentShaderSourceCore : fragmentShaderSource);


    m_program->bindAttributeLocation("vertex", 0);
    m_program->bindAttributeLocation("normal", 1);
    m_program->link();

    m_program->bind();
    m_projMatrixLoc = m_program->uniformLocation("projMatrix");
    m_mvMatrixLoc = m_program->uniformLocation("mvMatrix");
    m_normalMatrixLoc = m_program->uniformLocation("normalMatrix");
    m_lightPosLoc = m_program->uniformLocation("lightPos");

    // Create a vertex array object. In OpenGL ES 2.0 and OpenGL 2.x
    // implementations this is optional and support may not be present
    // at all. Nonetheless the below code works in all cases and makes
    // sure there is a VAO when one is needed.
    m_vao.create();
    QOpenGLVertexArrayObject::Binder vaoBinder(&m_vao);

    // Setup our vertex buffer object.
    m_logoVbo.create();
    m_logoVbo.bind();
    m_logoVbo.allocate(visualizationDrawer.constData(), visualizationDrawer.count() * sizeof(GLfloat));

    // Store the vertex attribute bindings for the program.
    setupVertexAttribs();

    // Our camera never changes in this example.
    m_camera.setToIdentity();
    m_camera.translate(0, 0, -1);
    m_camera.rotate(180, 0, 1, 0);

    // Light position is fixed.
    m_program->setUniformValue(m_lightPosLoc, QVector3D(0, 0, 70));

    m_program->release();
}

void SceneEditorWidget::setupVertexAttribs()
{
    m_logoVbo.bind();
    QOpenGLFunctions *f = QOpenGLContext::currentContext()->functions();
    f->glEnableVertexAttribArray(0);
    f->glEnableVertexAttribArray(1);
    f->glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(GLfloat), 0);
    f->glVertexAttribPointer(1,
                             3,
                             GL_FLOAT,
                             GL_FALSE,
                             6 * sizeof(GLfloat),
                             reinterpret_cast<void *>(3 * sizeof(GLfloat)));
    m_logoVbo.release();
}

void SceneEditorWidget::paintGL()
{
   // info_msg("paint GL");
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glEnable(GL_DEPTH_TEST);
    //  glEnable(GL_CULL_FACE);

    m_world.setToIdentity();
    m_world.rotate(180.0f - (m_xRot / 16.0f), 1, 0, 0);
    m_world.rotate(m_yRot / 16.0f, 0, 1, 0);
    m_world.rotate(m_zRot / 16.0f, 0, 0, 1);

    QOpenGLVertexArrayObject::Binder vaoBinder(&m_vao);
    m_program->bind();
    m_program->setUniformValue(m_projMatrixLoc, m_proj);
    m_program->setUniformValue(m_mvMatrixLoc, m_camera * m_world);
    QMatrix3x3 normalMatrix = m_world.normalMatrix();
    m_program->setUniformValue(m_normalMatrixLoc, normalMatrix);

    glDrawArrays(GL_TRIANGLES, 0, visualizationDrawer.vertexCount());

    paintBox();

    char buf[256];
    sprintf(buf, "ExpNum %d: ", _curExpNum);
    std::string caption = buf;
//    if (!_state.empty())
//        caption += _pathReader->getSceneWrapper()->checkCollision(_state) ? "Collided" : "No collision";
    _parent->setWindowTitle(caption.c_str());
    _parent->update();

    m_program->release();

}

void SceneEditorWidget::resizeGL(int w, int h)
{
    m_proj.setToIdentity();
    m_proj.perspective(45.0f, GLfloat(w) / h, 0.01f, 100.0f);
}

void SceneEditorWidget::mousePressEvent(QMouseEvent *event)
{
    m_lastPos = event->pos();
}

void SceneEditorWidget::mouseMoveEvent(QMouseEvent *event)
{
    int dx = event->x() - m_lastPos.x();
    int dy = event->y() - m_lastPos.y();

    if (event->buttons() & Qt::LeftButton) {
        setXRotation(m_xRot + 8 * dy);
        setYRotation(m_yRot + 8 * dx);
    }
    else if (event->buttons() & Qt::RightButton) {
        setXRotation(m_xRot + 8 * dy);
        setZRotation(m_zRot + 8 * dx);
    }
    m_lastPos = event->pos();
}

void SceneEditorWidget::setPathFinder(std::shared_ptr<PathFinder> pathFinder)
{
    assert(pathFinder);
    _pathFinder = pathFinder;

    info_msg("SceneEditorWidget::setPathFinder");

    //SceneWrapper::dispState(_translation," start translation");

//    SceneWrapper::dispState(_translation, "tr");
//    SceneWrapper::dispState(_rotation, "rot");
//    SceneWrapper::dispState(_scale, "scale");




    //  info_msg("before random state");
    std::vector<double> state = pathFinder->getSceneWrapper()->getRandomState();
    SceneWrapper::dispState(state,"random state");
    _pathFinder->setState(state);
    visualizationDrawer.setPathFinder(_pathFinder);

    info_msg("after random state");
    visualizationDrawer.setState(state);



    info_msg("scene editor: set scene wrapper complete");

    _modifySliders();

    update();
}

void SceneEditorWidget::_setChanges()
{
    assert(_pathFinder);
    info_msg("SceneEditorWidget: setChanges ");
    visualizationDrawer.setPathFinder(_pathFinder);

    info_msg("visualizationDrawer.setSceneWrapper(_sceneWrapper) - complete");

    m_logoVbo.bind();
    m_logoVbo.allocate(visualizationDrawer.constData(), visualizationDrawer.count() * sizeof(GLfloat));
    m_logoVbo.release();

    update();

}

void SceneEditorWidget::makeChangeExpNum()
{
    _tm = 0;
    // _slider->setMinimum(0);
    //  _slider->setValue(0);
//    _slider->setMaximum((int) _pathReader->getDuration(_curExpNum) * 1000);
//    _slider->update();
//    visualizationDrawer.setState(_pathReader->getState(_curExpNum, 0));
    //  _setChanges(0);
}

void SceneEditorWidget::doAdd(std::string path)
{
    info_msg("scene editor widget: do Add");
    _pathFinder->getSceneWrapper()->addModel(path);
    info_msg("model added");
    _modifySliders();
    info_msg("sliders modified");
    _setChanges();
    info_msg("changes seted");
}

void SceneEditorWidget::doDelete()
{
    if (_pathFinder->getSceneWrapper()->getRobotCnt() > 0) {
        _pathFinder->getSceneWrapper()->deleteModel(actualRobotNum);
        if(actualRobotNum>=_pathFinder->getSceneWrapper()->getRobotCnt()){
            actualRobotNum = 0;
        }
        _setChanges();
        _modifySliders();
    }
}

void SceneEditorWidget::doPrev()
{
    if (_pathFinder->getSceneWrapper()->getRobotCnt() > 0) {
        if (actualRobotNum == 0)
            actualRobotNum = _pathFinder->getSceneWrapper()->getRobotCnt() - 1;
        else
            actualRobotNum--;

        _modifySliders();
    }
}

void SceneEditorWidget::doNext()
{
    if (_pathFinder->getSceneWrapper()->getRobotCnt() > 0) {
        actualRobotNum++;
        if (actualRobotNum >= _pathFinder->getSceneWrapper()->getRobotCnt())
            actualRobotNum = 0;
    }
}

void SceneEditorWidget::paintBox()
{
    //info_msg("paint box");
    if (_pathFinder->getSceneWrapper()->getRobotCnt()>0) {
        std::vector<double> chords =
            _pathFinder->getBoxChords(
                actualRobotNum,
                _pathFinder->getState()
            );
        // SceneWrapper::dispState(chords, "paint box");
        glColor3f(1.0f, 1.0f, 1.0f);
        glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
        glBegin(GL_QUAD_STRIP);
        glVertex3d(chords.at(0) * _scene_scale, chords.at(1) * _scene_scale, chords.at(2) * _scene_scale);
        glVertex3d(chords.at(0) * _scene_scale, chords.at(1) * _scene_scale, chords.at(5) * _scene_scale);
        glVertex3d(chords.at(3) * _scene_scale, chords.at(1) * _scene_scale, chords.at(2) * _scene_scale);
        glVertex3d(chords.at(3) * _scene_scale, chords.at(1) * _scene_scale, chords.at(5) * _scene_scale);
        glVertex3d(chords.at(3) * _scene_scale, chords.at(4) * _scene_scale, chords.at(2) * _scene_scale);
        glVertex3d(chords.at(3) * _scene_scale, chords.at(4) * _scene_scale, chords.at(5) * _scene_scale);
        glVertex3d(chords.at(0) * _scene_scale, chords.at(4) * _scene_scale, chords.at(2) * _scene_scale);
        glVertex3d(chords.at(0) * _scene_scale, chords.at(4) * _scene_scale, chords.at(5) * _scene_scale);
        glVertex3d(chords.at(0) * _scene_scale, chords.at(1) * _scene_scale, chords.at(2) * _scene_scale);
        glVertex3d(chords.at(0) * _scene_scale, chords.at(1) * _scene_scale, chords.at(5) * _scene_scale);
        glEnd();
        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    }
}

void SceneEditorWidget::setScrollTranslationX(int angle)
{
    _sliderValues.at(0) = angle;
    _applySliders();
    emit setScrollTranslationXChanged(angle);

}

void SceneEditorWidget::setScrollTranslationY(int angle)
{
    _sliderValues.at(1) = angle;
    _applySliders();
    emit setScrollTranslationYChanged(angle);

}

void SceneEditorWidget::setScrollTranslationZ(int angle)
{
    _sliderValues.at(2) = angle;
    _applySliders();
    emit setScrollTranslationZChanged(angle);
}

void SceneEditorWidget::setScrollRotationX(int angle)
{
    _sliderValues.at(3) = angle;
    _applySliders();
    emit setScrollRotationXChanged(angle);

}

void SceneEditorWidget::setScrollRotationY(int angle)
{
    _sliderValues.at(4) = angle;
    _applySliders();
    emit setScrollRotationYChanged(angle);

}

void SceneEditorWidget::setScrollRotationZ(int angle)
{
    _sliderValues.at(5) = angle;
    _applySliders();
    emit setScrollRotationZChanged(angle);
}

void SceneEditorWidget::setScrollScaleX(int angle)
{
    _sliderValues.at(6) = angle;
    _applySliders();
    emit setScrollScaleXChanged(angle);

}

void SceneEditorWidget::setScrollScaleY(int angle)
{
    _sliderValues.at(7) = angle;
    _applySliders();
    emit setScrollScaleYChanged(angle);

}

void SceneEditorWidget::setScrollScaleZ(int angle)
{
    _sliderValues.at(8) = angle;
    _applySliders();
    emit setScrollScaleZChanged(angle);
}

void SceneEditorWidget::setSliders(std::vector<QSlider *> sliders)
{
    _sliders = std::move(sliders);
}

void SceneEditorWidget::_applySliders()
{
    info_msg("apply sliders");
    LocalPathFinder::dispChords(_sliderValues, "slide values");

    for (unsigned int i = 0; i < 3; i++) {
        _groupedStartTranslation.at(actualRobotNum).at(i) =
            _sliderToTranslation(_sliderValues.at(i));
        _groupedStartRotation.at(actualRobotNum).at(i) =
            _sliderToRotation(_sliderValues.at(i + 3));
        _groupedStartScale.at(actualRobotNum).at(i) =
            _sliderToScale(_sliderValues.at(i + 6));
    }

    _pathFinder->getSceneWrapper()->setGroupedTranslation(_groupedStartTranslation);
    _pathFinder->getSceneWrapper()->setGroupedRotation(_groupedStartRotation);
    _pathFinder->getSceneWrapper()->setGroupedScale(_groupedStartScale);

   // info_msg(actualRobotNum);

   // LocalPathFinder::dispChords(_sliderValues, "slider values");

    _setChanges();

}

void SceneEditorWidget::_modifySliders()
{
    if (_pathFinder->getSceneWrapper()->getRobotCnt() > 0) {
        info_msg("modify sliders");

        _groupedStartTranslation = _pathFinder->getSceneWrapper()->getGroupedTranslation();
        _groupedStartRotation = _pathFinder->getSceneWrapper()->getGroupedRotation();
        _groupedStartScale = _pathFinder->getSceneWrapper()->getGroupedScale();

        LocalPathFinder::dispChords(_sliderValues, "slider values before");

        for (unsigned int i = 0; i < 3; i++) {
            info_msg(_translationToSlider(
                _groupedStartTranslation.at(actualRobotNum).at(i)
            ));
            _sliderValues.at(i) = _translationToSlider(
                _groupedStartTranslation.at(actualRobotNum).at(i)
            );
            _sliderValues.at(i + 3) = _rotationToSlider(
                _groupedStartRotation.at(actualRobotNum).at(i)
            );
            _sliderValues.at(i + 6) = _scaleToSlider(
                _groupedStartScale.at(actualRobotNum).at(i)
            );
        }

        LocalPathFinder::dispChords(_sliderValues, "slider values after");

        for (unsigned int i = 0; i < 9; i++) {
            _sliders.at(i)->setValue(_sliderValues.at(i));
            _sliders.at(i)->update();
        }
    }
}

double SceneEditorWidget::_sliderToTranslation(int translation)
{
    return (double) translation / 100;
}

double SceneEditorWidget::_sliderToRotation(int rotation)
{
    return (double) rotation / 180 * M_PI;
}

double SceneEditorWidget::_sliderToScale(int scale)
{
    return (double) scale / 1000;
}

int SceneEditorWidget::_translationToSlider(double translation)
{
    return (int) (translation * 100);
}

int SceneEditorWidget::_rotationToSlider(double rotation)
{
    return (int) (rotation / 180 * M_PI);
}

int SceneEditorWidget::_scaleToSlider(double scale)
{
    return (int) (scale * 1000);
}


