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

#include "scene_editor_drawer.h"
#include <qmath.h>
#include <memory>
#include <utility>

SceneEditorDrawer::SceneEditorDrawer()
    : m_count(0)
{
}

void SceneEditorDrawer::setScale(double scale)
{
    _scale = scale;
}

void SceneEditorDrawer::add(const QVector3D &v, const QVector3D &n)
{
    GLfloat *p = m_data.data() + m_count;
    *p++ = v.x();
    *p++ = v.y();
    *p++ = v.z();
    *p++ = n.x();
    *p++ = n.y();
    *p++ = n.z();
    m_count += 6;
}

void SceneEditorDrawer::setState(const std::vector<double> &state)
{
    assert(!state.empty());
    SceneWrapper::dispState(state, "drawer incoming state");
    info_msg("sceene editor drawer: set state");
    //info_msg("set state scene editor drawer");
    //info_msg("set state");

    //  info_msg(_sceneWrapper->getJointCnt());
    //  auto matrices = _sceneWrapper->getSceneDescrition()->getTrasformMatrices(state);

    // info_msg("matrices size ",matrices.size());
    // SceneWrapper::dispState(state, "incoming state ");

//    info_msg("sceene editor drawer: set state");

    std::vector<Eigen::Matrix4d> matrices = _pathFinder->getSceneWrapper()->getTrasformMatrices(state);
    info_msg("matrices geted: ",matrices.size());

    _pathFinder->updateCollider();

   // info_msg(_sceneWrapper->getCollider()->getLinks().size());

    const std::vector<float> &points = _pathFinder->getCollider()->getPoints(matrices);

    //  std::vector<Eigen::Matrix4d> ms = _sceneWrapper->getSceneDescrition()->getTrasformMatrices(state);
    //  info_msg(ms.at(1));

    //info_msg("setState to logo ", points.size());

    m_count = 0;
    m_data.resize(static_cast<int>(points.size() * 6));

    unsigned long triCnt = points.size() / 12;

    for (unsigned int i = 0; i < triCnt; i++) {
        QVector3D normal{static_cast<float>(points.at(i * 12) * _scale),
                         static_cast<float>(points.at(i * 12 + 1) * _scale),
                         static_cast<float>(points.at(i * 12 + 2) * _scale)};
        QVector3D point1{static_cast<float>(points.at(i * 12 + 3) * _scale),
                         static_cast<float>(points.at(i * 12 + 4) * _scale),
                         static_cast<float>(points.at(i * 12 + 5) * _scale)};
        QVector3D point2{static_cast<float>(points.at(i * 12 + 6) * _scale),
                         static_cast<float>(points.at(i * 12 + 7) * _scale),
                         static_cast<float>(points.at(i * 12 + 8) * _scale)};
        QVector3D point3{static_cast<float>(points.at(i * 12 + 9) * _scale),
                         static_cast<float>(points.at(i * 12 + 10) * _scale),
                         static_cast<float>(points.at(i * 12 + 11) * _scale)};
        add(point1, normal);
        add(point2, normal);
        add(point3, normal);
    }

    info_msg("after set state");

}

void SceneEditorDrawer::setPathFinder(std::shared_ptr<PathFinder> pathFinder)
{
   // info_msg("SceneEditorDrawer::setPathFinder");

    assert(pathFinder);
   // info_msg("scene editor drawer");
    _pathFinder = pathFinder;
    assert(!_pathFinder->getState().empty());
    SceneWrapper::dispState(_pathFinder->getState(),"set scene wrapper");
    setState(_pathFinder->getState());

}

