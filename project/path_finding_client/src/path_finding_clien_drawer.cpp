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

#include "path_finding_clien_drawer.h"
#include <qmath.h>
#include <memory>
#include <utility>

PathVisulaizationDrawer::PathVisulaizationDrawer()
    : m_count(0)
{

}

void PathVisulaizationDrawer::add(const QVector3D &v, const QVector3D &n)
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

void PathVisulaizationDrawer::setState(std::vector<double> &state)
{
    //info_msg("set state");
    float scale = 0.3;

    assert(_pathFinder);

  //  info_msg("state size ",state.size());
    // SceneWrapper::dispState(state, "incoming state ");
    auto matrices = _pathFinder->getSceneWrapper()->getTrasformMatrices(std::move(state));

    const std::vector<float> &points =
        _pathFinder->getCollider()->getPoints(matrices);

    //  std::vector<Eigen::Matrix4d> ms = _sceneWrapper->getSceneDescrition()->getTrasformMatrices(state);
    //  info_msg(ms.at(1));

    // info_msg("setState to logo ", points.size());

    m_count = 0;
    m_data.clear();
    m_data.resize(points.size() * 12);

    unsigned long triCnt = points.size() / 12;

    for (unsigned int i = 0; i < triCnt; i++) {
        QVector3D normal{points.at(i * 12) * scale, points.at(i * 12 + 1) * scale, points.at(i * 12 + 2) * scale};
        QVector3D point1{points.at(i * 12 + 3) * scale, points.at(i * 12 + 4) * scale, points.at(i * 12 + 5) * scale};
        QVector3D point2{points.at(i * 12 + 6) * scale, points.at(i * 12 + 7) * scale, points.at(i * 12 + 8) * scale};
        QVector3D point3{points.at(i * 12 + 9) * scale, points.at(i * 12 + 10) * scale, points.at(i * 12 + 11) * scale};
        add(point1, normal);
        add(point2, normal);
        add(point3, normal);
    }
    //  info_msg("state seted");
}

void PathVisulaizationDrawer::setPathFinder(std::shared_ptr<PathFinder> pathFinder)
{
    _pathFinder = pathFinder;
}
