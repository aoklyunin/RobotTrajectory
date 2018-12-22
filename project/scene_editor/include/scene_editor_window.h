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
#include <QWidget>
#include <QtWidgets/QLabel>
#include <QtWidgets/QVBoxLayout>
#include "scene_editor_widget.h"

QT_BEGIN_NAMESPACE
class QSlider;
class QPushButton;
QT_END_NAMESPACE

class SceneEditorWidget;
class SceneEditorMainWindow;

class SceneEditorWindow: public QWidget
{
Q_OBJECT

public:
    SceneEditorWindow(SceneEditorMainWindow *mw, std::string scenePath);
    void saveScene(std::string path);

protected:
    void keyPressEvent(QKeyEvent *event) Q_DECL_OVERRIDE;

private slots:
    void dockUndock();
    void doAdd();
    void doDelete();
    void doNext();
    void doPrev();


private:
    SceneEditorWidget *glWidget;
    QPushButton *dockBtn;
    QPushButton *addBtn;
    QPushButton *deleteBtn;

    QLabel *translateLabel;
    QLabel *rotateLabel;
    QLabel *scaleLabel;

    QSlider *translateXSlider;
    QSlider *translateYSlider;
    QSlider *translateZSlider;

    QSlider *rotateXSlider;
    QSlider *rotateYSlider;
    QSlider *rotateZSlider;

    QSlider *scaleXSlider;
    QSlider *scaleYSlider;
    QSlider *scaleZSlider;

    QPushButton * nextBtn;
    QPushButton * prevBtn;

    SceneEditorMainWindow *mainWindow;

    std::shared_ptr<PathFinder> _pathFinder;
    QSlider *createSlider(int min, int max);
    QWidget *getSliderWithCaption(QSlider *pSlider, QVBoxLayout *pLayout, const char string[2]);
};
