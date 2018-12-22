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

#include "scene_editor_window.h"
#include "scene_editor_main_window.h"

#include <QSlider>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QKeyEvent>
#include <QPushButton>
#include <QDesktopWidget>
#include <QApplication>
#include <QMessageBox>
#include <solid_collider.h>
#include <QtWidgets/QFileDialog>
#include <tra_star_path_finder.h>

SceneEditorWindow::SceneEditorWindow(SceneEditorMainWindow *mw, std::string scenePath)
    : mainWindow(mw)
{

    glWidget = new SceneEditorWidget(mw);

    QVBoxLayout *mainLayout = new QVBoxLayout;
    QHBoxLayout *container = new QHBoxLayout;

    QWidget *w = new QWidget;
    w->setLayout(container);
    mainLayout->addWidget(w);

    container->addWidget(glWidget);

    QHBoxLayout *btnContainer = new QHBoxLayout;
    QWidget *btnWidget = new QWidget;
    btnWidget->setLayout(btnContainer);
    mainLayout->addWidget(btnWidget);


    dockBtn = new QPushButton(tr("Undock"), this);
    connect(dockBtn, SIGNAL(clicked()), this, SLOT(dockUndock()));
    btnContainer->addWidget(dockBtn);


    QVBoxLayout *editContainer = new QVBoxLayout;
    QWidget *editWidget = new QWidget;
    editWidget->setLayout(editContainer);

    container->addWidget(editWidget);

    QHBoxLayout *firstEditButtonContainer = new QHBoxLayout;
    QWidget *firstEditButtonWidget = new QWidget;
    firstEditButtonWidget->setLayout(firstEditButtonContainer);

    editContainer->addWidget(firstEditButtonWidget);

    addBtn = new QPushButton(tr("Add"), this);
    connect(addBtn, SIGNAL(clicked()), this, SLOT(doAdd()));
    firstEditButtonContainer->addWidget(addBtn);

    deleteBtn = new QPushButton(tr("Delete"), this);
    connect(deleteBtn, SIGNAL(clicked()), this, SLOT(doDelete()));
    firstEditButtonContainer->addWidget(deleteBtn);

    translateLabel = new QLabel(this);
    translateLabel->setText("Translate");
    translateLabel->setAlignment(Qt::AlignCenter);
    editContainer->addWidget(translateLabel);

    translateXSlider = createSlider(-500, 500);
    QWidget *translateWidgetX = getSliderWithCaption(translateXSlider, editContainer, "X");
    editContainer->addWidget(translateWidgetX);

    translateYSlider = createSlider(-500, 500);
    QWidget *translateWidgetY = getSliderWithCaption(translateYSlider, editContainer, "Y");
    editContainer->addWidget(translateWidgetY);

    translateZSlider = createSlider(-500, 500);
    QWidget *translateWidgetZ = getSliderWithCaption(translateZSlider, editContainer, "Z");
    editContainer->addWidget(translateWidgetZ);


    rotateLabel = new QLabel(this);
    rotateLabel->setText("Rotate");
    rotateLabel->setAlignment(Qt::AlignCenter);
    editContainer->addWidget(rotateLabel);

    rotateXSlider = createSlider(-360, 360);
    QWidget *rotateWidgetX = getSliderWithCaption(rotateXSlider, editContainer, "R");
    editContainer->addWidget(rotateWidgetX);

    rotateYSlider = createSlider(-360, 360);
    QWidget *rotateWidgetY = getSliderWithCaption(rotateYSlider, editContainer, "P");
    editContainer->addWidget(rotateWidgetY);

    rotateZSlider = createSlider(-360, 360);
    QWidget *rotateWidgetZ = getSliderWithCaption(rotateZSlider, editContainer, "Y");
    editContainer->addWidget(rotateWidgetZ);

    scaleLabel = new QLabel(this);
    scaleLabel->setText("Scale");
    scaleLabel->setAlignment(Qt::AlignCenter);
    editContainer->addWidget(scaleLabel);


    scaleXSlider = createSlider(1, 10000);
    QWidget *scaleWidgetX = getSliderWithCaption(scaleXSlider, editContainer, "X");
    editContainer->addWidget(scaleWidgetX);

    scaleYSlider = createSlider(1, 10000);
    QWidget *scaleWidgetY = getSliderWithCaption(scaleYSlider, editContainer, "Y");
    editContainer->addWidget(scaleWidgetY);

    scaleZSlider = createSlider(1, 10000);
    QWidget *scaleWidgetZ = getSliderWithCaption(scaleZSlider, editContainer, "Z");
    editContainer->addWidget(scaleWidgetZ);

    connect(translateXSlider, SIGNAL(valueChanged(int)), glWidget, SLOT(setScrollTranslationX(int)));
    connect(glWidget, SIGNAL(setScrollTranslationXChanged(int)), translateXSlider, SLOT(setValue(int)));

    connect(translateYSlider, SIGNAL(valueChanged(int)), glWidget, SLOT(setScrollTranslationY(int)));
    connect(glWidget, SIGNAL(setScrollTranslationYChanged(int)), translateYSlider, SLOT(setValue(int)));

    connect(translateZSlider, SIGNAL(valueChanged(int)), glWidget, SLOT(setScrollTranslationZ(int)));
    connect(glWidget, SIGNAL(setScrollTranslationZChanged(int)), translateZSlider, SLOT(setValue(int)));


    connect(rotateXSlider, SIGNAL(valueChanged(int)), glWidget, SLOT(setScrollRotationX(int)));
    connect(glWidget, SIGNAL(setScrollRotationXChanged(int)), rotateXSlider, SLOT(setValue(int)));

    connect(rotateYSlider, SIGNAL(valueChanged(int)), glWidget, SLOT(setScrollRotationY(int)));
    connect(glWidget, SIGNAL(setScrollRotationYChanged(int)), rotateYSlider, SLOT(setValue(int)));

    connect(rotateZSlider, SIGNAL(valueChanged(int)), glWidget, SLOT(setScrollRotationZ(int)));
    connect(glWidget, SIGNAL(setScrollRotationZChanged(int)), rotateZSlider, SLOT(setValue(int)));


    connect(scaleXSlider, SIGNAL(valueChanged(int)), glWidget, SLOT(setScrollScaleX(int)));
    connect(glWidget, SIGNAL(setScrollScaleXChanged(int)), scaleXSlider, SLOT(setValue(int)));

    connect(scaleYSlider, SIGNAL(valueChanged(int)), glWidget, SLOT(setScrollScaleY(int)));
    connect(glWidget, SIGNAL(setScrollScaleYChanged(int)), scaleYSlider, SLOT(setValue(int)));

    connect(scaleZSlider, SIGNAL(valueChanged(int)), glWidget, SLOT(setScrollScaleZ(int)));
    connect(glWidget, SIGNAL(setScrollScaleZChanged(int)), scaleZSlider, SLOT(setValue(int)));


    QHBoxLayout *secondEditButtonContainer = new QHBoxLayout;
    QWidget *secondEditButtonWidget = new QWidget;
    secondEditButtonWidget->setLayout(secondEditButtonContainer);

    editContainer->addWidget(secondEditButtonWidget);

    nextBtn = new QPushButton(tr("Next"), this);
    connect(nextBtn, SIGNAL(clicked()), this, SLOT(doNext()));
    secondEditButtonContainer->addWidget(nextBtn);

    prevBtn = new QPushButton(tr("Prev"), this);
    connect(prevBtn, SIGNAL(clicked()), this, SLOT(doPrev()));
    secondEditButtonContainer->addWidget(prevBtn);

    std::shared_ptr<SceneWrapper> sceneWrapper = std::make_shared<SceneWrapper>();
    sceneWrapper->buildFromFile(scenePath);

    _pathFinder = std::make_shared<AStarPathFinder>(sceneWrapper, true, 100, 10, 4000, 251, 1, 1, 0, 2);


    glWidget->setSliders(std::vector<QSlider *>{
        translateXSlider, translateYSlider, translateZSlider,
        rotateXSlider, rotateYSlider, rotateZSlider,
        scaleXSlider, scaleYSlider, scaleZSlider
    });

    glWidget->setPathFinder(_pathFinder);


    QDesktopWidget dw;

    glWidget->setFixedSize(dw.width() * 0.5, dw.height() * 0.7);

    setLayout(mainLayout);
    setWindowTitle(tr("Scene Editor"));
}

QSlider *SceneEditorWindow::createSlider(int min, int max)
{
    QSlider *slider = new QSlider(Qt::Horizontal);
    slider->setRange(min, max);
    return slider;
}

void SceneEditorWindow::keyPressEvent(QKeyEvent *e)
{
    if (e->key() == Qt::Key_Escape)
        close();
    else
        QWidget::keyPressEvent(e);
}

void SceneEditorWindow::dockUndock()
{
    if (parent()) {
        setParent(0);
        setAttribute(Qt::WA_DeleteOnClose);
        move(QApplication::desktop()->width() / 2 - width() / 2,
             QApplication::desktop()->height() / 2 - height() / 2);
        dockBtn->setText(tr("Dock"));
        show();
    }
    else {
        if (!mainWindow->centralWidget()) {
            if (mainWindow->isVisible()) {
                setAttribute(Qt::WA_DeleteOnClose, false);
                dockBtn->setText(tr("Undock"));
                mainWindow->setCentralWidget(this);
            }
            else {
                QMessageBox::information(0, tr("Cannot dock"), tr("Main window already closed"));
            }
        }
        else {
            QMessageBox::information(0, tr("Cannot dock"), tr("Main window already occupied"));
        }
    }
}
void SceneEditorWindow::doAdd()
{
    QString fileName = QFileDialog::getOpenFileName(this,
                                                    QString::fromUtf8("Открыть файл"),
                                                    "/opt/tra/path_finding/config/urdf");
    info_msg(fileName.toStdString());
    //_scenewrapper->addModel();

    //info_msg("before adding");
    glWidget->doAdd(fileName.toStdString());

    info_msg("SceneEditorWindow do add complete");
}

void SceneEditorWindow::doDelete()
{
    glWidget->doDelete();
}

QWidget *SceneEditorWindow::getSliderWithCaption(QSlider *pSlider, QVBoxLayout *pLayout, const char *caption)
{
    QHBoxLayout *sliderLayout = new QHBoxLayout;
    QWidget *sliderWidget = new QWidget;
    sliderWidget->setLayout(sliderLayout);

    sliderLayout->addWidget(pSlider);

    QLabel *sliderLabel = new QLabel(this);
    sliderLabel->setText(caption);
    sliderLayout->addWidget(sliderLabel);

    return sliderWidget;

}

void SceneEditorWindow::doNext()
{
    glWidget->doNext();
}
void SceneEditorWindow::doPrev()
{
    glWidget->doPrev();
}
void SceneEditorWindow::saveScene(std::string path)
{
    _pathFinder->getSceneWrapper()->saveScene(path);
}


