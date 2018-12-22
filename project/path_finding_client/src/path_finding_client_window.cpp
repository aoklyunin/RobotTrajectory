#include "path_finding_client_window.h"
#include "path_finding_client_main_window.h"

#include <QSlider>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QKeyEvent>
#include <QPushButton>
#include <QDesktopWidget>
#include <QApplication>
#include <QMessageBox>
#include <utility>

PathVisualizationWindow::PathVisualizationWindow(PathVisualizationMainWindow *mw, std::string logPath)
    : mainWindow(mw)
{
    _flgPlay = false;

    timeSlider = createSlider();

    glWidget = new PathVisualisationWidget(timeSlider, mw);

    std::shared_ptr<PathReader> pathreader = std::make_shared<PathReader>(5);
    pathreader->init(logPath);

    info_msg("pathreader init");
    glWidget->setPathReader(pathreader);

    info_msg("pathreader seted");

    timeSlider->setMinimum(0);
    timeSlider->setValue(0);
    timeSlider->setMaximum((int) pathreader->getDuration(0) * 1000);
    timeSlider->update();

    connect(timeSlider, SIGNAL(valueChanged(int)), glWidget, SLOT(setTime(int)));
    connect(glWidget, SIGNAL(timeChanged(int)), timeSlider, SLOT(setValue(int)));

    auto *mainLayout = new QVBoxLayout;
    auto *container = new QHBoxLayout;
    container->addWidget(glWidget);
    container->addWidget(timeSlider);

    QWidget *w = new QWidget;
    w->setLayout(container);
    mainLayout->addWidget(w);

    auto *btnContainer = new QHBoxLayout;
    QWidget *btnWidget = new QWidget;
    btnWidget->setLayout(btnContainer);
    mainLayout->addWidget(btnWidget);

    dockBtn = new QPushButton(tr("Undock"), this);
    connect(dockBtn, SIGNAL(clicked()), this, SLOT(dockUndock()));
    btnContainer->addWidget(dockBtn);

    nextBtn = new QPushButton(tr("Next"), this);
    connect(nextBtn, SIGNAL(clicked()), this, SLOT(next()));
    btnContainer->addWidget(nextBtn);

    prevBtn = new QPushButton(tr("Prev"), this);
    connect(prevBtn, SIGNAL(clicked()), this, SLOT(prev()));
    btnContainer->addWidget(prevBtn);

    playBtn = new QPushButton(tr("Play"), this);
    connect(playBtn, SIGNAL(clicked()), this, SLOT(play()));
    btnContainer->addWidget(playBtn);

    setLayout(mainLayout);

    timeSlider->setValue(15 * 16);

    setWindowTitle(tr("Path Visualization"));
}

QSlider *PathVisualizationWindow::createSlider()
{
    auto *slider = new QSlider(Qt::Vertical);
    slider->setRange(0, 360 * 16);
    slider->setSingleStep(16);
    slider->setPageStep(15 * 16);
    slider->setTickInterval(15 * 16);
    slider->setTickPosition(QSlider::TicksRight);
    return slider;
}

void PathVisualizationWindow::keyPressEvent(QKeyEvent *e)
{
    if (e->key() == Qt::Key_Escape)
        close();
    else
        QWidget::keyPressEvent(e);
}

void PathVisualizationWindow::dockUndock()
{
    if (parent()) {
        setParent(nullptr);
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
                QMessageBox::information(nullptr, tr("Cannot dock"), tr("Main window already closed"));
            }
        }
        else {
            QMessageBox::information(nullptr, tr("Cannot dock"), tr("Main window already occupied"));
        }
    }
}

void PathVisualizationWindow::play()
{
    if (_flgPlay) {
        playBtn->setText("Play");
    }
    else {
        playBtn->setText("Stop");
    }
    playBtn->update();
    _flgPlay = !_flgPlay;
    timeSlider->setEnabled(!_flgPlay);

    glWidget->setFlgPlay(_flgPlay);
}

void PathVisualizationWindow::prev()
{
    glWidget->prev();
}

void PathVisualizationWindow::next()
{
    glWidget->next();
}


