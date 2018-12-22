#include "path_finding_client_main_window.h"
#include "path_finding_client_window.h"

#include <QMenuBar>
#include <QMessageBox>
#include <QtWidgets/QDesktopWidget>
#include <QtWidgets/QFileDialog>

PathVisualizationMainWindow::PathVisualizationMainWindow()
{

    auto *menuBar = new QMenuBar;
    QMenu *menuWindow = menuBar->addMenu(tr("&File"));
    auto *addNew = new QAction(menuWindow);
    addNew->setText(tr("OpenPath"));
    menuWindow->addAction(addNew);
    connect(addNew, SIGNAL(triggered()), this, SLOT(onAddNew()));
    setMenuBar(menuBar);

    QDesktopWidget dw;
    setFixedSize((int)(dw.width() * 0.8), (int)(dw.height() * 0.8));

}

void PathVisualizationMainWindow::onAddNew()
{
    if (!centralWidget()) {
        QString fileName = QFileDialog::getOpenFileName(this,
                                                        QString::fromUtf8("Open path"),
                                                        "/opt/tra/path_finding/config/paths/");
                                                       // "/home/alex/Trajectories/src/CppFindPath6D/config_deploy/paths");
        setCentralWidget(new PathVisualizationWindow(this, fileName.toStdString()));
    }
    else
        QMessageBox::information(nullptr, tr("Can't add new window"), tr("Already occupied. Undock first."));

}