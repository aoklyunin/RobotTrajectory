#pragma  once

#include "path_finding_client_widget.h"

#include <QWidget>


QT_BEGIN_NAMESPACE
class QSlider;
class QPushButton;
QT_END_NAMESPACE

class PathVisualisationWidget;
class PathVisualizationMainWindow;

class PathVisualizationWindow : public QWidget
{
Q_OBJECT

public:
    PathVisualizationWindow(PathVisualizationMainWindow *mw,std::string scenePath);

protected:
    void keyPressEvent(QKeyEvent *event) Q_DECL_OVERRIDE;

private slots:
    void dockUndock();
    void next();
    void prev();
    void play();

private:
    QSlider *createSlider();

    PathVisualisationWidget *glWidget;
    QSlider *timeSlider;

    QPushButton *dockBtn;
    QPushButton *nextBtn;
    QPushButton *prevBtn;
    QPushButton *playBtn;

    PathVisualizationMainWindow *mainWindow;

    bool _flgPlay;
};
