#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QtCore>
#include <Qt3DCore/qentity.h>
#include <QKeyEvent>
#include <Qt3DExtras/qt3dwindow.h>
#include "dualsensedriver.h"
#include "world.h"

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

private slots:
    void start_sim();
    void reset_map();

signals:
    void send_map(LidarMapper* map);

private:
    Ui::MainWindow *ui;
    Qt3DRender::QCamera* cameraEntity;

    void timerEvent(QTimerEvent *);
    void setTimeStep(double ts) {timeStep = ts;}
    void keyPressEvent(QKeyEvent* event);
    void keyReleaseEvent(QKeyEvent* event);
    void setup_3D_world();
    void setup_camera();
    void update_camera();

    double timeStep{1/60.0};
    World* mWorld;
    DualSenseDriver controllerDriver;

    Qt3DCore::QEntity *mRootEntity;
    Qt3DExtras::Qt3DWindow* view;
};

#endif // MAINWINDOW_H
