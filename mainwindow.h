#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QtCore>
#include <Qt3DExtras/QTorusMesh>
#include <Qt3DExtras/QSphereMesh>
#include <Qt3DCore/qentity.h>
#include <QKeyEvent>
#include <Qt3DExtras/qt3dwindow.h>
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
    void on_actionStart_triggered();
    void setup();

signals:
    void send_map(const uchar* mapImageBuffer);

private:
  Ui::MainWindow *ui;
  void timerEvent(QTimerEvent *);
  void setTimeStep(double ts) {timeStep = ts;}
  void keyPressEvent(QKeyEvent* event);
  void keyReleaseEvent(QKeyEvent* event);

  void setup_3D_world();
  void setup_camera();

  double timeStep{1/60.0};
  World* mWorld;

  Qt3DCore::QEntity *mRootEntity;
  Qt3DExtras::Qt3DWindow* view;
};

#endif // MAINWINDOW_H
