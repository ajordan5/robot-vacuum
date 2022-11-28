#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QtCore>
#include <Qt3DExtras/QTorusMesh>
#include <Qt3DExtras/QSphereMesh>
#include <Qt3DCore/qentity.h>
#include <QKeyEvent>
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

private:
  Ui::MainWindow *ui;
  void timerEvent(QTimerEvent *);
  void setTimeStep(double ts) {timeStep = ts;}
  void keyPressEvent(QKeyEvent* event);
  void keyReleaseEvent(QKeyEvent* event);

//  void setup_3D_world();
//  void setup_camera();

  double timeStep;
  World* mWorld;

  Qt3DCore::QEntity *mRootEntity;

};

#endif // MAINWINDOW_H
