#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QtCore>
#include "btBulletDynamicsCommon.h"
#include "ground.h"
#include "vacuum.h"
#include "obstacles.h"
#include <Qt3DExtras/QTorusMesh>
#include <Qt3DExtras/QSphereMesh>
#include <Qt3DCore/qentity.h>

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

  btBroadphaseInterface* broadphaseInterface;
  btDefaultCollisionConfiguration* defaultCollisionConfig;
  btCollisionDispatcher* collisionDispatcher;
  btSequentialImpulseConstraintSolver* seqImpConstraintSolver;
  btDiscreteDynamicsWorld* dynamicsWorld;

  Ground* mGround;
  Vacuum* mVacuum;
  Obstacles* mObstacles;
  double timeStep;

  void initPhysics();
  void createWorld();

  Qt3DExtras::QSphereMesh *mSphere;
  Qt3DCore::QEntity *mSphereEntity;
  Qt3DCore::QEntity *mRootEntity;

};

#endif // MAINWINDOW_H
