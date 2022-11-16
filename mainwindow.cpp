//-------------------------------------------------------
// Filename: mainwindow.cpp
//
// Description:  The cpp file for the qt5 bullet bouncy ball example.
//
// Creator:  Professor Corey McBride for MEEN 570 - Brigham Young University
//
// Creation Date: 11/22/16
//
// Owner: Corey McBride
//-------------------------------------------------------
#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <Qt3DExtras/qt3dwindow.h>
//#include <Qt3DExtras/qfirstpersoncameracontroller.h>
#include <Qt3DRender/qpointlight.h>
#include <Qt3DRender/qcamera.h>
#include <Qt3DCore/qattribute.h>
#include <Qt3DCore/qbuffer.h>
#include <Qt3DCore/qentity.h>
#include <Qt3DRender/qcameralens.h>
#include <Qt3DExtras/qforwardrenderer.h>
#include <Qt3DCore/qtransform.h>
#include <Qt3DExtras/QPhongMaterial>
#include <Qt3DExtras/qorbitcameracontroller.h>


#include <QGridLayout>

MainWindow::MainWindow(QWidget *parent) :
  QMainWindow(parent),
  ui(new Ui::MainWindow)
{
  ui->setupUi(this);

  Qt3DExtras::Qt3DWindow *view = new Qt3DExtras::Qt3DWindow();
  view->defaultFrameGraph()->setClearColor(QColor(QRgb(0x4d4d9f)));
  QWidget *container = QWidget::createWindowContainer(view);
  this->setCentralWidget(container);

  mRootEntity = new Qt3DCore::QEntity();


  // Camera
  Qt3DRender::QCamera *cameraEntity = view->camera();

  cameraEntity->lens()->setPerspectiveProjection(45.0f, 16.0f/9.0f, 0.1f, 1000.0f);
  cameraEntity->setPosition(QVector3D(200.0f, 0, 75));
  cameraEntity->setUpVector(QVector3D(0, 0, 1.0f));
  cameraEntity->setViewCenter(QVector3D(0, 0, 50));

  Qt3DCore::QEntity *lightEntity = new Qt3DCore::QEntity(mRootEntity);
  Qt3DRender::QPointLight *light = new Qt3DRender::QPointLight(lightEntity);
  light->setColor("white");
  light->setIntensity(.9f);
  lightEntity->addComponent(light);
  Qt3DCore::QTransform *lightTransform = new Qt3DCore::QTransform(lightEntity);
  lightTransform->setTranslation(QVector3D(200.0f, 200.0f, 200.0f));
  lightEntity->addComponent(lightTransform);

  Qt3DExtras::QOrbitCameraController *camController = new Qt3DExtras::QOrbitCameraController(mRootEntity);
  camController->setLinearSpeed( 50.0f );
  camController->setLookSpeed( 180.0f );
  camController->setCamera(cameraEntity);


  timeStep = 1/60.0;


  initPhysics();
  createWorld();
  view->setRootEntity(mRootEntity);


  QTimer::singleShot(1000, this, SLOT(setup()));
}
void MainWindow::setup()
{


}

MainWindow::~MainWindow()
{
  delete ui;

  dynamicsWorld->removeRigidBody(mGround->getRigidBodyPtr());
  delete mGround;

//  for (int i = 0; i < bouncyBalls.size(); i++)
//  {
//    dynamicsWorld->removeRigidBody(bouncyBalls[i]->getRigidBodyPtr());
//    delete bouncyBalls[i];
//  }

  delete seqImpConstraintSolver;
  delete collisionDispatcher;
  delete defaultCollisionConfig;
  delete broadphaseInterface;
  delete dynamicsWorld;
}

void MainWindow::initPhysics()
{
  // The BulletWidget owns and controls everything to do with
  // the dynamics world. This call allocates the solvers
  // and collision objects, and sets the gravity.
  broadphaseInterface = new btDbvtBroadphase();
  defaultCollisionConfig = new btDefaultCollisionConfiguration();
  collisionDispatcher = new btCollisionDispatcher(defaultCollisionConfig);
  seqImpConstraintSolver = new btSequentialImpulseConstraintSolver;
  dynamicsWorld = new btDiscreteDynamicsWorld(collisionDispatcher, broadphaseInterface,
                                              seqImpConstraintSolver, defaultCollisionConfig);

  dynamicsWorld->setGravity(btVector3(0, 0, -1000));
}
void MainWindow::createWorld()
{

  // This creates and adds the ground to the world.
  mGround= new Ground(Qt::white);
  dynamicsWorld->addRigidBody(mGround->getRigidBodyPtr());
  mGround->getQEntity()->setParent(mRootEntity);

  btVector3 initPos(0,0,0);
  mVacuum = new Vacuum(dynamicsWorld, initPos);
  mVacuum->getQEntity()->setParent(mRootEntity);

//  // Here is where all the spheres are created.
//  BouncyBall *ball = new BouncyBall({0,0.0,100}, Qt::red, 100, 10);
//  ball->getQEntity()->setParent(mRootEntity);
//  bouncyBalls.push_back(ball);

//  ball = new BouncyBall({0,0,50}, Qt::blue, 50, 10);
//  ball->getQEntity()->setParent(mRootEntity);
//  bouncyBalls.push_back(ball);


//  // Here, we ask each ball for its btRigidBody*,
//  // which is added into the world, free to interact with
//  // everything else in the world.
//  for (BouncyBall* ball: bouncyBalls)
//  {
//    dynamicsWorld->addRigidBody(ball->getRigidBodyPtr());
//  }


}


void MainWindow::timerEvent(QTimerEvent *)
{
  // This call updates the world; all the rigid
  // bodies in the world update their translations
  // and rotations.

  dynamicsWorld->stepSimulation(timeStep, 10);

//  for (BouncyBall* ball: bouncyBalls)
//  {
//    ball->update_position();
//  }



}
void MainWindow::on_actionStart_triggered()
{
  // And, start the timer.
  startTimer(timeStep * 1000);
}
