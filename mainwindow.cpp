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
#include <QDebug>
#include <iostream>


MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    Qt3DExtras::Qt3DWindow *view = new Qt3DExtras::Qt3DWindow();
    view->defaultFrameGraph()->setClearColor(QColor(QRgb(0x4d4d9f)));
    QWidget *container = QWidget::createWindowContainer(view);
    auto layout = new QVBoxLayout();
    layout->addWidget(container);
    ui->frame->setLayout(layout);

    mRootEntity = new Qt3DCore::QEntity();

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

    delete seqImpConstraintSolver;
    delete collisionDispatcher;
    delete defaultCollisionConfig;
    delete broadphaseInterface;
    delete dynamicsWorld;
}

void MainWindow::initPhysics()
{
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
    mGround= new Ground(Qt::white);
    dynamicsWorld->addRigidBody(mGround->getRigidBodyPtr());
    mGround->getQEntity()->setParent(mRootEntity);

    mObstacles = new Obstacles(dynamicsWorld);
    mObstacles->getQEntity()[0]->setParent(mRootEntity);

    btVector3 initPos(10,0,0);
    mVacuum = new Vacuum(dynamicsWorld, initPos);
    mVacuum->getQEntity()[0]->setParent(mRootEntity);
    mVacuum->getQEntity()[1]->setParent(mRootEntity);
    mVacuum->getQEntity()[2]->setParent(mRootEntity);
    mVacuum->getQEntity()[3]->setParent(mRootEntity);


}


void MainWindow::timerEvent(QTimerEvent *)
{
    dynamicsWorld->stepSimulation(timeStep, 10);
    mVacuum->update_position();
}
void MainWindow::on_actionStart_triggered()
{
    startTimer(timeStep * 1000);
}

void MainWindow::keyPressEvent(QKeyEvent *event)
{
    auto key = event->key();
    if(key == Qt::Key_Down || key == Qt::Key_Up || key == Qt::Key_Right || key == Qt::Key_Left) // TODO is_arrowKey
    {
        mVacuum->drive(key);
    }
}

void MainWindow::keyReleaseEvent(QKeyEvent *event)
{
    auto key = event->key();
    if(key == Qt::Key_Down || key == Qt::Key_Up || key == Qt::Key_Right || key == Qt::Key_Left) // TODO is_arrowKey
    {
        mVacuum->stop();
    }
}
