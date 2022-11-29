#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <Qt3DExtras/qt3dwindow.h>
#include <Qt3DRender/qpointlight.h>
#include <Qt3DRender/qcamera.h>
#include <Qt3DCore/qattribute.h>
#include <Qt3DCore/qbuffer.h>
#include <Qt3DCore/qentity.h>
#include <Qt3DRender/qcameralens.h>
#include <Qt3DExtras/qforwardrenderer.h>
#include <Qt3DCore/qtransform.h>
#include <Qt3DExtras/QPhongMaterial>
#include <QGridLayout>
#include <QDebug>
#include <iostream>


MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    setup_3D_world();
    setup_camera();

    mWorld = new World(mRootEntity, timeStep);

    QTimer::singleShot(1000, this, SLOT(setup()));
}

MainWindow::~MainWindow()
{
    delete ui;

}

void MainWindow::setup()
{

}

void MainWindow::timerEvent(QTimerEvent *)
{
    mWorld->step();
}

void MainWindow::on_actionStart_triggered()
{
    startTimer(timeStep * 1000);
}

void MainWindow::keyPressEvent(QKeyEvent *event)
{
    auto key = event->key();
    if(is_arrowkey(key))
    {
        mWorld->key_press(key);
    }
}

void MainWindow::keyReleaseEvent(QKeyEvent *event)
{
    auto key = event->key();
    if(is_arrowkey(key))
    {
        mWorld->key_release();
    }
}

void MainWindow::setup_3D_world()
{
    view = new Qt3DExtras::Qt3DWindow();
    view->defaultFrameGraph()->setClearColor(QColor(QRgb(0x4d4d9f)));
    QWidget *container = QWidget::createWindowContainer(view);
    auto layout = new QVBoxLayout();
    layout->addWidget(container);
    ui->frame->setLayout(layout);

    mRootEntity = new Qt3DCore::QEntity();
    view->setRootEntity(mRootEntity);

}

void MainWindow::setup_camera()
{
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
}
