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
#include "ds5w.h"
#include <Windows.h>
#include <iostream>
#include "dualsensedriver.h"


MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{

    ui->setupUi(this);
    setup_3D_world();
    setup_camera();

    mWorld = new World(mRootEntity, timeStep);
    update_camera();

    connect(this, SIGNAL(send_map(LidarMapper*)), ui->mapWidget, SLOT(save_map_pointer(LidarMapper*)));
    connect(ui->startButton, SIGNAL(clicked()), this, SLOT(start_sim()));
    connect(ui->resetButton, SIGNAL(clicked()), this, SLOT(reset_map()));
    emit(send_map(mWorld->get_map()));
}

MainWindow::~MainWindow()
{
    delete ui;

}
void MainWindow::timerEvent(QTimerEvent*)
{
    mWorld->step();
    update_camera();
    ui->mapWidget->update();
    ui->progressBar->setValue(mWorld->get_map()->percent_seen());

    if(driver.is_available())
    {
        VacuumControlState control = driver.get_control();
        mWorld->get_vacuum()->controller_drive(control);
    }


}

void MainWindow::start_sim()
{
    startTimer(timeStep * 1000);
}

void MainWindow::reset_map()
{
    mWorld->get_map()->set_initial_image();
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
    ui->worldFrame->setLayout(layout);

    mRootEntity = new Qt3DCore::QEntity();
    view->setRootEntity(mRootEntity);

}

void MainWindow::setup_camera()
{
    cameraEntity = view->camera();
    cameraEntity->lens()->setPerspectiveProjection(45.0f, 16.0f/9.0f, 0.1f, 1000.0f);
    cameraEntity->setUpVector(QVector3D(0, 0, 1.0f));

    Qt3DCore::QEntity *lightEntity = new Qt3DCore::QEntity(mRootEntity);
    Qt3DRender::QPointLight *light = new Qt3DRender::QPointLight(lightEntity);
    light->setColor("white");
    light->setIntensity(.9f);
    lightEntity->addComponent(light);
    Qt3DCore::QTransform *lightTransform = new Qt3DCore::QTransform(lightEntity);
    lightTransform->setTranslation(QVector3D(200.0f, 200.0f, 200.0f));
    lightEntity->addComponent(lightTransform);
}

void MainWindow::update_camera()
{
    VehicleState currentState = mWorld->get_vacuum()->get_state();
    cameraEntity->setViewCenter(QVector3D(currentState.x, currentState.y, 50));
    double cameraX{currentState.x - 200*cos(currentState.heading)};
    double cameraY{currentState.y - 200*sin(currentState.heading)};
    cameraEntity->setPosition(QVector3D(cameraX, cameraY, 125));

}

void MainWindow::controller()
{
        DualSenseDriver driver;

        while(true)
        {
            VacuumControlState control = driver.get_control();
            qInfo() << control.drive;
        }

}
