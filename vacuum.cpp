#define KEY_UP 16777235
#define KEY_LEFT 16777234
#define KEY_DOWN 16777237
#define KEY_RIGHT 16777236

#include "vacuum.h"

#include <Qt3DExtras/QPhongMaterial>
#include <Qt3DCore/qentity.h>
#include <Qt3DCore/qtransform.h>
#include <Qt3DExtras/QCylinderMesh>
#include <Qt3DExtras/QCuboidMesh>
#include <QColor>

Vacuum::Vacuum(btDynamicsWorld* ownerWorld, btVector3 initalPosition)
    : m_ownerWorld{ownerWorld }, mResetPosition{initalPosition.getX(),initalPosition.getY(),initalPosition.getZ()}
{
    lidar = new Lidar(m_ownerWorld, lidarRange, 180, 5);
    setup_physics(initalPosition);
    setup_graphics();


}

void Vacuum::setup_physics(const btVector3& initalPosition)
{
    btVector3 vacuumBody(bodyRadius, halfBodyThickness, bodyRadius);
    m_shapes[0] = new btCylinderShape(vacuumBody);

    btVector3 wheelBody(wheelRadius, halfWheelThickness, wheelRadius);
    m_shapes[1] = new btCylinderShape(wheelBody);
    m_shapes[2] = new btCylinderShape(wheelBody);

    btVector3 lidarBody(1, 1, 1);
    m_shapes[3] = new btBoxShape(lidarBody);

    btTransform offset;
    offset.setIdentity();
    offset.setOrigin(initalPosition);

    btVector3 vRoot = btVector3(btScalar(0.), btScalar(0.), btScalar(wheelRadius));
    btTransform transform;
    transform.setIdentity();
    transform.setOrigin(vRoot);
    btVector3 axis(1, 0, 0);
    transform.setRotation(btQuaternion(axis, M_PI_2));
    m_bodies[0] = local_create_rigidBody(m_ownerWorld, btScalar(1.), offset * transform, m_shapes[0]);
    m_bodies[0]->setAngularFactor(btVector3(0, 0, 1));

    transform.setIdentity();
    btVector3 vLidarOrigin = btVector3(btScalar(0.0), btScalar(0.0), btScalar(1+halfBodyThickness+wheelRadius));
    transform.setOrigin(vLidarOrigin);
    m_bodies[3] = local_create_rigidBody(m_ownerWorld, btScalar(.0001), offset*transform, m_shapes[3]);

    transform.setIdentity();
    btVector3 vWheelOrigin = btVector3(btScalar(0.0), btScalar(bodyRadius+halfWheelThickness), btScalar(wheelRadius));
    transform.setOrigin(vWheelOrigin);
    m_bodies[1] = local_create_rigidBody(m_ownerWorld, btScalar(1.), offset * transform, m_shapes[1]);

    transform.setIdentity();
    vWheelOrigin = btVector3(btScalar(0.0), btScalar(-bodyRadius-halfWheelThickness), btScalar(wheelRadius));
    transform.setOrigin(vWheelOrigin);
    m_bodies[2] = local_create_rigidBody(m_ownerWorld, btScalar(1.), offset * transform, m_shapes[2]);

    for (int i = 0; i < 3; ++i)
    {
        m_bodies[i]->setDamping(0.05, 0.85);
        m_bodies[i]->setDeactivationTime(0.8);
        m_bodies[i]->setSleepingThresholds(0.5f, 0.5f);
    }

    btHingeConstraint* hingeC;
    btVector3 pivotA{0, 0, btScalar(-bodyRadius-halfWheelThickness)};
    btVector3 pivotB{0., 0., 0.};
    btVector3 axisA{0., 0., -1.};
    btVector3 axisB{0., 1., 0.};
    hingeC = new btHingeConstraint(*m_bodies[0], *m_bodies[1], pivotA, pivotB, axisA, axisB);

    m_joints[0] = hingeC;
    m_ownerWorld->addConstraint(m_joints[0], true);

    pivotA = {0, 0, btScalar(bodyRadius+halfWheelThickness)};
    axisA = {0, 0, 1};
    axisB = {0, -1, 0};
    hingeC = new btHingeConstraint(*m_bodies[0], *m_bodies[2], pivotA, pivotB, axisA, axisB);

    m_joints[1] = hingeC;
    m_ownerWorld->addConstraint(m_joints[1], true);

    m_joints[0]->enableAngularMotor(true, 0, 5);
    m_joints[1]->enableAngularMotor(true, 0, 5);

    pivotA = {0, btScalar(halfBodyThickness+1), 0};
    axisA = {0, 1, 0};
    axisB = {0, 0, 1};
    hingeC = new btHingeConstraint(*m_bodies[0], *m_bodies[3], pivotA, pivotB, axisA, axisB);
    hingeC->setLimit(0, 0);
    m_ownerWorld->addConstraint(hingeC, true);

    stop();

}

void Vacuum::setup_graphics()
{
    Qt3DExtras::QCylinderMesh* bodyMesh = new Qt3DExtras::QCylinderMesh();
    bodyMesh->setRadius(bodyRadius);
    bodyMesh->setLength(halfBodyThickness*2);
    mTransforms[0] = new Qt3DCore::QTransform();
    mTransforms[0]->setTranslation(mResetPosition);

    Qt3DExtras::QPhongMaterial *material = new Qt3DExtras::QPhongMaterial();
    material->setDiffuse(Qt::red);

    mCylinderEntities[0] = new Qt3DCore::QEntity();
    mCylinderEntities[0]->addComponent(bodyMesh);
    mCylinderEntities[0]->addComponent(material);
    mCylinderEntities[0]->addComponent(mTransforms[0]);
    mCylinderEntities[0]->setEnabled(true);

    Qt3DExtras::QCylinderMesh* wheelMeshRight = new Qt3DExtras::QCylinderMesh();
    wheelMeshRight->setRadius(wheelRadius);
    wheelMeshRight->setLength(halfWheelThickness*2);
    QVector3D wheelPosition(0.0, wheelRadius, bodyRadius+halfWheelThickness);
    mTransforms[1] = new Qt3DCore::QTransform();
    mTransforms[1]->setTranslation(wheelPosition);

    Qt3DExtras::QPhongMaterial *wheelMaterial = new Qt3DExtras::QPhongMaterial();
    wheelMaterial->setDiffuse(Qt::black);

    mCylinderEntities[1] = new Qt3DCore::QEntity();
    mCylinderEntities[1]->addComponent(wheelMeshRight);
    mCylinderEntities[1]->addComponent(wheelMaterial);
    mCylinderEntities[1]->addComponent(mTransforms[1]);
    mCylinderEntities[1]->setEnabled(true);

    Qt3DExtras::QCylinderMesh* wheelMeshLeft = new Qt3DExtras::QCylinderMesh();
    wheelMeshLeft->setRadius(wheelRadius);
    wheelMeshLeft->setLength(halfWheelThickness*2);
    QVector3D wheelPositionLeft(0.0, wheelRadius, -bodyRadius-halfWheelThickness);
    mTransforms[2] = new Qt3DCore::QTransform();
    mTransforms[2]->setTranslation(wheelPositionLeft);

    mCylinderEntities[2] = new Qt3DCore::QEntity();
    mCylinderEntities[2]->addComponent(wheelMeshLeft);
    mCylinderEntities[2]->addComponent(wheelMaterial);
    mCylinderEntities[2]->addComponent(mTransforms[2]);
    mCylinderEntities[2]->setEnabled(true);

    Qt3DExtras::QCuboidMesh* lidarMesh = new Qt3DExtras::QCuboidMesh();
    lidarMesh->setXExtent(2);
    lidarMesh->setYExtent(2);
    lidarMesh->setZExtent(2);
    mTransforms[3] = new Qt3DCore::QTransform();

    mCylinderEntities[3] = new Qt3DCore::QEntity();
    mCylinderEntities[3]->addComponent(lidarMesh);
    mCylinderEntities[3]->addComponent(wheelMaterial);
    mCylinderEntities[3]->addComponent(mTransforms[3]);
    mCylinderEntities[3]->setEnabled(true);

    update_position();

}

void Vacuum::update_position()
{
    for (int i = 0; i < 4; i++)
    {
        m_bodies[i]->getMotionState()->getWorldTransform(trans);
        trans.getOpenGLMatrix(btMat);

        QMatrix4x4 trans(btMat);
        trans=trans.transposed();
        mTransforms[i]->setMatrix(trans);
    }

}

std::pair<Eigen::VectorXd, Eigen::VectorXd> Vacuum::update_measurements()
{
    m_bodies[3]->getMotionState()->getWorldTransform(trans);
    return lidar->get_intersections_angle_and_distance(trans);
}

double Vacuum::get_heading()
{
    m_bodies[3]->getMotionState()->getWorldTransform(trans);
    return heading_of_z_rotation(trans);
}

VehicleState Vacuum::get_state()
{
    btVector3 position = trans.getOrigin();
    state.x = position[0];
    state.y = position[1];
    state.heading = get_heading();

    return state;
}

void Vacuum::drive(int key)
{
    switch(key)
    {
    case Qt::Key::Key_Down:
        m_joints[0]->enableAngularMotor(true, 15, 50);
        m_joints[1]->enableAngularMotor(true, -15, 50);
        break;
    case Qt::Key::Key_Up:
        m_joints[0]->enableAngularMotor(true, -15, 50);
        m_joints[1]->enableAngularMotor(true, 15, 50);
        break;
    case Qt::Key::Key_Left:
        m_joints[0]->enableAngularMotor(true, 5, 50);
        m_joints[1]->enableAngularMotor(true, 5, 50);
        break;
    case Qt::Key::Key_Right:
        m_joints[0]->enableAngularMotor(true, -5, 50);
        m_joints[1]->enableAngularMotor(true, -5, 50);
        break;
    }

}

void Vacuum::stop()
{
    m_joints[0]->enableAngularMotor(true, 0, 5);
    m_joints[1]->enableAngularMotor(true, 0, 5);
}



