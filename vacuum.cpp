#include "vacuum.h"

#include <Qt3DExtras/QPhongMaterial>
#include <Qt3DCore/qentity.h>
#include <Qt3DCore/qtransform.h>
#include <Qt3DExtras/QCylinderMesh>
#include <QColor>

Vacuum::Vacuum(btDynamicsWorld* ownerWorld, btVector3 initalPosition)
    : m_ownerWorld{ownerWorld }, mResetPosition{initalPosition.getX(),initalPosition.getY(),initalPosition.getZ()}
{
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
    btVector3 pivotA{0, 0, btScalar(-bodyRadius)};
    btVector3 pivotB{0., 0., 0.};
    btVector3 axisA{0., 0., -1.};
    btVector3 axisB{0., 1., 0.};
    hingeC = new btHingeConstraint(*m_bodies[0], *m_bodies[1], pivotA, pivotB, axisA, axisB);
    hingeC->enableAngularMotor(true, 10, 5);
    m_joints[0] = hingeC;
    m_ownerWorld->addConstraint(m_joints[0], true);

    pivotA = {0, 0, btScalar(bodyRadius)};
    axisA = {0, 0, 1};
    axisB = {0, -1, 0};
    hingeC = new btHingeConstraint(*m_bodies[0], *m_bodies[2], pivotA, pivotB, axisA, axisB);
    hingeC->enableAngularMotor(true, -10, 5);
    m_joints[1] = hingeC;
    m_ownerWorld->addConstraint(m_joints[1], true);

}

void Vacuum::setup_graphics()
{
    Qt3DExtras::QCylinderMesh* bodyMesh = new Qt3DExtras::QCylinderMesh();
    bodyMesh->setRadius(5);
    bodyMesh->setLength(4);
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
    wheelMeshRight->setRadius(2);
    wheelMeshRight->setLength(2);
    QVector3D wheelPosition(0.0, wheelRadius, bodyRadius+halfWheelThickness);
    mTransforms[1] = new Qt3DCore::QTransform();
    mTransforms[1]->setTranslation(wheelPosition);

    mCylinderEntities[1] = new Qt3DCore::QEntity();
    mCylinderEntities[1]->addComponent(wheelMeshRight);
    mCylinderEntities[1]->addComponent(material);
    mCylinderEntities[1]->addComponent(mTransforms[1]);
    mCylinderEntities[1]->setEnabled(true);

    Qt3DExtras::QCylinderMesh* wheelMeshLeft = new Qt3DExtras::QCylinderMesh();
    wheelMeshLeft->setRadius(2);
    wheelMeshLeft->setLength(2);
    QVector3D wheelPositionLeft(0.0, wheelRadius, -bodyRadius-halfWheelThickness);
    mTransforms[2] = new Qt3DCore::QTransform();
    mTransforms[2]->setTranslation(wheelPositionLeft);

    mCylinderEntities[2] = new Qt3DCore::QEntity();
    mCylinderEntities[2]->addComponent(wheelMeshLeft);
    mCylinderEntities[2]->addComponent(material);
    mCylinderEntities[2]->addComponent(mTransforms[2]);
    mCylinderEntities[2]->setEnabled(true);

    update_position();

}

void Vacuum::update_position()
{
    for (int i = 0; i < 3; i++)
    {
        m_bodies[i]->getMotionState()->getWorldTransform(trans);
        trans.getOpenGLMatrix(btMat);

        QMatrix4x4 trans(btMat);
        trans=trans.transposed();//Converting between opengl column major to Qt row major
        mTransforms[i]->setMatrix(trans);
    }
}



