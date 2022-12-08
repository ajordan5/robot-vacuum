#include "vacuum.h"
#include "simutils.h"

#include <Qt3DExtras/QPhongMaterial>
#include <Qt3DCore/qentity.h>
#include <Qt3DCore/qtransform.h>
#include <Qt3DExtras/QCylinderMesh>
#include <Qt3DExtras/QCuboidMesh>
#include <QColor>

Vacuum::Vacuum(btDynamicsWorld* ownerWorld, btVector3 initalPosition)
    : m_ownerWorld{ownerWorld }, mResetPosition{initalPosition.getX(),initalPosition.getY(),initalPosition.getZ()}
{
    lidar = new Lidar(m_ownerWorld, lidarRange, lidarFieldOfView, lidarNumRays);
    setup_physics(initalPosition);
    setup_graphics();


}

void Vacuum::setup_physics(const btVector3& initalPosition)
{
    btTransform offset;
    offset.setIdentity();
    offset.setOrigin(initalPosition);

    setup_vacuum_body(offset);
    setup_wheels(offset);
    setup_constraints();

    brake_wheels();

}

void Vacuum::setup_wheels(const btTransform& offset)
{
    btVector3 wheelBody(wheelRadius, halfWheelThickness, wheelRadius);
    m_shapes[1] = new btCylinderShape(wheelBody);
    m_shapes[2] = new btCylinderShape(wheelBody);

    btTransform transform;
    transform.setIdentity();
    btVector3 vWheelOrigin = btVector3(btScalar(0.0), btScalar(bodyRadius+halfWheelThickness), btScalar(wheelRadius));
    transform.setOrigin(vWheelOrigin);
    m_bodies[1] = local_create_rigidBody(m_ownerWorld, btScalar(1.), offset * transform, m_shapes[1]);

    transform.setIdentity();
    vWheelOrigin = btVector3(btScalar(0.0), btScalar(-bodyRadius-halfWheelThickness), btScalar(wheelRadius));
    transform.setOrigin(vWheelOrigin);
    m_bodies[2] = local_create_rigidBody(m_ownerWorld, btScalar(1.), offset * transform, m_shapes[2]);
}

void Vacuum::setup_vacuum_body(const btTransform& offset)
{
    btVector3 vacuumBody(bodyRadius, halfBodyThickness, bodyRadius);
    m_shapes[0] = new btCylinderShape(vacuumBody);

    btVector3 vRoot = btVector3(btScalar(0.), btScalar(0.), btScalar(wheelRadius));
    btTransform transform;
    transform.setIdentity();
    transform.setOrigin(vRoot);
    btVector3 axis(1, 0, 0);
    transform.setRotation(btQuaternion(axis, M_PI_2));
    m_bodies[0] = local_create_rigidBody(m_ownerWorld, btScalar(1.), offset * transform, m_shapes[0]);
    m_bodies[0]->setAngularFactor(btVector3(0, 0, 1));

    btVector3 lidarBody(1, 1, 1);
    m_shapes[3] = new btBoxShape(lidarBody);

    transform.setIdentity();
    btVector3 vLidarOrigin = btVector3(btScalar(0.0), btScalar(0.0), btScalar(1+halfBodyThickness+wheelRadius));
    transform.setOrigin(vLidarOrigin);
    m_bodies[3] = local_create_rigidBody(m_ownerWorld, btScalar(.0001), offset*transform, m_shapes[3]);
}

void Vacuum::setup_constraints()
{
    for (int i = 0; i < 3; ++i)
    {
        m_bodies[i]->setDamping(0.05, 0.85);
        m_bodies[i]->setDeactivationTime(0.8);
        m_bodies[i]->setSleepingThresholds(0.5f, 0.5f);
    }

    btHingeConstraint* hinge;
    btVector3 pivotA{0, 0, btScalar(-bodyRadius-halfWheelThickness)};
    btVector3 pivotB{0., 0., 0.};
    btVector3 axisA{0., 0., -1.};
    btVector3 axisB{0., 1., 0.};
    hinge = new btHingeConstraint(*m_bodies[0], *m_bodies[1], pivotA, pivotB, axisA, axisB);

    m_joints[0] = hinge;
    m_ownerWorld->addConstraint(m_joints[0], true);

    pivotA = {0, 0, btScalar(bodyRadius+halfWheelThickness)};
    axisA = {0, 0, 1};
    axisB = {0, -1, 0};
    hinge = new btHingeConstraint(*m_bodies[0], *m_bodies[2], pivotA, pivotB, axisA, axisB);

    m_joints[1] = hinge;
    m_ownerWorld->addConstraint(m_joints[1], true);

    m_joints[0]->enableAngularMotor(true, 0, 5);
    m_joints[1]->enableAngularMotor(true, 0, 5);

    pivotA = {0, btScalar(halfBodyThickness+1), 0};
    axisA = {0, 1, 0};
    axisB = {0, 0, 1};
    hinge = new btHingeConstraint(*m_bodies[0], *m_bodies[3], pivotA, pivotB, axisA, axisB);
    hinge->setLimit(0, 0);
    m_ownerWorld->addConstraint(hinge, true);
}

void Vacuum::setup_graphics()
{
    QColor bodyColor{Qt::red};
    mTransforms[0] = new Qt3DCore::QTransform();
    mTransforms[0]->setTranslation(mResetPosition);
    mEntities[0] = create_cylinder_graphics(bodyRadius, 2*halfBodyThickness, bodyColor, mTransforms[0]);

    QColor wheelColor{Qt::black};
    mTransforms[1] = new Qt3DCore::QTransform();
    mEntities[1] = create_cylinder_graphics(wheelRadius, 2*halfWheelThickness, wheelColor, mTransforms[1]);

    mTransforms[2] = new Qt3DCore::QTransform();
    mEntities[2] = create_cylinder_graphics(wheelRadius, 2*halfWheelThickness, wheelColor, mTransforms[2]);

    mTransforms[3] = new Qt3DCore::QTransform();
    mEntities[3] = create_cuboid_graphics(2, 2, 2, wheelColor, mTransforms[3]);

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
    return get_heading_of_z_rot_matrix(trans);
}

VehicleState Vacuum::get_state()
{
    btVector3 position = trans.getOrigin();
    state.x = position[0];
    state.y = position[1];
    state.heading = get_heading();

    return state;
}

void Vacuum::arrowkey_drive(int key)
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

void Vacuum::controller_drive(const VacuumControlState& control)
{
    double rightMotor = control.drive * 15 * (1+control.turbo*5);
    double leftMotor = control.drive * -15 * (1+control.turbo*5);

    if (control.turn > 0)
        leftMotor = leftMotor - 7*control.turn;
    else if (control.turn < 0)
        rightMotor = rightMotor - 7*control.turn;

    m_joints[0]->enableAngularMotor(true, leftMotor, 50);
    m_joints[1]->enableAngularMotor(true, rightMotor, 50);
}

void Vacuum::brake_wheels()
{
    m_joints[0]->enableAngularMotor(true, 0, 5);
    m_joints[1]->enableAngularMotor(true, 0, 5);
}



