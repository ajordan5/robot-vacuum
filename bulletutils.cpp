#define _USE_MATH_DEFINES

#include "bulletutils.h"
#include "occupancygridutils.h"
#include <QtCore>

btRigidBody* local_create_rigidBody(btDynamicsWorld* m_ownerWorld, btScalar mass, const btTransform& startTransform, btCollisionShape* shape)
{
    btVector3 localInertia(1, 1, 1);
    shape->calculateLocalInertia(mass, localInertia);

    btDefaultMotionState* myMotionState = new btDefaultMotionState(startTransform);
    btRigidBody::btRigidBodyConstructionInfo rbInfo(mass, myMotionState, shape, localInertia);
    btRigidBody* body = new btRigidBody(rbInfo);
    body->setFriction(1.0);

    m_ownerWorld->addRigidBody(body);

    return body;
}

btVector3 rotate_ray_local(const btTransform& localFrame, const btVector3& ray, double angleDegrees)
{
    double angleRadians = deg_2_rad(angleDegrees);
    btVector3 rotated = ray.rotate({0,0,1}, angleRadians);
    return localFrame*rotated;
}

double heading_of_z_rotation(btTransform trans)
{
    btMatrix3x3 rotation{trans.getBasis()};
    double sinTh{rotation[1][0]};
    double cosTh{rotation[1][1]};
    return atan2(sinTh, cosTh);
}

bool is_arrowkey(int key)
{
    return (key == Qt::Key_Down || key == Qt::Key_Up || key == Qt::Key_Right || key == Qt::Key_Left);
}
