#define _USE_MATH_DEFINES

#include "bulletutils.h"

btRigidBody* local_create_rigidBody(btDynamicsWorld* m_ownerWorld, btScalar mass, const btTransform& startTransform, btCollisionShape* shape)
{
    btVector3 localInertia(1, 1, 1);
    shape->calculateLocalInertia(mass, localInertia);

    btDefaultMotionState* myMotionState = new btDefaultMotionState(startTransform);
    btRigidBody::btRigidBodyConstructionInfo rbInfo(mass, myMotionState, shape, localInertia);
    btRigidBody* body = new btRigidBody(rbInfo);

    m_ownerWorld->addRigidBody(body);

    return body;
}

btVector3 rotate_ray_local(const btTransform& localFrame, const btVector3& ray, double angleDegrees)
{
    double angleRadians = deg_2_rad(angleDegrees);
    btVector3 rotated = ray.rotate({0,0,1}, angleRadians);
    return localFrame*rotated;
}

double deg_2_rad(double angleDegrees)
{
    return angleDegrees*M_PI/180;
}
