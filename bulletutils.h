#ifndef BULLETUTILS_H
#define BULLETUTILS_H

#include "btBulletDynamicsCommon.h"

btRigidBody* local_create_rigidBody(btDynamicsWorld* m_ownerWorld, btScalar mass, const btTransform& startTransform, btCollisionShape* shape);
btVector3 rotate_ray_local(const btTransform& localFrame, const btVector3& ray, double angleDegrees);
double deg_2_rad(double angleDegrees);
#endif // BULLETUTILS_H
