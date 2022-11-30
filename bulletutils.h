#ifndef BULLETUTILS_H
#define BULLETUTILS_H

#include "btBulletDynamicsCommon.h"

btRigidBody* local_create_rigidBody(btDynamicsWorld* m_ownerWorld, btScalar mass, const btTransform& startTransform, btCollisionShape* shape);
btVector3 rotate_ray_local(const btTransform& localFrame, const btVector3& ray, double angleDegrees);
double heading_of_z_rotation(btTransform trans);
bool is_arrowkey(int key);
#endif // BULLETUTILS_H
