#ifndef BULLETUTILS_H
#define BULLETUTILS_H

#include "btBulletDynamicsCommon.h"
#include <Qt3DCore/qentity.h>
#include <Qt3DCore/qtransform.h>

btRigidBody* local_create_rigidBody(btDynamicsWorld* m_ownerWorld, btScalar mass, const btTransform& startTransform, btCollisionShape* shape);
btVector3 rotate_ray_local(const btTransform& localFrame, const btVector3& ray, double angleDegrees);

Qt3DCore::QEntity* create_cuboid(double xExtent, double yExtent, double zExtent, const QColor &bodyColor, Qt3DCore::QTransform* mTransform);

double heading_of_z_rotation(btTransform trans);
bool is_arrowkey(int key);
#endif // BULLETUTILS_H
