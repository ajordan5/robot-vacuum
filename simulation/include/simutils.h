#ifndef BULLETUTILS_H
#define BULLETUTILS_H

#include "btBulletDynamicsCommon.h"
#include <Qt3DCore/qentity.h>
#include <Qt3DCore/qtransform.h>

btRigidBody* local_create_rigidBody(btDynamicsWorld* m_ownerWorld, btScalar mass, const btTransform& startTransform, btCollisionShape* shape);
btVector3 rotate_ray_local(const btTransform& localFrame, const btVector3& ray, double angleDegrees);

Qt3DCore::QEntity* create_cuboid_graphics(double xExtent, double yExtent, double zExtent, const QColor &bodyColor, Qt3DCore::QTransform* mTransform);
Qt3DCore::QEntity* create_cylinder_graphics(double bodyRadius, double bodyThickness, const QColor &bodyColor, Qt3DCore::QTransform* mTransform);

double get_heading_of_z_rot_matrix(btTransform homogeneousTrans);
bool is_arrowkey(int key);

#endif // BULLETUTILS_H
