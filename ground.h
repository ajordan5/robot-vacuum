
#ifndef GROUND_H
#define GROUND_H
#include <QMatrix4x4>
#include "btBulletDynamicsCommon.h"
#include <Qt3DCore/QEntity>

#include <QColor>

class Ground
{
public:
  Ground();
  Ground(QColor color);
  btRigidBody* getRigidBodyPtr() const {return mRigidBody;}
  Qt3DCore::QEntity* getQEntity() const {return mPlaneEntity;}
  void update_position();

private:
  double mSize{4000};
  btCollisionShape* mGroundShape;
  btDefaultMotionState* mGroundMotionState;
  btRigidBody* mRigidBody;
  btRigidBody::btRigidBodyConstructionInfo* mRigidCI;

  QColor mColor;
  Qt3DCore::QEntity *mPlaneEntity;

  void create_physics_representation();
  void create_graphics_representation();

  void destroy();
};

#endif // CHESSBOARD_H
