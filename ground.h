//-------------------------------------------------------
// Filename: Ground.h
//
// Description:  The h file for the qt vtk bullet bouncy ball example.
//
// Creator:  Professor Corey McBride for MEEN 570 - Brigham Young University
//
// Creation Date: 11/22/16
//
// Owner: Corey McBride
//-------------------------------------------------------

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
  btRigidBody* getRigidBodyPtr() {return mRigidBody;}
  Qt3DCore::QEntity* getQEntity() {return mPlaneEntity;}
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
