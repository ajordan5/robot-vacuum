#ifndef VACUUM_H
#define VACUUM_H

#include "btBulletDynamicsCommon.h"
#include <QMatrix4x4>
#include <Qt3DCore/QEntity>
#include <Qt3DCore/QTransform>
#include <QColor>

#include "simutils.h"
#include "lidar.h"
#include "lidarmapper.h"
#include "dualsensedriver.h"

class Vacuum
{
public:
    Vacuum(btDynamicsWorld* ownerWorld, btVector3 initalPosition);
    btRigidBody** get_vehicle_body() {return m_bodies;}
    Qt3DCore::QEntity** getQEntity() {return mEntities;}
    double get_lidar_range() const {return lidarRange;}
    double get_heading() ;
    VehicleState get_state();

    void update_position();
    std::pair<Eigen::VectorXd, Eigen::VectorXd> update_measurements();    
    void arrowkey_drive(int key);
    void controller_drive(const VacuumControlState& control);
    void brake_wheels();

private:
    btDynamicsWorld* m_ownerWorld;
    btCollisionShape* m_shapes[4];
    btRigidBody* m_bodies[4];
    btHingeConstraint* m_joints[2];
    btScalar btMat[16];
    btTransform trans;
    QVector3D mResetPosition;
    Qt3DCore::QTransform *mTransforms[4];
    Qt3DCore::QEntity *mEntities[4];

    double bodyRadius{5};
    double halfBodyThickness{2};
    double wheelRadius{2.4};
    double halfWheelThickness{0.5};
    VehicleState state;

    void setup_physics(const btVector3& initalPosition);
    void setup_graphics();
    void setup_vacuum_body(const btTransform& offset);
    void setup_wheels(const btTransform& offset);
    void setup_constraints();

    Lidar* lidar;
    double lidarRange{60};
    double lidarFieldOfView{160};
    double lidarNumRays{10};


};


#endif // VACUUM_H
