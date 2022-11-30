#include "world.h"

World::World(Qt3DCore::QEntity* rootEntity, double simTimeStep)
    : mRootEntity{rootEntity}, timeStep{simTimeStep}
{
    init_physics();
    create_world();
    map = new LidarMapper(mVacuum->get_lidar_range(), 1000, 1000, 1);
}

World::~World()
{
    dynamicsWorld->removeRigidBody(mGround->getRigidBodyPtr());
    delete mGround;

//    dynamicsWorld->removeRigidBody(mVacuum->getRigidBodyPtr());
    delete mVacuum;

//    dynamicsWorld->removeRigidBody(mObstacles->getRigidBodyPtr());
    delete mObstacles;


    delete seqImpConstraintSolver;
    delete collisionDispatcher;
    delete defaultCollisionConfig;
    delete broadphaseInterface;
    delete dynamicsWorld;
}

void World::init_physics()
{
    broadphaseInterface = new btDbvtBroadphase();
    defaultCollisionConfig = new btDefaultCollisionConfiguration();
    collisionDispatcher = new btCollisionDispatcher(defaultCollisionConfig);
    seqImpConstraintSolver = new btSequentialImpulseConstraintSolver;
    dynamicsWorld = new btDiscreteDynamicsWorld(collisionDispatcher, broadphaseInterface,
                                                seqImpConstraintSolver, defaultCollisionConfig);

    dynamicsWorld->setGravity(btVector3(0, 0, -1000));
}

void World::create_world()
{
    mGround= new Ground(Qt::white);
    dynamicsWorld->addRigidBody(mGround->getRigidBodyPtr());
    mGround->getQEntity()->setParent(mRootEntity);

    mObstacles = new Obstacles(dynamicsWorld);
    mObstacles->getQEntity()[0]->setParent(mRootEntity);

    btVector3 initPos(10,0,0);
    mVacuum = new Vacuum(dynamicsWorld, initPos);
    mVacuum->getQEntity()[0]->setParent(mRootEntity);
    mVacuum->getQEntity()[1]->setParent(mRootEntity);
    mVacuum->getQEntity()[2]->setParent(mRootEntity);
    mVacuum->getQEntity()[3]->setParent(mRootEntity);

}

void World::step()
{
    dynamicsWorld->stepSimulation(timeStep, 10);
    mVacuum->update_position();
    pass_measurements_to_map();

}

void World::pass_measurements_to_map()
{
    std::pair<Eigen::VectorXd, Eigen::VectorXd> measurements = mVacuum->update_measurements();
    VehicleState vehicleState{mVacuum->get_state()};
    map->add_measurements_to_map(measurements, vehicleState);

}

void World::key_press(int key) const
{
    mVacuum->drive(key);
}

void World::key_release() const
{
    mVacuum->stop();
}
