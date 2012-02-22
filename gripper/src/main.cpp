#include <iostream>
#include <btBulletDynamicsCommon.h>

int main () {
 
    // Build the broadphase
    btBroadphaseInterface* broadphase = new btDbvtBroadphase();
 
    // Set up the collision configuration and dispatcher
    btDefaultCollisionConfiguration* collisionConfiguration = new btDefaultCollisionConfiguration();
    btCollisionDispatcher* dispatcher = new btCollisionDispatcher(collisionConfiguration);
 
    // The actual physics solver
    btSequentialImpulseConstraintSolver* solver = new btSequentialImpulseConstraintSolver;
 
    // The world.
    btDiscreteDynamicsWorld* dynamicsWorld = new btDiscreteDynamicsWorld(dispatcher,broadphase,solver,collisionConfiguration);
    dynamicsWorld->setGravity(btVector3(0,-10,0));
 
    // Do_everything_else_here
    //Ground
    btCollisionShape* groundShape = new btStaticPlaneShape(btVector3(0,1,0),1);    
    btDefaultMotionState* groundMotionState =
                new btDefaultMotionState(btTransform(btQuaternion(0,0,0,1),btVector3(0,-1,0)));
    btRigidBody::btRigidBodyConstructionInfo
                groundRigidBodyCI(0,groundMotionState,groundShape,btVector3(0,0,0));
    btRigidBody* groundRigidBody = new btRigidBody(groundRigidBodyCI);
    dynamicsWorld->addRigidBody(groundRigidBody);
    
    //Sphere
    btDefaultMotionState* fallMotionState =
                new btDefaultMotionState(btTransform(btQuaternion(0,0,0,1),btVector3(0,50,0)));    
    btScalar mass = 1;
    btVector3 fallInertia(0,0,0);
    btCollisionShape* fallShape = new btSphereShape(1);
    fallShape->calculateLocalInertia(mass,fallInertia);
    btRigidBody::btRigidBodyConstructionInfo fallRigidBodyCI(mass,fallMotionState,fallShape,fallInertia);
    btRigidBody* fallRigidBody = new btRigidBody(fallRigidBodyCI);
    dynamicsWorld->addRigidBody(fallRigidBody);
    
    double freq = 1000;
    int time_all = 5;
    int n_steps = time_all*freq;
    
    for (int i=0 ; i<n_steps ; i++) {
 
      dynamicsWorld->stepSimulation(1/freq,0);

      btTransform trans;
      fallRigidBody->getMotionState()->getWorldTransform(trans);
      std::cout << trans.getOrigin().getY() << std::endl;
    }
    // Clean up behind ourselves like good little programmers
    
    dynamicsWorld->removeRigidBody(fallRigidBody);
    delete fallRigidBody->getMotionState();
    delete fallRigidBody;

    dynamicsWorld->removeRigidBody(groundRigidBody);
    delete groundRigidBody->getMotionState();
    delete groundRigidBody;


    delete fallShape;

    delete groundShape;


    delete dynamicsWorld;
    delete solver;
    delete collisionConfiguration;
    delete dispatcher;
    delete broadphase;
 
    return 0;
}
