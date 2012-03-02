/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2006 Erwin Coumans  http://continuousphysics.com/Bullet/

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/



#define CUBE_HALF_EXTENTS 0.5
#define EXTRA_HEIGHT 1.f

#include "CcdPhysicsDemo.h"
#include "GlutStuff.h"
#include "GLDebugFont.h"

///btBulletDynamicsCommon.h is the main Bullet include file, contains most common include files.
#include "btBulletDynamicsCommon.h"

#include <stdio.h> //printf debugging
#include <iostream>
#include "GLDebugDrawer.h"
#include "btBulletWorldImporter.h"
#include <sstream>

#include <simulation.h>

static GLDebugDrawer	sDebugDrawer;

void callback(btDynamicsWorld *world, btScalar timeStep);

std::ostream& operator<< (std::ostream& os, btVector3 v) {
    os<<v.getX()<<", "<<v.getY()<<", "<<v.getZ();
    return os;
}

CcdPhysicsDemo::CcdPhysicsDemo(std::string gripper_filename, Simulation* sim)
:m_ccdMode(USE_CCD)
{
    setDebugMode(btIDebugDraw::DBG_DrawText+btIDebugDraw::DBG_NoHelpText);
    setCameraDistance(btScalar(20.));
    m_gripper_filename = gripper_filename;
    m_simulation = sim;
    m_simulation_clock.reset();
    m_simulation_step = 1;
    m_is_paused = true;
    createROSTransformation();
}

void CcdPhysicsDemo::createROSTransformation() {
    btMatrix3x3 rot(0, 0, 1,
                    1, 0, 0,
                    0, 1, 0);
    m_ROSTransform.setIdentity();
    m_ROSTransform.setBasis(rot);
    m_ROSTransform = m_ROSTransform.inverse();
    
}

void CcdPhysicsDemo::clientMoveAndDisplay()
{    
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    
    double first_tick;
    if (m_simulation_step == 0) {
        //beginning of the simulation
        first_tick = 0;
    }
    else {
        first_tick =  m_simulation->trajectory[m_simulation_step-1].time;    
    }
    double next_tick = m_simulation->trajectory[m_simulation_step].time; 
    double delta_tick = next_tick - first_tick;
    
    double elapsed = m_simulation_clock.getTimeMicroseconds() * 1.e-6;
//     std::cout<<"Elapsed time: "<<elapsed<<"\n";
    
    if (elapsed < delta_tick) {
        usleep(1);
        return;
    }
    
//     std::cerr<<"Step: "<<m_simulation_step<<", Elapsed: "<<elapsed<<", delta_tick: "<<delta_tick<<"\n";
    
    m_simulation_clock.reset();
    if ((! m_is_paused) && (m_simulation_step < m_simulation->trajectory.size()))
        step_simulation();
    else
        delta_tick = 0;
    
    ///step the simulation
    if (m_dynamicsWorld)
    {	  
        btScalar delta_t = delta_tick;
        m_dynamicsWorld->stepSimulation(delta_t);
        //optional but useful: debug drawing
        m_dynamicsWorld->debugDrawWorld();
    }
    
    renderme(); 
    
    displayText();
    
    glFlush();
    swapBuffers();
    
}

void CcdPhysicsDemo::displayText()
{
    int lineWidth=440;
//     int xStart = m_glutScreenWidth - lineWidth;
    int xStart = 0;
    int yStart = 0;
    
    if((getDebugMode() & btIDebugDraw::DBG_DrawText)!=0)
    {
        setOrthographicProjection();
        glDisable(GL_LIGHTING);
        glColor3f(0, 0, 0);
        char buf[124];
        
//         glRasterPos3f(xStart, yStart, 0);
        if (m_is_paused) {
            sprintf(buf,"Step %d, SIMULATION PAUSED", m_simulation_step);
        }
        else {
            sprintf(buf,"Step %d", m_simulation_step);
        }        
        yStart+=20;
        GLDebugDrawString(xStart,yStart,buf);
        
        std::stringstream ss;
        ss<<"Current gripper position: "<<m_ROSTransform.inverse() * m_gripper->getWorldTransform().getOrigin();
        yStart += 20;
        GLDebugDrawString(xStart,yStart,ss.str().c_str());
        
        ss.clear();
        ss.str("");        
        ss<<"Recorded gripper position: "<<m_simulation->trajectory[m_simulation_step-1].pose.position;
        yStart += 20;
        GLDebugDrawString(xStart,yStart,ss.str().c_str());
        
        
        sprintf(buf,"space to restart, h(elp), t(ext), w(ire)");
        yStart+=20;
        GLDebugDrawString(xStart,yStart,buf);
        
        resetPerspectiveProjection();
        glEnable(GL_LIGHTING);
    }	
    
}



void CcdPhysicsDemo::displayCallback(void) {
    
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); 
    
    renderme();
    
    displayText();
    
    //optional but useful: debug drawing to detect problems
    if (m_dynamicsWorld)
    {
        m_dynamicsWorld->debugDrawWorld();
    }
    
    glFlush();
    swapBuffers();
}

btCollisionShape* CcdPhysicsDemo::readGripper(void) {
    
    btBulletWorldImporter importer;
    if (! importer.loadFile(m_gripper_filename.c_str())) {
        std::cerr<<"Error loading "<<m_gripper_filename<<"\n";
        return NULL;
    }
        
    btCollisionShape* shape = importer.getCollisionShapeByName("gripper");
    if (shape == NULL) {
        std::cerr<<"No shape named gripper was found!\n";
        return NULL;
    }
    return shape;
        
    
}

void	CcdPhysicsDemo::initPhysics()
{
    setTexturing(true);
    setShadows(true);
//     m_is_paused = true;
    
    m_ShootBoxInitialSpeed = 20.f;
    m_simulation_clock.reset();
    m_simulation_step = 1;
    m_prevLinearVelocity = btVector3(0,0,0);
    m_prevAngularVelocity = btVector3(0,0,0);
    
    ///collision configuration contains default setup for memory, collision setup
    m_collisionConfiguration = new btDefaultCollisionConfiguration();
    //	m_collisionConfiguration->setConvexConvexMultipointIterations();
    
    ///use the default collision dispatcher. For parallel processing you can use a diffent dispatcher (see Extras/BulletMultiThreaded)
    m_dispatcher = new	btCollisionDispatcher(m_collisionConfiguration);
    m_dispatcher->registerCollisionCreateFunc(BOX_SHAPE_PROXYTYPE,BOX_SHAPE_PROXYTYPE,m_collisionConfiguration->getCollisionAlgorithmCreateFunc(CONVEX_SHAPE_PROXYTYPE,CONVEX_SHAPE_PROXYTYPE));
    
    m_broadphase = new btDbvtBroadphase();
    
    ///the default constraint solver. For parallel processing you can use a different solver (see Extras/BulletMultiThreaded)
    btSequentialImpulseConstraintSolver* sol = new btSequentialImpulseConstraintSolver;
    m_solver = sol;
    
    m_dynamicsWorld = new btDiscreteDynamicsWorld(m_dispatcher,m_broadphase,m_solver,m_collisionConfiguration);
    m_dynamicsWorld ->setDebugDrawer(&sDebugDrawer);
    m_dynamicsWorld->getSolverInfo().m_splitImpulse=true;
    m_dynamicsWorld->getSolverInfo().m_numIterations = 20;    
    
    if (m_ccdMode==USE_CCD)
    {
        m_dynamicsWorld->getDispatchInfo().m_useContinuous=true;
    } else
    {
        m_dynamicsWorld->getDispatchInfo().m_useContinuous=false;
    }
    
    m_dynamicsWorld->setGravity(btVector3(0,btScalar(-10),0));
    
    //Collision Box
    {
        btScalar mass(0.5);
        btVector3 localInertia(0,0,0);
        btVector3 boxdimensions = m_simulation->pre_box_dims;
        
        btCollisionShape* boxshape = new btBoxShape(boxdimensions);
        m_collisionShapes.push_back(boxshape);
        boxshape->calculateLocalInertia(mass, localInertia);
        
        btTransform init_transform = m_simulation->pre_box_pose.getTransform();
        btTransform box_transform = m_ROSTransform * init_transform;
        
        btDefaultMotionState* myMotionState = new btDefaultMotionState(box_transform);
        btRigidBody::btRigidBodyConstructionInfo rbInfo(mass,myMotionState,boxshape,localInertia);
        btRigidBody* body = new btRigidBody(rbInfo);
        body->setFriction(1.0);
        m_dynamicsWorld->addRigidBody(body);
        m_collision_body = body;
    }
    
    //Ground
    {
        btTransform groundTransform = btTransform::getIdentity();
        groundTransform.setIdentity();        
        btScalar ground_height = m_collision_body->getWorldTransform().getOrigin().getY() - (m_simulation->pre_box_dims.getZ());
//         btScalar ground_height = 0;
        btVector3 ground_pos = btVector3(0,ground_height,0);
        groundTransform.setOrigin(ground_pos);
        
        btBoxShape* groundShape = new btBoxShape(btVector3(btScalar(110.),btScalar(0),btScalar(110.)));
        m_collisionShapes.push_back(groundShape);

        std::cout<<"Ground pos: "<<ground_pos<<"\n";
        std::cout<<"Object pos: "<<m_collision_body->getWorldTransform().getOrigin()<<"\n";
        
        btScalar mass(0.);
        btVector3 localInertia(0,0,0);
        
        //using motionstate is recommended, it provides interpolation capabilities, and only synchronizes 'active' objects
        btDefaultMotionState* myMotionState = new btDefaultMotionState(groundTransform);
        btRigidBody::btRigidBodyConstructionInfo rbInfo(mass,myMotionState,groundShape,localInertia);
        btRigidBody* body = new btRigidBody(rbInfo);
        
        //add the body to the dynamics world
        m_dynamicsWorld->addRigidBody(body);
    }
    
    //Gripper
    {
        btScalar mass(2.0);
        btVector3 localInertia(0,0,0);
        btCollisionShape* gripper_shape = readGripper();
        m_collisionShapes.push_back(gripper_shape);
        gripper_shape->calculateLocalInertia(mass, localInertia);
        
        btTransform init_transform = m_simulation->trajectory[0].pose.getTransform();
        btTransform gripper_transform = m_ROSTransform* init_transform;
        
        btDefaultMotionState* myMotionState = new btDefaultMotionState(gripper_transform);
        btRigidBody::btRigidBodyConstructionInfo rbInfo(mass,myMotionState,gripper_shape,localInertia);
        btRigidBody* body = new btRigidBody(rbInfo);
        body->setGravity(btVector3(0,0,0));
        // 	  body->setFriction(1.0);
        m_dynamicsWorld->addRigidBody(body);
        
        m_gripper = body;
        m_gripper->setGravity(btVector3(0,0,0));
        
    }
//     m_dynamicsWorld->setInternalTickCallback(callback, this, true);
    
}

void	CcdPhysicsDemo::clientResetScene()
{
    exitPhysics();
    initPhysics();
}

void CcdPhysicsDemo::keyboardCallback(unsigned char key, int x, int y)
{
    if (key=='c')
    {
        switch (m_ccdMode)
        {
            case USE_CCD:
            {
                m_ccdMode = USE_NO_CCD;
                break;
            }
            case USE_NO_CCD:
            default:
            {
                m_ccdMode = USE_CCD;
            }
        };
        clientResetScene();
    } else  if (key == 'p')
    {
        m_is_paused = ! m_is_paused;        
    }
    else
    {
        DemoApplication::keyboardCallback(key,x,y);
    }
}


void	CcdPhysicsDemo::shootBox(const btVector3& destination)
{
    
    if (m_dynamicsWorld)
    {
        float mass = 0.1;
        btTransform startTransform;
        startTransform.setIdentity();
        btVector3 camPos = getCameraPosition();
        startTransform.setOrigin(camPos);
        
        setShootBoxShape ();
        
        
        btRigidBody* body = this->localCreateRigidBody(mass, startTransform,m_shootBoxShape);
        body->setLinearFactor(btVector3(1,1,1));
        //body->setRestitution(1);
        
        btVector3 linVel(destination[0]-camPos[0],destination[1]-camPos[1],destination[2]-camPos[2]);
        linVel.normalize();
        linVel*=m_ShootBoxInitialSpeed;
        
        body->getWorldTransform().setOrigin(camPos);
        body->getWorldTransform().setRotation(btQuaternion(0,0,0,1));
        body->setLinearVelocity(linVel);
        body->setAngularVelocity(btVector3(0,0,0));
        body->setContactProcessingThreshold(1e30);
        
        ///when using m_ccdMode, disable regular CCD
        if (m_ccdMode==USE_CCD)
        {
            body->setCcdMotionThreshold(CUBE_HALF_EXTENTS);
            body->setCcdSweptSphereRadius(0.4f);
        }
        
    }
}

void CcdPhysicsDemo::mystep(btScalar step)
{
    btVector3 v = m_gripper->getLinearVelocity();
    std::cout<<"Velocity: "<<v.x()<<", "<<v.y()<<", "<<v.z()<<"\n";
    m_gripper->activate(true);
    
//     m_collisionBody->applyCentralForce(btVector3(0.,force, 0.));
    
    btTransform transform;
    m_gripper->getMotionState()->getWorldTransform(transform);
    
    btVector3 pos = transform.getOrigin();
    
    btVector3 speed = btVector3(0.1, 0, 0);
    speed *= step;
    
    transform.setOrigin(pos + speed);
//     m_collisionBody->getMotionState()->setWorldTransform(transform);
    
//     m_collisionBody->setWorldTransform(transform);
//     m_collisionBody->setLinearVelocity(speed);
//     m_collisionBody->translate(speed);
//     m_collisionBody->applyCentralImpulse(mass * speed);
}


void CcdPhysicsDemo::step_simulation() {
    m_gripper->activate(true);
    btScalar inv_mass = m_gripper->getInvMass();
    
    btTransform prev_pose = m_simulation->trajectory[m_simulation_step-1].pose.getTransform();
    btTransform current_pose = m_simulation->trajectory[m_simulation_step].pose.getTransform();
    
    btTransform next_pose;
    if (m_simulation_step == m_simulation->trajectory.size()-1)
        next_pose = current_pose;
    else
        next_pose = m_simulation->trajectory[m_simulation_step+1].pose.getTransform();
    
    double time_diff = m_simulation->trajectory[m_simulation_step+1].time - m_simulation->trajectory[m_simulation_step].time;
    
    btVector3 thisStepLinearVel, thisStepAngularVel;
    btTransformUtil::calculateVelocity(current_pose, next_pose, time_diff, thisStepLinearVel, thisStepAngularVel);
    
    btVector3 linearImpulse = m_ROSTransform * (thisStepLinearVel - m_prevLinearVelocity) / inv_mass;
    btVector3 angularImpulse = m_ROSTransform * (thisStepAngularVel - m_prevAngularVelocity) / inv_mass;
    
    m_prevLinearVelocity = thisStepLinearVel;
    m_prevAngularVelocity = thisStepAngularVel;
    
//     m_gripper->applyCentralImpulse(linearImpulse);
//     m_gripper->applyTorqueImpulse(angularImpulse);
    
    m_gripper->setLinearVelocity(m_ROSTransform * thisStepLinearVel);
    m_gripper->setAngularVelocity(m_ROSTransform * thisStepAngularVel);
    
//     btTransform gripper_transform = m_ROSTransform * m_simulation->trajectory[m_simulation_step].pose.getTransform();    
//     m_gripper->setWorldTransform(gripper_transform);
    
    m_simulation_step++;
}


void CcdPhysicsDemo::exitPhysics()
{
    
    //cleanup in the reverse order of creation/initialization
    
    //remove the rigidbodies from the dynamics world and delete them
    int i;
    for (i=m_dynamicsWorld->getNumCollisionObjects()-1; i>=0 ;i--)
    {
        btCollisionObject* obj = m_dynamicsWorld->getCollisionObjectArray()[i];
        btRigidBody* body = btRigidBody::upcast(obj);
        if (body && body->getMotionState())
        {
            delete body->getMotionState();
        }
        m_dynamicsWorld->removeCollisionObject( obj );
        delete obj;
    }
    
    //delete collision shapes
    for (int j=0;j<m_collisionShapes.size();j++)
    {
        btCollisionShape* shape = m_collisionShapes[j];
        delete shape;
    }
    m_collisionShapes.clear();
    
    delete m_dynamicsWorld;
    
    delete m_solver;
    
    delete m_broadphase;
    
    delete m_dispatcher;
    
    delete m_collisionConfiguration;
    
}

void callback(btDynamicsWorld *world, btScalar timeStep) {
    
    CcdPhysicsDemo* demo = (CcdPhysicsDemo*)world->getWorldUserInfo();
    
    demo->mystep(timeStep);
}



