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
#ifndef BT_CCD_PHYSICS_DEMO_H
#define BT_CCD_PHYSICS_DEMO_H

#include "GlutDemoApplication.h"
#define PlatformDemoApplication GlutDemoApplication
#include <string>

#include "LinearMath/btAlignedObjectArray.h"


class btBroadphaseInterface;
class btCollisionShape;
class btOverlappingPairCache;
class btCollisionDispatcher;
class btConstraintSolver;
struct btCollisionAlgorithmCreateFunc;
class btDefaultCollisionConfiguration;
class Simulation;

///CcdPhysicsDemo is good starting point for learning the code base and porting.

class CcdPhysicsDemo : public PlatformDemoApplication
{

	//keep the collision shapes, for deletion/cleanup
	btAlignedObjectArray<btCollisionShape*>	m_collisionShapes;

	btBroadphaseInterface*	m_broadphase;

	btCollisionDispatcher*	m_dispatcher;
    btConstraintSolver*	m_solver;
	btRigidBody*	m_collision_body;
	btRigidBody*	m_gripper;
    Simulation* m_simulation;
    btClock m_simulation_clock;
    int m_simulation_step;
    bool m_is_paused;
    btTransform m_ROSTransform;
    btVector3 m_prevLinearVelocity;
    btVector3 m_prevAngularVelocity;
    

	enum
	{
		USE_CCD=1,
		USE_NO_CCD
	};
	int 	m_ccdMode;

	btDefaultCollisionConfiguration* m_collisionConfiguration;
    std::string m_gripper_filename;
    void step_simulation();
    void createROSTransformation();
    

	public:

    CcdPhysicsDemo(std::string gripper_filename, Simulation* sim);

	virtual ~CcdPhysicsDemo()
	{
		exitPhysics();
	}
	void	initPhysics();

	void	exitPhysics();

	virtual void clientMoveAndDisplay();

	void displayText();

	virtual void keyboardCallback(unsigned char key, int x, int y);

	virtual void displayCallback();
	virtual void	shootBox(const btVector3& destination);
	virtual void	clientResetScene();
    
    btCollisionShape* readGripper(void);
	
	static DemoApplication* Create(std::string filename, Simulation* sim)
	{
		CcdPhysicsDemo* demo = new CcdPhysicsDemo(filename, sim);
		demo->myinit();
		demo->initPhysics();
		return demo;
	}
	void mystep(btScalar);

	
};

#endif //BT_CCD_PHYSICS_DEMO_H

