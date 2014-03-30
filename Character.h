#ifndef _CHARACTER_H_
#define _CHARACTER_H_
#pragma once
#include <btBulletDynamicsCommon.h>
#include <BulletDynamics\Character\btKinematicCharacterController.h>
#include <BulletCollision\CollisionDispatch\btGhostObject.h>
//#include "D:\Delbert\OSG\Bullet\Demos\OpenGL\DemoApplication.h"

#include <osgbDynamics/RigidBody.h>
#include <osgbDynamics/MotionState.h>
#include <osgbCollision/CollisionShapes.h>
#include <osgbCollision/RefBulletObject.h>

#include <osg/ShapeDrawable>
#include <osg/MatrixTransform>
#include <osg/PolygonMode>
#include <osg/Node>
#include <osg/Group>
#include <osg/Geode>

/*#include "MyKinematicMotionState.h"*/

#include <iostream>
using namespace std;

class btCollisionShape;
class btRigidBody;
class btKinematicCharacterController;
class btCharacterControllerInterface;

class Character 
// 	:
// 	public DemoApplication
{
public:
	Character(btDynamicsWorld* dynamicsWorld);
	~Character(void);

	btKinematicCharacterController* m_character;
	btVector3*	m_vertices;//¶¥µã

	class btPairCachingGhostObject* m_ghostObject;
	class btBroadphaseInterface*	m_overlappingPairCache;
	class btCollisionDispatcher*	m_dispatcher;
	class btConstraintSolver*		m_constraintSolver;
	class btDefaultCollisionConfiguration*	m_collisionConfiguration;
	class btTriangleIndexVertexArray*	m_indexVertexArrays;

	class MyKinematicMotionState* m_myKMS;

	osg::ref_ptr<osg::MatrixTransform>	getMan();
	btPairCachingGhostObject*			getGhostObject();
	btTransform	getCurrTrans();
	bool		getManualFlag();
	btScalar	getMaxSlope();
	btScalar	getJumpHeight();

	void goForward();
	void goBackward();
	void goLeft();
	void goRight();
	void goJump();
	void idle();
	void updateTrans();
	

	void setStartOrigin(const btVector3& startOrigin);
	void setWalkVelocity(btScalar velocity);//velocity mph
	void setWalkDirection(btScalar dirInDegrees, bool abs = false);//direction in degree
	void setPrevPos(btVector3& previousPos = btVector3(0, 0, 0));//initial as the origin point
	void setNextPos(btVector3& nextPos);
	void setInterval(btScalar interval = 5000);//time ms
	void setManualFlag(bool flag = true);
	//void setStepHeight(btScalar stepHeight);
	void setMaxSlope(btScalar maxSlope);
	void setJumpHeight(btScalar jumpHeight);

private:
	bool m_manualFlag;
	bool m_setDirectionFlag;
	bool m_updateFlag;

	int m_goForward;
	int m_goBackward;
	int m_goLeft;
	int m_goRight;
	int m_goJump;

	unsigned int m_frameCount;
	unsigned int m_tempFrameCount;
	unsigned int m_FPS;
	//unsigned int m_intervalTime;

	btScalar m_characterHeight;
	btScalar m_characterWidth;
	btScalar m_stepHeight;
	btScalar m_walkVelocity;
	btScalar m_walkSpeed;
	btScalar m_maxSlope;
	btScalar m_maxSlopeCosine;
	btScalar m_jumpHeight;

	btScalar m_currRot;
	btScalar m_interval;

	btVector3 m_startOrigin;
	btVector3 m_strafeDir;
	btVector3 m_upDir;
	btVector3 m_forwardDir;
	btVector3 m_walkDirection;
	btVector3 m_prevPos;
	btVector3 m_nextPos;

	btDynamicsWorld* m_dynamicsWorld;

	osgbDynamics::MotionState* m_motionState;
	btTransform m_startTransform;
	btTransform m_currTransform;
	btTransform m_nextTransform;
	btTransform m_prevTransform;

	osg::ref_ptr<osg::Geode> m_capsule;
	osg::ref_ptr<osg::MatrixTransform> m_man;

	void initCharacter();
	void initPhysics();//´¿Ðéº¯Êý
	void initDirection();

	void goMove();
	void goDirection();
	void checkRotThreshold();
	void calcForwardDir();

	btScalar calcPointsDistance(btVector3& src, btVector3& dest);
	btScalar calcPointsRotation(btVector3& src, btVector3& dest);
// 	void setDrawClusters(bool drawClusters);
// 	void myinit();
// 	void updateCamera();
// 	void clientMoveAndDisplay();
// 	void clientResetScene();
// 	void setShootBoxShape();
// 	void setShootBoxShape(const btVector3& destination);
// 	void keyboardCallback(unsigned char key, int x, int y);
// 	void keyboardUpCallback(unsigned char key, int x, int y) {}
// 	void specialKeyboard(int key, int x, int y){}//°´¼ü
// 	void specialKeyboardUp(int key, int x, int y){}//°´¼üµ¯Æð
// 	void reshape(int w, int h);
// 	void mouseFunc(int button, int state, int x, int y);
// 	void mouseMotionFunc(int x,int y);
// 	void displayCallback();
// 	void renderme();
// 	void swapBuffers();//Ë«»º³å£¿
// 	void updateModifierKeys();
};

#endif _CHARACTER_H_