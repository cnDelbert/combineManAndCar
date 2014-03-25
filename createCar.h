#pragma once

#include <osg/AnimationPath>
#include <osg/MatrixTransform>
#include <osg/AutoTransform>
#include <osg/ShapeDrawable>
#include <osg/Switch>
#include <osg/Geode>

#include <osgText/Text>

#include <btBulletDynamicsCommon.h>

#include <osgbDynamics/RigidBody.h>
#include <osgbDynamics/MotionState.h>
#include <osgbCollision/CollisionShapes.h>
#include <osgbCollision/RefBulletObject.h>

class createCar :
	public osg::Node
{
public:
	createCar( const btVector3& carInitPos = btVector3( 0, 0, 2));
	~createCar(void);

	osg::AnimationPathCallback*	createWheelAnimation(const osg::Vec3& base);
	osg::MatrixTransform*	createRunningCar();
	osg::MatrixTransform*	createStaticCar();
	osg::Node* createNameLabel( const std::string& textFont,
		float fontSize, const std::string& textMessage,
		const osg::Vec3& position );
	void	createRigidBody( osg::MatrixTransform* mainBody);

	osg::MatrixTransform*	getPosition();
	osg::Switch*			getSwitchNode();
	btRigidBody*			getRigidCar();
	btTransform	getCarTransform();
	btVector3	getCurrentPos();
	btVector3	getPreviousPos();

	bool	setFont(const std::string& font);
	//bool	setCarInitPos(const btVector3& initPos);
	void	setCurrentPos(const btVector3& currentPos);
	void	setNextPos(const btVector3& nextPos);	//for next position
	void	setPreviousPos(const btVector3& previousPos);	//for prev position
	//osg::AnimationPath*		carAnimationPath(osg::Vec3f& beginPoint, osg::Vec3f& endPoint);
private:
	bool	m_updateState;	//更新状态
	float	m_offsetAngle;	//模型默认补偿角 调整到X轴正向
	float	m_headingAngle;	//从正向开始算的朝向
	unsigned int	m_frameCount;
	unsigned int	m_tempFrameCount;
	unsigned int	m_intervalTime;
	unsigned int	m_FPS;
	
	std::string m_font;
	btRigidBody* m_rbCar;

	btVector3 m_carInitPos;
	btVector3 m_nextPos;
	btVector3 m_currentPos;
	btVector3 m_previousPos;
	btVector3 m_tempPos;

	btScalar m_stepX;
	btScalar m_stepY;

	btTransform m_carTransform;
	btQuaternion m_carRotation;
	btMotionState* m_carMotionState;

 	osg::ref_ptr< osg::Switch > m_carSwitchStateNode;
 	osg::ref_ptr< osg::MatrixTransform > m_carPosition;

	void initCar();
	void carTimer();
	void calcHeadingAngle();
	void calcStep();//TO DO
};

