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
	bool	setCurrentPos(const btVector3& currentPos);
	//bool	setPreviousPos(const btVector3& previousPos);
	//osg::AnimationPath*		carAnimationPath(osg::Vec3f& beginPoint, osg::Vec3f& endPoint);
private:
	bool	_updateState;	//更新状态
	float	_offsetAngle;	//模型默认补偿角 调整到X轴正向
	float	_headingAngle;	//从正向开始算的朝向
	unsigned int	_frameCount;
	unsigned int	_tempFrameCount;
	unsigned int	_intervalTime;
	unsigned int	_FPS;
	
	std::string _font;
	btRigidBody* _rbCar;

	btVector3 _carInitPos;
	btVector3 _currentPos;
	btVector3 _previousPos;
	btVector3 _tempPos;

	btTransform _carTransform;
	btQuaternion _carRotation;
	btMotionState* _carMotionState;

 	osg::ref_ptr< osg::Switch > _carSwitchStateNode;
 	osg::ref_ptr< osg::MatrixTransform > _carPosition;

	void initCar();
	void carTimer();
	void calcHeadingAngle();
};

