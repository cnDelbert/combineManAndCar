#include "createCar.h"

#include <osg/PolygonMode>
#include <osg/PolygonOffset>

//#include <osgbDynamics/RigidBodyAnimation.h>

osg::MatrixTransform* createTransformNode( osg::Drawable*
	shape, const osg::Matrix& matrix )
{
	osg::ref_ptr<osg::Geode> geode = new osg::Geode;
	geode->addDrawable( shape );
	osg::ref_ptr<osg::MatrixTransform> trans = new osg::MatrixTransform;
	trans->addChild( geode.get() );
	trans->setMatrix( matrix );
	return trans.release();
}

createCar::createCar( const btVector3& carInitPos )
{
	initCar();
	_carInitPos = carInitPos;

	osg::ref_ptr< osg::MatrixTransform > runningCar	=	createRunningCar();
	osg::ref_ptr< osg::MatrixTransform > staticCar	=	createStaticCar();
	osg::Vec3 postion =	osg::Vec3( _carPosition->getMatrix().getTrans().x(),
		_carPosition->getMatrix().getTrans().y(),
		_carPosition->getMatrix().getTrans().z()+10 );

	osg::ref_ptr< osg::Node > staticCarLabel = createNameLabel( _font, 15,
		"Static Car", postion );
	osg::Node * runningCarLabel = createNameLabel( _font, 15,
		"Running Car", postion );

	_carSwitchStateNode->addChild( runningCar, 0 );
	_carSwitchStateNode->addChild( staticCar, 1 );
	_carPosition->addChild( _carSwitchStateNode );
	createRigidBody( _carPosition );

	//设置动画的话 则会不受碰撞和重力影响
// 	osg::ref_ptr< osg::AnimationPathCallback > acpb = new osg::AnimationPathCallback;
// 	acpb->setAnimationPath( carAnimationPath(osg::Vec3(0, 0, 0), osg::Vec3(0, 0, 5) ) );
// 	_carPosition->setUpdateCallback( acpb );
// 	osgbCollision::RefBulletObject< btRigidBody >* carRigid = new osgbCollision::RefBulletObject< btRigidBody >( _rbCar );//创建一个刚体盒子
// 	_carPosition->setUserData( carRigid );
// 	osgbDynamics::RigidBodyAnimation * rba = new osgbDynamics::RigidBodyAnimation;
// 	acpb->setNestedCallback( rba );
	//设置速度会有摩擦阻力
//	_rbCar->setLinearVelocity(btVector3(15, 15,0));

	//Adding labels after RigidBody to avoid the RigidLabel
	runningCar->addChild( runningCarLabel );
	staticCar->addChild( staticCarLabel );	
}

void createCar::initCar( )
{
	_font = "fonts/arial.ttf";
	
	_carSwitchStateNode =	new osg::Switch;
	_carPosition	=	new osg::MatrixTransform;
	_previousPos	= btVector3(0, 0, 0);
	_currentPos		= btVector3(0, 0, 0);
	_updateState	= true;
	_offsetAngle	= 90;
	_frameCount	= 0;
	_FPS = 60;
}

void createCar::carTimer()
{
	_intervalTime = 5;/*Should be EndTime - BeginTime*/
}

void createCar::calcHeadingAngle()
{
	if( _previousPos.x() == _currentPos.x() )
	{
		if ( _currentPos.y() - _previousPos.y() > 0 )
		{
			_headingAngle = osg::PI_2;
		}
		else if ( _currentPos.y() - _previousPos.y() < 0 )
		{
			_headingAngle = -osg::PI_2;
		}
	}
	else
	{
		_headingAngle = atan(( _currentPos.y() - _previousPos.y() ) / (_currentPos.x() - _previousPos.x()));
	}
	_headingAngle = osg::RadiansToDegrees( _headingAngle ) + _offsetAngle;
	_headingAngle = osg::DegreesToRadians( _headingAngle );
}

createCar::~createCar(void)
{
}



osg::AnimationPathCallback*	createCar::createWheelAnimation(const osg::Vec3& base)
{// to make animation wheels
	osg::ref_ptr<osg::AnimationPath> wheelPath =
		new osg::AnimationPath;
	wheelPath->setLoopMode( osg::AnimationPath::LOOP );
	wheelPath->insert( 0.0, osg::AnimationPath::ControlPoint(
		base, osg::Quat()) );
	wheelPath->insert( 0.01, osg::AnimationPath::ControlPoint(
		base + osg::Vec3(0.0f, 0.02f, 0.0f), osg::Quat(
		osg::PI_2, osg::Z_AXIS)) );
	wheelPath->insert( 0.02, osg::AnimationPath::ControlPoint(
		base + osg::Vec3(0.0f,-0.02f, 0.0f), osg::Quat(
		osg::PI, osg::Z_AXIS)) );
	osg::ref_ptr<osg::AnimationPathCallback> apcb =
		new osg::AnimationPathCallback;
	apcb->setAnimationPath( wheelPath.get() );
	return apcb.release();
}


osg::MatrixTransform* createCar::createRunningCar()
{
	//默认是Y UP的
	// The prototype of the main rod
	osg::ref_ptr<osg::ShapeDrawable> mainRodShape =
		new osg::ShapeDrawable( new osg::Cylinder(
		osg::Vec3(), 0.4f, 10.0f) );
	// The prototype of the coupling (wheel) rod
	osg::ref_ptr<osg::ShapeDrawable> wheelRodShape =
		new osg::ShapeDrawable( new osg::Cylinder(
		osg::Vec3(), 0.4f, 8.0f) );
	// The prototypes of the wheel and the car body
	osg::ref_ptr<osg::ShapeDrawable> wheelShape =
		new osg::ShapeDrawable( new osg::Cylinder(
		osg::Vec3(), 2.0f, 1.0f) );
	osg::ref_ptr<osg::ShapeDrawable> bodyShape =
		new osg::ShapeDrawable( new osg::Box(
		osg::Vec3(), 6.0f, 14.0f, 4.0f) );
	//换成Z轴向上的

	osg::ref_ptr< osg::MatrixTransform > wheel1 = createTransformNode( wheelShape, osg::Matrix::translate( 0.0f, 0.0f, -4.0f));
	wheel1->setUpdateCallback( createWheelAnimation( osg::Vec3( 0.0f, 0.0f, -4.0f)));

	osg::ref_ptr< osg::MatrixTransform > wheel2 = createTransformNode(
		wheelShape.get(), osg::Matrix::translate(0.0f, 0.0f, 4.0f) );
	wheel2->setUpdateCallback(
		createWheelAnimation(osg::Vec3(0.0f, 0.0f, 4.0f)) );

	osg::ref_ptr< osg::MatrixTransform > wheelRod1 = createTransformNode(
		wheelRodShape.get(),
		osg::Matrix::rotate(osg::Z_AXIS, osg::X_AXIS) *
		osg::Matrix::translate(0.0f, 7.0f,-2.0f) );
	wheelRod1->addChild( wheel1 );
	wheelRod1->addChild( wheel2 );

	osg::ref_ptr< osg::MatrixTransform > wheelRod2 =
		static_cast< osg::MatrixTransform* >(
		wheelRod1->clone(osg::CopyOp::SHALLOW_COPY) );
	wheelRod2->setMatrix( osg::Matrix::rotate(osg::Z_AXIS,
		osg::X_AXIS) * osg::Matrix::translate(0.0f, -3.0f, -2.0f) );

	osg::ref_ptr< osg::MatrixTransform > carBody = createTransformNode(
		bodyShape.get(), osg::Matrix::translate(0.0f, 2.2f, 0.0f) );
	osg::ref_ptr< osg::MatrixTransform > mainRod = createTransformNode(
		mainRodShape.get(), osg::Matrix::rotate( osg::Z_AXIS, osg::Y_AXIS ) * osg::Matrix::translate( osg::Vec3( 0, 2, -2) ) );

	osg::ref_ptr< osg::MatrixTransform > mainBody = new osg::MatrixTransform;
	osg::ref_ptr< osg::MatrixTransform > unitedCar = new osg::MatrixTransform;//United parts into a car
	unitedCar->addChild( mainRod );
	unitedCar->addChild( carBody );
	unitedCar->addChild( wheelRod2 );
	unitedCar->addChild( wheelRod1 );

	unitedCar->setMatrix( osg::Matrix::translate( osg::Vec3( 0, 0, 0) ) );//And make translation
	mainBody->addChild( unitedCar );

	return (mainBody.release());
}


osg::MatrixTransform* createCar::createStaticCar()
{
	//默认是Y UP的
	// The prototype of the main rod
	osg::ref_ptr<osg::ShapeDrawable> mainRodShape =
		new osg::ShapeDrawable( new osg::Cylinder(
		osg::Vec3(), 0.4f, 10.0f) );
	// The prototype of the coupling (wheel) rod
	osg::ref_ptr<osg::ShapeDrawable> wheelRodShape =
		new osg::ShapeDrawable( new osg::Cylinder(
		osg::Vec3(), 0.4f, 8.0f) );
	// The prototypes of the wheel and the car body
	osg::ref_ptr<osg::ShapeDrawable> wheelShape =
		new osg::ShapeDrawable( new osg::Cylinder(
		osg::Vec3(), 2.0f, 1.0f) );
	// 	osg::ref_ptr<osg::ShapeDrawable> bodyShape =
	// 		new osg::ShapeDrawable( new osg::Box(
	// 		osg::Vec3(), 6.0f, 4.0f, 14.0f) );
	osg::ref_ptr<osg::ShapeDrawable> bodyShape =
		new osg::ShapeDrawable( new osg::Box(
		osg::Vec3(), 6.0f, 14.0f, 4.0f) );
	//换成Z轴向上的

	osg::ref_ptr< osg::MatrixTransform > wheel1 = createTransformNode( wheelShape, osg::Matrix::translate( 0.0f, 0.0f, -4.0f));
	osg::ref_ptr< osg::MatrixTransform > wheel2 = createTransformNode( wheelShape, osg::Matrix::translate(0.0f, 0.0f, 4.0f) );

	osg::ref_ptr< osg::MatrixTransform > wheelRod1 = createTransformNode(
		wheelRodShape.get(),
		osg::Matrix::rotate(osg::Z_AXIS, osg::X_AXIS) *
		osg::Matrix::translate(0.0f, 7.0f,-2.0f) );
	wheelRod1->addChild( wheel1 );
	wheelRod1->addChild( wheel2 );

	osg::ref_ptr< osg::MatrixTransform > wheelRod2 =
		static_cast< osg::MatrixTransform* >(
		wheelRod1->clone(osg::CopyOp::SHALLOW_COPY) );
	wheelRod2->setMatrix( osg::Matrix::rotate(osg::Z_AXIS,
		osg::X_AXIS) * osg::Matrix::translate(0.0f, -3.0f, -2.0f) );

	osg::ref_ptr< osg::MatrixTransform > carBody = createTransformNode(
		bodyShape.get(), osg::Matrix::translate(0.0f, 2.2f, 0.0f) );
	osg::ref_ptr< osg::MatrixTransform > mainRod = createTransformNode(
		mainRodShape.get(), osg::Matrix::rotate( osg::Z_AXIS, osg::Y_AXIS ) * osg::Matrix::translate( osg::Vec3( 0, 2, -2) ));

	osg::ref_ptr< osg::MatrixTransform > mainBody = new osg::MatrixTransform;
	osg::ref_ptr< osg::MatrixTransform > unitedCar = new osg::MatrixTransform;//United parts into a car
	unitedCar->addChild( mainRod );
	unitedCar->addChild( carBody );
	unitedCar->addChild( wheelRod2 );
	unitedCar->addChild( wheelRod1 );

	unitedCar->setMatrix( osg::Matrix::translate( osg::Vec3( 0, 0, 0) ) );//And make translation
	mainBody->addChild( unitedCar );

	return (mainBody.release());
}


/*btRigidBody**/
void createCar::createRigidBody( osg::MatrixTransform* mainBody)
{
	/*osgBullet Code*/
	osgbDynamics::MotionState* motion = new osgbDynamics::MotionState;
	motion->setTransform( mainBody );
	btCollisionShape* collision = osgbCollision::btConvexTriMeshCollisionShapeFromOSG( mainBody );


// 	 	// This is mainly for debugging.
// 	 	osg::Node* debugNode = osgbCollision::osgNodeFromBtCollisionShape( collision );//For Debugging
// 	 	mainBody->addChild( debugNode );
// 	 
// 	 	// Set debug node state.
// 	 	osg::StateSet* state = debugNode->getOrCreateStateSet();
// 	 	osg::PolygonMode* pm = new osg::PolygonMode( osg::PolygonMode::FRONT_AND_BACK, osg::PolygonMode::LINE );
// 	 	state->setAttributeAndModes( pm );
// // 	 	osg::PolygonOffset* po = new osg::PolygonOffset( -1, -1 );
// // 	 	state->setAttributeAndModes( po );
// 	 	state->setMode( GL_LIGHTING, osg::StateAttribute::OFF );

	/*  BULLET CODE */
	btTransform bodyTransform;
	bodyTransform.setIdentity();//
	bodyTransform.setOrigin( _carInitPos );//Position where the car show out
	motion->setWorldTransform( bodyTransform );

	btScalar mass( 500000.0f );//
	btVector3 inertia;//
	collision->calculateLocalInertia( mass, inertia );//
	btRigidBody::btRigidBodyConstructionInfo rbinfo( mass, motion, collision, inertia );//
	_rbCar = new btRigidBody( rbinfo );
	//_rbCar->setCollisionFlags( _rbCar->getCollisionFlags() | btCollisionObject::CF_KINEMATIC_OBJECT );//去掉动力学才有用，为什么？
	_rbCar->setActivationState( DISABLE_DEACTIVATION );
}


osg::Node* createCar::createNameLabel( const std::string& textFont,
	float fontSize, const std::string& textMessage,
	const osg::Vec3& position )
{
	osg::ref_ptr< osgText::Text > textLabel	= new osgText::Text;
	osg::ref_ptr< osg::Geode >		geode	= new osg::Geode;
	osg::ref_ptr< osg::AutoTransform > at	= new osg::AutoTransform;
	textLabel->setCharacterSize( fontSize );
	textLabel->setText( textMessage );
	textLabel->setFont( textFont );
	textLabel->setAlignment( osgText::Text::CENTER_CENTER );

	geode->addDrawable( textLabel );
	at->addChild( geode );
	at->setAutoRotateMode( osg::AutoTransform::ROTATE_TO_CAMERA );
	at->setAutoScaleToScreen( true );
	at->setMinimumScale( 0.0 );
	at->setMaximumScale( 1.0 );
	at->setPosition( position );

	return at.release();
}

// Begin Get Methods //
osg::MatrixTransform* createCar::getPosition()
{
	return _carPosition.release();
}

osg::Switch* createCar::getSwitchNode()
{
	return _carSwitchStateNode.release();
}

btRigidBody* createCar::getRigidCar()
{
	return _rbCar;
}

btTransform	createCar::getCarTransform()
{
	_carMotionState = _rbCar->getMotionState();
	_carMotionState->getWorldTransform( _carTransform );
	return _carTransform;
}

btVector3 createCar::getCurrentPos()
{
	return _currentPos;
}

btVector3 createCar::getPreviousPos()
{
	return _previousPos;
}

// End Get Methods //

// Begin Set Methods //
bool createCar::setFont(const std::string& font)
{
	if( font.empty() )
	{
		_font = font;
		return true;
	}
	else
	{
		_font = "fonts/arial.ttf";
		return false;
	}
}

// bool createCar::setCarInitPos(const btVector3& initPos)
// {
// // 	if( _carInitPos = initPos )
// // 	{
// // 		return true;
// // 	}
// // 	else
// // 	{
// // 		return false;
// // 	}
// 	_carInitPos = initPos;
// 	_previousPos = _carInitPos;
// 	return true;
// }

bool createCar::setCurrentPos(const btVector3& currentPos)
{
	_carMotionState = _rbCar->getMotionState();
	_carMotionState->getWorldTransform( _carTransform );

	if( _updateState )
	{
		carTimer();		
		_currentPos	= currentPos;
		calcHeadingAngle();
		_frameCount = _FPS * _intervalTime;	//60 is FPS
		_tempFrameCount = _frameCount;		
		_updateState = false;
	}
	else
	{
		_frameCount--;

		if( _previousPos == _currentPos )
		{
			_carSwitchStateNode->setValue( 0, false );
			_carSwitchStateNode->setValue( 1, true );//Show Static One
		}
		else
		{
			_carSwitchStateNode->setValue( 0, true );
			_carSwitchStateNode->setValue( 1, false );//Show the Running One

			btTransform tempTransform = _carTransform;
			btVector3	tempPos = btVector3( tempTransform.getOrigin().x(),
				tempTransform.getOrigin().y(), tempTransform.getOrigin().z() );
			float stepX = ( _currentPos.x() - _previousPos.x() ) / _tempFrameCount;
			float stepY = ( _currentPos.y() - _previousPos.y() ) / _tempFrameCount;
			
			_carRotation =  btQuaternion( btVector3( 0, 0, 1), _headingAngle);
			_carTransform.setRotation( _carRotation );
			_carMotionState->setWorldTransform( _carTransform );
			_rbCar->setMotionState( _carMotionState );

			_carTransform.setOrigin( btVector3( tempTransform.getOrigin().x() + stepX,
				tempTransform.getOrigin().y() + stepY, tempTransform.getOrigin().z() ) );
			_carMotionState->setWorldTransform( _carTransform );
			_rbCar->setMotionState( _carMotionState );
		}
		if( _frameCount == 0 )
		{
			_previousPos = _currentPos;
			_updateState = true;
		}
	}
	return true;
}
// End Set Methods //

//osg::AnimationPath*	 createCar::carAnimationPath(osg::Vec3f& beginPoint, osg::Vec3f& endPoint)
//{
//	if((beginPoint.x() == endPoint.x()) && (beginPoint.y() == endPoint.y()))
//	{
//	
//	}
//	else
//	{
//		osg::ref_ptr< osg::AnimationPath > path = new osg::AnimationPath;
//		float beginTime = beginPoint.z();
//		float endTime	= endPoint.z();
//		unsigned int numberSamples = 32;
//		 _offsetAngle = 90;
//		btTransform rbTransform;
//		btMotionState* carMotionState;
//		carMotionState	= _rbCar->getMotionState();
//		carMotionState->getWorldTransform( rbTransform );
//		float deltaTime = (endPoint.z() - beginPoint.z()) / numberSamples;
//		_headingAngle = osg::DegreesToRadians( _offsetAngle ) + std::atan( (endPoint.y() - beginPoint.y())
//			/ (endPoint.x() - beginPoint.x()));
//
//		for( unsigned int i = 0;
//			i < numberSamples; ++i )
//		{
//			osg::Vec3 pos( beginPoint.x() + i * (endPoint.x() - beginPoint.x()) / numberSamples,
//				beginPoint.y() + i * (endPoint.y() - beginPoint.y()) / numberSamples,
//				rbTransform.getOrigin().z());
//			osg::Quat rot( _headingAngle, osg::Z_AXIS );
//			path->insert(deltaTime*i, osg::AnimationPath::ControlPoint(pos, rot));
//		}
//
//		return path.release();
//	}
//}