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

createCar::createCar(btDynamicsWorld* dynamicsWorld, const btVector3& carInitPos )
{
	initCar();
	m_carInitPos = carInitPos;
	m_dynamicsWorld = dynamicsWorld;

	osg::ref_ptr< osg::MatrixTransform > runningCar	=	createRunningCar();
	osg::ref_ptr< osg::MatrixTransform > staticCar	=	createStaticCar();
	osg::Vec3 postion =	osg::Vec3( m_carPosition->getMatrix().getTrans().x(),
		m_carPosition->getMatrix().getTrans().y(),
		m_carPosition->getMatrix().getTrans().z()+10 );

	osg::ref_ptr< osg::Node > staticCarLabel = createNameLabel( m_font, 15,
		"Static Car", postion );
	osg::Node * runningCarLabel = createNameLabel( m_font, 15,
		"Running Car", postion );

	m_carSwitchStateNode->addChild( runningCar, 0 );
	m_carSwitchStateNode->addChild( staticCar, 1 );
	m_carPosition->addChild( m_carSwitchStateNode );
	createRigidBody( m_carPosition );

// 	osg::ref_ptr< osg::AnimationPathCallback > acpb = new osg::AnimationPathCallback;
// 	acpb->setAnimationPath( carAnimationPath(osg::Vec3(0, 0, 0), osg::Vec3(0, 0, 5) ) );
// 	m_carPosition->setUpdateCallback( acpb );
// 	osgbCollision::RefBulletObject< btRigidBody >* carRigid = new osgbCollision::RefBulletObject< btRigidBody >( m_rbCar );//创建一个刚体盒子
// 	m_carPosition->setUserData( carRigid );
// 	osgbDynamics::RigidBodyAnimation * rba = new osgbDynamics::RigidBodyAnimation;
// 	acpb->setNestedCallback( rba );
	//设置速度会有摩擦阻力
//	m_rbCar->setLinearVelocity(btVector3(15, 15,0));

	//Adding labels after RigidBody to avoid the RigidLabel
	runningCar->addChild( runningCarLabel );
	staticCar->addChild( staticCarLabel );	
	m_dynamicsWorld->addRigidBody( m_rbCar );
}

void createCar::initCar( )
{
	m_font = "fonts/arial.ttf";
	
	m_carSwitchStateNode =	new osg::Switch;
	m_carPosition	=	new osg::MatrixTransform;
	m_previousPos	= btVector3(0, 0, 0);
	m_currentPos	= btVector3(0, 0, 0);
	m_updateState	= true;
	m_offsetAngle	= 90;
	m_frameCount	= 0;
	m_FPS = 60;
}

void createCar::carTimer()
{
	m_intervalTime = 5;/*Should be EndTime - BeginTime*/
}

void createCar::calcHeadingAngle()
{
	if( m_previousPos.x() == m_nextPos.x() )
	{
		if ( m_nextPos.y() - m_previousPos.y() > 0 )
		{
			m_headingAngle = osg::PI_2;
		}
		else if ( m_nextPos.y() - m_previousPos.y() < 0 )
		{
			m_headingAngle = -osg::PI_2;
		}
	}
	else
	{
		m_headingAngle = atan(( m_nextPos.y() - m_previousPos.y() ) / (m_nextPos.x() - m_previousPos.x()));
	}
// 	if( m_previousPos.x() == m_nextPos.x() )
// 	{
// 		if ( m_nextPos.y() - m_previousPos.y() > 0 )
// 		{
// 			m_headingAngle = osg::PI_2;
// 		}
// 		else if ( m_nextPos.y() - m_previousPos.y() < 0 )
// 		{
// 			m_headingAngle = -osg::PI_2;
// 		}
// 	}
// 	else if( m_previousPos.y() == m_nextPos.y())
// 	{
// 		if( m_nextPos.x() - m_previousPos.x() > 0)
// 		{
// 			m_headingAngle = 0;
// 		}
// 		else if( m_nextPos.x() - m_previousPos.x() < 0)
// 		{
// 			m_headingAngle = osg::PI;
// 		}
// 	}
// 	else if( m_nextPos.x() - m_previousPos.x() > 0)
// 	{
// 		m_headingAngle = btAtan(( m_currentPos.y() - m_previousPos.y() ) / ( m_currentPos.x() - m_previousPos.x()));
// 	}
// 	else
// 	{
// 		m_headingAngle = osg::PI + btAtan(( m_currentPos.y() - m_previousPos.y() ) / ( m_currentPos.x() - m_previousPos.x()));
// 	}

	m_headingAngle = osg::RadiansToDegrees( m_headingAngle ) + m_offsetAngle;
	m_headingAngle = osg::DegreesToRadians( m_headingAngle );
}

void createCar::calcStep()
{
	m_stepX = ( m_nextPos.x() - m_previousPos.x() ) / m_tempFrameCount;
	m_stepY = ( m_nextPos.y() - m_previousPos.y() ) / m_tempFrameCount;
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
	bodyTransform.setOrigin( m_carInitPos );//Position where the car show out
	motion->setWorldTransform( bodyTransform );

	btScalar mass( 500.0f );//
	btVector3 inertia;//
	collision->calculateLocalInertia( mass, inertia );//
	btRigidBody::btRigidBodyConstructionInfo rbinfo( mass, motion, collision, inertia );//
	m_rbCar = new btRigidBody( rbinfo );
	//_rbCar->setCollisionFlags( m_rbCar->getCollisionFlags() | btCollisionObject::CF_KINEMATIC_OBJECT );//去掉动力学才有用，为什么？
	m_rbCar->setActivationState( DISABLE_DEACTIVATION );
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
	return m_carPosition.release();
}

osg::Switch* createCar::getSwitchNode()
{
	return m_carSwitchStateNode.release();
}

btRigidBody* createCar::getRigidCar()
{
	return m_rbCar;
}

btTransform	createCar::getCarTransform()
{
	m_carMotionState = m_rbCar->getMotionState();
	m_carMotionState->getWorldTransform( m_carTransform );
	return m_carTransform;
}

btVector3 createCar::getCurrentPos()
{
	return m_currentPos;
}

btVector3 createCar::getPreviousPos()
{
	return m_previousPos;
}

// End Get Methods //

// Begin Set Methods //
bool createCar::setFont(const std::string& font)
{
	if( font.empty() )
	{
		m_font = font;
		return true;
	}
	else
	{
		m_font = "fonts/arial.ttf";
		return false;
	}
}

bool createCar::setCurrentPos(const btVector3& currentPos)
{
	m_carMotionState = m_rbCar->getMotionState();
	m_carMotionState->getWorldTransform( m_carTransform );

	if( m_updateState )
	{
		carTimer();		
		m_nextPos	= currentPos;
		m_frameCount = m_FPS * m_intervalTime;	//60 is FPS
		m_tempFrameCount = m_frameCount;
		calcHeadingAngle();
		calcStep();
		m_updateState = false;
	}
	else
	{
		m_frameCount--;

		if( m_previousPos == m_nextPos )
		{
			m_carSwitchStateNode->setValue( 0, false );
			m_carSwitchStateNode->setValue( 1, true );//Show Static One
		}
		else
		{
			m_carSwitchStateNode->setValue( 0, true );
			m_carSwitchStateNode->setValue( 1, false );//Show the Running One
			 
			btTransform tempTransform = m_carTransform;
			btVector3	tempPos = btVector3( tempTransform.getOrigin().x(),
				tempTransform.getOrigin().y(), tempTransform.getOrigin().z() );
			
			m_carRotation =  btQuaternion( btVector3( 0, 0, 1), m_headingAngle);
			m_carTransform.setRotation( m_carRotation );
			m_carMotionState->setWorldTransform( m_carTransform );
			m_rbCar->setMotionState( m_carMotionState );

			m_carTransform.setOrigin( btVector3( tempTransform.getOrigin().x() + m_stepX,
				tempTransform.getOrigin().y() + m_stepY, tempTransform.getOrigin().z() ) );
			m_carMotionState->setWorldTransform( m_carTransform );
			m_rbCar->setMotionState( m_carMotionState );
		}
		if( m_frameCount == 0 )
		{
			m_previousPos = m_nextPos;
			m_updateState = true;
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
//		 m_offsetAngle = 90;
//		btTransform rbTransform;
//		btMotionState* carMotionState;
//		carMotionState	= m_rbCar->getMotionState();
//		carMotionState->getWorldTransform( rbTransform );
//		float deltaTime = (endPoint.z() - beginPoint.z()) / numberSamples;
//		m_headingAngle = osg::DegreesToRadians( m_offsetAngle ) + std::atan( (endPoint.y() - beginPoint.y())
//			/ (endPoint.x() - beginPoint.x()));
//
//		for( unsigned int i = 0;
//			i < numberSamples; ++i )
//		{
//			osg::Vec3 pos( beginPoint.x() + i * (endPoint.x() - beginPoint.x()) / numberSamples,
//				beginPoint.y() + i * (endPoint.y() - beginPoint.y()) / numberSamples,
//				rbTransform.getOrigin().z());
//			osg::Quat rot( m_headingAngle, osg::Z_AXIS );
//			path->insert(deltaTime*i, osg::AnimationPath::ControlPoint(pos, rot));
//		}
//
//		return path.release();
//	}
//}