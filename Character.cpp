#include "Character.h"


Character::Character(btDynamicsWorld* dynamicsWorld)
{
	m_character = 0;
	m_indexVertexArrays = 0;
	m_vertices = 0;
	m_dynamicsWorld = dynamicsWorld;
	m_overlappingPairCache = m_dynamicsWorld->getBroadphase();
	initPhysics();
	initDirection();
}


Character::~Character(void)
{
}
/******** Public Methods ********/
/* Get Methods */
osg::ref_ptr<osg::MatrixTransform> Character::getMan()
{
	return m_man;
}

btPairCachingGhostObject* Character::getGhostObject()
{
	return m_ghostObject;
}

btTransform Character::getCurrTrans()
{
	m_currTransform = m_ghostObject->getWorldTransform();
	return m_currTransform;
}

bool Character::getManualFlag()
{
	return m_manualFlag;
}

btScalar Character::getMaxSlope()
{
	return m_maxSlope;
}

btScalar Character::getJumpHeight()
{
	return m_jumpHeight;
}

/* Go Methods */

void Character::goForward()
{
	if(!getManualFlag())
		return;
	if(m_setDirectionFlag)
	{
		goDirection();
		return;
	}
	m_walkDirection = m_forwardDir;
	goMove();
}

void Character::goBackward()
{
	if(!getManualFlag())
		return;
	if(m_setDirectionFlag)
	{
		goDirection();
		return;
	}
	m_walkDirection = -m_forwardDir;
	goMove();
}

void Character::goLeft()
{
	if(!getManualFlag())
		return;
	if(m_setDirectionFlag)
	{
		goDirection();
		return;
	}
	m_currRot += (osg::PI)/180;
	checkRotThreshold();
	calcForwardDir();
	m_walkDirection = m_forwardDir;
	btMatrix3x3 orn = m_ghostObject->getWorldTransform().getBasis();
	orn *= btMatrix3x3(btQuaternion(btVector3(0,0,1),osg::PI/180));
	m_ghostObject->getWorldTransform().setBasis(orn);
	goMove();
}

void Character::goRight()
{
	if(!getManualFlag())
		return;
	if(m_setDirectionFlag)
	{
		goDirection();
		return;
	}
	m_currRot -= (osg::PI)/180;
	checkRotThreshold();
	calcForwardDir();
	m_walkDirection = m_forwardDir;
	btMatrix3x3 orn = m_ghostObject->getWorldTransform().getBasis();
	orn *= btMatrix3x3(btQuaternion(btVector3(0,0,1),-osg::PI/180));
	m_ghostObject->getWorldTransform().setBasis(orn);
	goMove();
}

void Character::goJump()
{
	if(!getManualFlag())
		return;
	if(m_character->canJump())
		m_character->jump();
}

void Character::goDirection()
{
	if(!getManualFlag())
		return;
	m_currRot -= (osg::PI)/180;
	checkRotThreshold();
	calcForwardDir();
	m_walkDirection = m_forwardDir;
	btMatrix3x3 orn = m_ghostObject->getWorldTransform().getBasis();
	orn *= btMatrix3x3(btQuaternion(btVector3(0,0,1),m_currRot));
	m_ghostObject->getWorldTransform().setBasis(orn);
	goMove();
	m_setDirectionFlag = false;
}

void Character::updateTrans()
{
	getCurrTrans();
	btVector3 tempTrans = m_currTransform.getOrigin();
	m_man->setMatrix(osg::Matrix::translate(tempTrans.x(), tempTrans.y(), tempTrans.z()));
	cout<<tempTrans.x()<<" "<<tempTrans.y()<<" "<< tempTrans.z()<<endl;
}

void Character::idle()
{
	m_character->reset(m_dynamicsWorld);
}

/********** Set Methods ************/
void Character::setStartOrigin(const btVector3& startOrigin)
{
	m_startOrigin = startOrigin;
	m_startTransform.setOrigin(m_startOrigin);
	m_ghostObject->setWorldTransform(m_startTransform);
}

void Character::setWalkVelocity(btScalar velocity)
{
	m_walkVelocity = velocity/3.6;//velocity mph
	m_walkSpeed = m_walkVelocity/60;
}

void Character::setWalkDirection(btScalar dirInDegrees , bool abs /*= false*/)
{
	m_currRot = osg::DegreesToRadians(dirInDegrees);
	btMatrix3x3 orn;
	if(abs)
	{
		orn.setIdentity();
	}
	else
	{
		orn = m_ghostObject->getWorldTransform().getBasis();
		orn *= btMatrix3x3(btQuaternion(btVector3(0,0,1),m_currRot));//gt0 for ccw;lt0 for cw
	}
	m_ghostObject->getWorldTransform().setBasis(orn);
	m_setDirectionFlag = true;
}

void Character::setInterval(btScalar interval/* = 5000*/)
{
	m_interval = interval;
}

void Character::setNextPos(btVector3& nextPos)
{
	setManualFlag(false);
	if(m_updateFlag)	//updateFlag == true
	{
		btVector3 tempPos = m_currTransform.getOrigin();
		setPrevPos(tempPos);
		m_nextPos = nextPos;
		m_nextTransform.setOrigin( m_nextPos );

		btScalar tempDistance = calcPointsDistance(tempPos, m_nextPos);
		if(tempDistance < 1)
		{
			idle();
		}
		else
		{
			btScalar tempRotation = calcPointsRotation(tempPos, m_nextPos);
			m_currRot = tempRotation;
			setWalkVelocity( 3600 * tempDistance / m_interval );
			setWalkDirection(osg::RadiansToDegrees(tempRotation), true);
			calcForwardDir();
			m_walkDirection = m_forwardDir;
			goMove();
		}
		m_frameCount = m_FPS * m_interval / 1000;
		m_updateFlag = false;
	}
	else	//updateFlag == false
	{
		if(m_frameCount > 0)
		{
			m_frameCount--;
		}
		if(m_frameCount == 0)//get new data method
		{
			m_updateFlag = true;
		}
	}

}

void Character::setPrevPos(btVector3& previousPos)
{
	m_prevPos = previousPos;
	m_prevTransform.setOrigin(m_prevPos);

}

void Character::setManualFlag(bool flag/* = true*/)
{
	m_manualFlag = flag;
}

// void Character::setStepHeight(btScalar stepHeight)
// {
// 	m_stepHeight = stepHeight;
// 	m_character->ste
// }

void Character::setMaxSlope(btScalar maxSlope)
{
	m_maxSlope = maxSlope;
	m_maxSlopeCosine = btCos(m_maxSlope);
	m_character->setMaxSlope(m_maxSlope);
}

void Character::setJumpHeight(btScalar jumpHeight)
{
	m_jumpHeight = jumpHeight;
	m_character->setMaxJumpHeight(m_jumpHeight);
}

/********* Private Methods ********/
void Character::initPhysics()
{
	//initial go directions
	m_goForward	= 0;
	m_goBackward	= 0;
	m_goLeft	= 0;
	m_goRight	= 0;
	m_goJump	= 0;
	m_currRot	= 0;
	m_startOrigin	= btVector3(0, 0, 50);
	m_stepHeight	= btScalar(0.45);
	m_startTransform.setIdentity();
	m_startTransform.setOrigin( m_startOrigin );

	m_updateFlag = true;
	m_interval	= 5000;

	m_walkDirection = btVector3(0.0, 0.0, 0.0);
	m_walkVelocity = btScalar(1.1) /** 4.0*/; // 4 km/h -> 1.1 m/s
	m_walkSpeed = m_walkVelocity /60;
	m_FPS = 60;

	initCharacter();
}

void Character::initCharacter()
{
	m_setDirectionFlag = false;
	m_capsule = new osg::Geode;
	m_man	=	new osg::MatrixTransform;

	m_characterHeight=1.75;//高度
	m_characterWidth =0.75;//半径
	/***********************/
	osg::ref_ptr<osg::ShapeDrawable> capsuleShape = new osg::ShapeDrawable(new osg::Capsule(osg::Vec3(0, 0, 0/*0*/), m_characterWidth, m_characterHeight ));
	m_capsule->addDrawable(capsuleShape);
	m_man->addChild(m_capsule);
	/*btConvexShape**/
	btConvexShape* capsule = new btCapsuleShapeZ(m_characterWidth, m_characterHeight);
	m_ghostObject = new btPairCachingGhostObject();
	m_ghostObject->setWorldTransform(m_startTransform);
	m_ghostObject->setCollisionShape (capsule);  
	m_ghostObject->setCollisionFlags (btCollisionObject::CF_CHARACTER_OBJECT);
	m_dynamicsWorld->getBroadphase()->getOverlappingPairCache()->setInternalGhostPairCallback(new btGhostPairCallback());

	m_character = new btKinematicCharacterController(m_ghostObject, capsule, m_stepHeight, 2);//2 for z-axis
	m_character->setGravity( btScalar(9.8) );//Gravity minus-axis direction
	m_character->warp(m_startOrigin);
	m_dynamicsWorld->addCollisionObject( m_ghostObject, btBroadphaseProxy::CharacterFilter,
		btBroadphaseProxy::StaticFilter|btBroadphaseProxy::DefaultFilter);
	m_dynamicsWorld->addAction(m_character);
}

void Character::initDirection()
{
	m_currTransform	= m_ghostObject->getWorldTransform();
	m_forwardDir	= m_currTransform.getBasis()[0];//	0	2	1
	m_upDir			= m_currTransform.getBasis()[1];//	1	1	2	
	m_strafeDir		= m_currTransform.getBasis()[2];//	2	0	0

	m_strafeDir.normalize();
	m_upDir.normalize();
	m_forwardDir.normalize();
}

void Character::goMove()
{
	m_currTransform = m_ghostObject->getWorldTransform();
	if(m_walkSpeed < 0.00001)
		idle();
	else
		m_character->setWalkDirection(m_walkDirection*m_walkSpeed);
	btQuaternion tempRot = m_currTransform.getRotation();
	btVector3 tempTrans = m_currTransform.getOrigin();
	/*以下内容会造成闪烁 注释掉……*/
	/*由update来更新*/
	//cout<<tempTrans.x()<<" "<<tempTrans.y()<<" "<< tempTrans.z()<<endl;//当前位置
	//m_man->setMatrix(osg::Matrix::translate(tempTrans.x(), tempTrans.y(),tempTrans.z()));
	//m_man->setMatrix(osg::Matrix::rotate(tempRot.getW(), tempRot.getX(), tempRot.getY(), tempRot.getZ()));
}

void Character::checkRotThreshold()
{
	if(m_currRot >= osg::PI)
	{
		m_currRot -= 2*osg::PI;
	}
	else if(m_currRot <= -osg::PI)
	{
		m_currRot += 2*osg::PI;
	}
}

void Character::calcForwardDir()
{
	if((m_currRot > osg::PI_2)||(m_currRot < -osg::PI_2))
	{
		m_forwardDir[0] = -1;
		m_forwardDir[1] = -btTan(m_currRot);
	}
	else if(m_currRot == osg::PI_2)
	{
		m_forwardDir[0] = 0;
		m_forwardDir[1] = 1;
	}
	else if(m_currRot == -osg::PI_2)
	{
		m_forwardDir[0] = 0;
		m_forwardDir[1] = -1;
	}
	else
	{
		m_forwardDir[0] = 1;
		m_forwardDir[1] = btTan(m_currRot);
	}
	m_forwardDir.normalize();
}

btScalar Character::calcPointsDistance(btVector3& src, btVector3& dest)
{
	btScalar tempDistance = btSqrt((dest.x()-src.x())*(dest.x()-src.x())
		+(dest.y()-src.y())*(dest.y()-src.y())
		/*+(dest.z()-src.z())*(dest.z()-src.z())*/);
	return tempDistance;
}
/**
* @brief 根据当前位置[src]和目标位置[dest]计算出人物的朝向
* @param [src] 当前位置
* @param [dest] 目标位置
* @return 返回角度值，btScalar，通过btMatrix3x3 和 btQuat转换，赋值给Worldtransform
*/
btScalar Character::calcPointsRotation(btVector3& src, btVector3& dest)
{
	btScalar deltaX = dest.x()-src.x();
	btScalar deltaY = dest.y()-src.y();
	if((deltaY == 0)&&(deltaX >= 0))
	{
		return 0;
	}
	else if((deltaY == 0)&&(deltaX < 0))
	{
		return osg::PI;
	}
	else if((deltaX == 0)&&(deltaY > 0))
	{
		return osg::PI_2;
	}
	else if((deltaX == 0)&&(deltaY < 0))
	{
		return -osg::PI_2;
	}
	else if(deltaX > 0)
	{
		return btAtan(deltaY/deltaX);
	}
	else if((deltaX < 0)&&(deltaY > 0))
	{
		return btAtan(deltaY/deltaX) + osg::PI;
	}
	else if((deltaX < 0)&&(deltaY < 0))
	{
		return btAtan(deltaY/deltaX) - osg::PI;
	}
	else
		return 0;
}