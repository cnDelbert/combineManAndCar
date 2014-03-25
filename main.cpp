
#include <btBulletDynamicsCommon.h>

#include <osgbDynamics/RigidBody.h>
#include <osgbDynamics/MotionState.h>
#include <osgbCollision/CollisionShapes.h>
#include <osgbCollision/RefBulletObject.h>

#include <osgDB/ReadFile>
#include <osgGA/TrackballManipulator>
#include <osgGA/GUIEventAdapter>
#include <osg/ShapeDrawable>
#include <osgViewer/Viewer>
#include <osg/LineSegment>
#include <osgUtil/IntersectVisitor>

#include "Character.h"
#include "manController.h"
#include "createCar.h"

#include <vector>
#include <iostream>
using namespace std;

class Character;
class createCar;
class manController;

template <typename T>
struct wrapper : public T
{
	wrapper() {}
	wrapper(const T& rhs) : T(rhs) {}
};//解决 vector C2719 问题

btVector3 loadCarData1(const int i )//位置数据
{
	vector< wrapper<btVector3> > carPos;
	carPos.push_back( btVector3( 10, 20, 0) );
	carPos.push_back( btVector3( 20, 40, 0) );
	carPos.push_back( btVector3( 30, 50, 0) );
	carPos.push_back( btVector3( 40, 60, 0) );
	carPos.push_back( btVector3( 50, 50, 0) );
	carPos.push_back( btVector3( 60, 30, 0) );
	carPos.push_back( btVector3( 70, 40, 0) );
	carPos.push_back( btVector3( 80, 40, 0) );
	carPos.push_back( btVector3( 90, 50, 0) );
	carPos.push_back( btVector3( 100, 70, 0) );
	carPos.push_back( btVector3( 110, 60, 0) );
	carPos.push_back( btVector3( 120, 70, 0) );
	carPos.push_back( btVector3( 130, 80, 0) );
	carPos.push_back( btVector3( 140, 90, 0) );
	carPos.push_back( btVector3( 150, 100, 0) );
	carPos.push_back( btVector3( 160, 90, 0) );
	carPos.push_back( btVector3( 170, 100, 0) );
	carPos.push_back( btVector3( 180, 100, 0) );
	carPos.push_back( btVector3( 190, 90, 0) );
	carPos.push_back( btVector3( 200, 90, 0) );
	carPos.push_back( btVector3( 210, 90, 0) );
	carPos.push_back( btVector3( 220, 100, 0) );
	carPos.push_back( btVector3( 230, 100, 0) );
	carPos.push_back( btVector3( 240, 100, 0) );
	int j = i/300;
	if( j < carPos.size())
		return carPos[j];
	else
		return btVector3( 0, 0, 0);
}
btVector3 loadCarData2(const int i )//位置数据
{
	vector< wrapper<btVector3> > carPos;
	carPos.push_back( btVector3( 10, 70, 0) );
	carPos.push_back( btVector3( 20, 80, 0) );
	carPos.push_back( btVector3( 30, 80, 0) );
	carPos.push_back( btVector3( 40, 80, 0) );
	carPos.push_back( btVector3( 50, 90, 0) );
	carPos.push_back( btVector3( 60, 80, 0) );
	carPos.push_back( btVector3( 70, 90, 0) );
	carPos.push_back( btVector3( 80, 90, 0) );
	carPos.push_back( btVector3( 90, 90, 0) );
	carPos.push_back( btVector3( 100, 90, 0) );
	carPos.push_back( btVector3( 110, 90, 0) );
	carPos.push_back( btVector3( 120, 90, 0) );
	carPos.push_back( btVector3( 130, 80, 0) );
	carPos.push_back( btVector3( 140, 70, 0) );
	carPos.push_back( btVector3( 150, 80, 0) );
	carPos.push_back( btVector3( 160, 80, 0) );
	carPos.push_back( btVector3( 170, 80, 0) );
	carPos.push_back( btVector3( 180, 80, 0) );
	carPos.push_back( btVector3( 190, 80, 0) );
	carPos.push_back( btVector3( 200, 80, 0) );
	carPos.push_back( btVector3( 210, 80, 0) );
	carPos.push_back( btVector3( 220, 80, 0) );
	carPos.push_back( btVector3( 230, 80, 0) );
	carPos.push_back( btVector3( 240, 80, 0) );
	int j = i/300;
	if( j < carPos.size())
		return carPos[j];
	else
		return btVector3( 0, 0, 0);
}

btVector3 loadManData1(const int i )//位置数据
{
	vector< wrapper<btVector3> > manPos;
	manPos.push_back( btVector3( 35, 20, 0) );
	manPos.push_back( btVector3( 35, 10, 0) );
	manPos.push_back( btVector3( 70, 40, 0) );
	int j = i/300;
	if( j < manPos.size())
		return manPos[j];
	else
		return btVector3( 0, 0, 0);
}

osg::MatrixTransform* createTerrain( btDynamicsWorld* dynamicsWorld )
{
/*
 * BEGIN: Create physics object code.
 * OSG CODE
 * */
	osg::ref_ptr< osg::MatrixTransform > terrain;
	osg::ref_ptr< osg::Node > terrainDB = osgDB::readNodeFile("D:\\Delbert\\Projects\\Terrain Data\\OSGTerrain.ive"/*"lz.osg"*/);
	if( !terrainDB.valid() )
	{
		osg::notify( osg::NOTICE )<<"Can\'t load terrain file."<<std::endl;
		exit( 0 );
	}
	if( ( terrain = dynamic_cast< osg::MatrixTransform * >( terrainDB.get() ) ) == NULL )
	{
		terrain = new osg::MatrixTransform;
		terrain->addChild( terrainDB.get() );//转换不了则addChild
	}

	/*osgBullet Code*/
	osgbDynamics::MotionState* motion = new osgbDynamics::MotionState;
	motion->setTransform( terrain );
	//btCollisionShape* collision = osgbCollision::btTriMeshCollisionShapeFromOSG( terrain );//两种的效果差不太多,但是直接triMesh会触发断点
	btCollisionShape* collision = osgbCollision::btCompoundShapeFromOSGGeodes( terrain, TRIANGLE_MESH_SHAPE_PROXYTYPE , osgbCollision::Z );
	/*BULLET CODE*/
	btTransform terrainTransform;
	terrainTransform.setIdentity();
	terrainTransform.setOrigin(btVector3( -49225, -9000, 0) );//将坐标原点移动到地形中心
	motion->setWorldTransform( terrainTransform );

	btScalar mass(0.0f);
	btVector3 inertia;
	collision->calculateLocalInertia( mass, inertia );
	btRigidBody::btRigidBodyConstructionInfo rbInfo( mass, motion, collision, inertia );
	btRigidBody* terrainBody = new btRigidBody( rbInfo );
	//terrainBody->setActivationState( DISABLE_DEACTIVATION );
	dynamicsWorld->addRigidBody( terrainBody );

	return ( terrain.release() );
}

btDynamicsWorld* initPhysics()
{
	btDefaultCollisionConfiguration * collisionConfiguration = new btDefaultCollisionConfiguration();
	btCollisionDispatcher * dispatcher = new btCollisionDispatcher( collisionConfiguration );
	btConstraintSolver * solver = new btSequentialImpulseConstraintSolver;

	btVector3 worldAabbMin( -10000, -10000, -10000 );
	btVector3 worldAabbMax( 10000, 10000, 10000 );
	btBroadphaseInterface * inter = new btAxisSweep3( worldAabbMin, worldAabbMax, 1000 );

	btDynamicsWorld * dynamicsWorld = new btDiscreteDynamicsWorld( dispatcher, inter, solver, collisionConfiguration );
	dynamicsWorld->getDispatchInfo().m_allowedCcdPenetration = 0.0001f;
	dynamicsWorld->setGravity( btVector3( 0, 0, -9.8 ) );

	return dynamicsWorld;
}

/////////////////////////////////////////////////////////////
osg::MatrixTransform * createOSGBox( osg::Vec3 size )//创建一个osg的盒子，返回transform
{
	osg::Box * box = new osg::Box();//new一个box
	box->setHalfLengths( size );
	osg::ShapeDrawable * shape = new osg::ShapeDrawable( box );
	osg::Geode * geode = new osg::Geode();
	geode->addDrawable( shape );
	osg::MatrixTransform * transform = new osg::MatrixTransform();
	transform->addChild( geode );
	return( transform );
}

btRigidBody * createBTBox( osg::MatrixTransform * box,
	osg::Vec3 center )//返回一个Bullet刚体，用到上面创建的盒子和一个中心
{
	btCollisionShape * collision = osgbCollision::btBoxCollisionShapeFromOSG( box );//从box创建一个碰撞形体
	osgbDynamics::MotionState * motion = new osgbDynamics::MotionState();
	motion->setTransform( box );
	motion->setParentTransform( osg::Matrix::translate( center ) );
	btScalar mass( 0.0 );
	btVector3 inertia( 0, 0, 0 );//质量和惯性为零，最外围的盒子？
	btRigidBody::btRigidBodyConstructionInfo rb( mass, motion, collision, inertia );//创建一个刚体
	btRigidBody * body = new btRigidBody( rb );
	return( body );
}

btRigidBody * createBTBox2( osg::MatrixTransform * box,
	osg::Vec3 center )//返回一个Bullet刚体，用到上面创建的盒子和一个中心
{
	btCollisionShape * collision = osgbCollision::btBoxCollisionShapeFromOSG( box );//从box创建一个碰撞形体

	osgbDynamics::MotionState * motion = new osgbDynamics::MotionState();
	motion->setTransform( box );
	motion->setParentTransform( osg::Matrix::translate( center ) );

	btScalar mass( 1.0 );
	btVector3 inertia( 0, 0, 0 );//质量和惯性为零，最外围的盒子？
	btRigidBody::btRigidBodyConstructionInfo rb( mass, motion, collision, inertia );//创建一个刚体
	btRigidBody * body = new btRigidBody( rb );

	return( body );
}
////////////////////////////////////////////////////

int main()
{
	int i = 0;

	osg::ref_ptr<osg::Group> root = new osg::Group;
	btDynamicsWorld* dynamicsWorld = initPhysics();

	osg::MatrixTransform * terrain;
	btRigidBody * groundBody;

	float thin = .01;//盒子壁的厚度
	terrain = createOSGBox( osg::Vec3( 500, 500, thin ) );//创建地面，厚度0.01
	root->addChild( terrain );
	groundBody = createBTBox( terrain, osg::Vec3( 0, 0, 0 ) );//创建地面的刚体 0质量  第二个参数为center
	dynamicsWorld->addRigidBody( groundBody );

// 	osg::ref_ptr< osg::MatrixTransform > terrain = createTerrain( dynamicsWorld );
// 	root->addChild( terrain );

	createCar car;
	btRigidBody* rbCar = car.getRigidCar();	
	dynamicsWorld->addRigidBody( rbCar );
	root->addChild( car.getPosition() );
	///////////////////////////////////////////////////////////
	//For Car2
	createCar car2( btVector3( 10, 0, 2));
	//car2.setCarInitPos( btVector3( 10, 0, 0));
	btRigidBody* rbCar2 = car2.getRigidCar();
	dynamicsWorld->addRigidBody( rbCar2 );
	root->addChild( car2.getPosition() );


	Character* man = new Character(dynamicsWorld);
	root->addChild(man->getMan());
	root->addChild(terrain);

	osg::ref_ptr<osgGA::TrackballManipulator> tb = new osgGA::TrackballManipulator();
	tb->setHomePosition( 
		osg::Vec3( 90, 90, 20),	//osg::Vec3( 10., 10., 0.),
		osg::Vec3( 0, 0, 0),	//osg::Vec3( 0., 0., 1.),
		osg::Z_AXIS );	//Camera Position: From eye to center
	osg::ref_ptr<manController> manCtrl = new manController(man);
	man->setStartOrigin(btVector3(50,50,20));
	man->setManualFlag(false);
	man->setWalkDirection(45,true);

	osgViewer::Viewer viewer;
	viewer.setSceneData(root.get());
	viewer.setCameraManipulator(tb);
	viewer.addEventHandler(manCtrl);
	viewer.setUpViewInWindow( 30, 30, 768, 480 );
	viewer.realize();
	
	double prevSimTime = 0.;
	while (!viewer.done())
	{
		const double currSimTime = viewer.getFrameStamp()->getSimulationTime();
		dynamicsWorld->stepSimulation( currSimTime - prevSimTime );
		prevSimTime = currSimTime;		

// 		car.setCurrentPos( loadCarData1( i ) );
// 		car2.setCurrentPos( loadCarData2( i ) );//TO DO Change method to setNextPos
		man->setNextPos( loadManData1(i));
		man->updateTrans();
		i++;
		viewer.frame();
	}
	return 0;
}