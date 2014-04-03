#include <osgViewer/Viewer>
#include <osg/MatrixTransform>
#include <osgDB/ReadFile>
#include <osgGA/TrackballManipulator>

//#include "carSwitchController.h"
#include "createCar.h"
#include "Character.h"
#include "manController.h"

#include <btBulletDynamicsCommon.h>

#include <vector>
#include <iostream>
#include <fstream>
using namespace std;

template <typename T>
struct wrapper : public T
{
	wrapper() {}
	wrapper(const T& rhs) : T(rhs) {}
};//To solve Error:vector C2719

class createCar;

//Begin Physics Initialize
btDynamicsWorld* initPhysics()//Physics Initialize
{
	btDefaultCollisionConfiguration * collisionConfiguration = new btDefaultCollisionConfiguration();
	btCollisionDispatcher * dispatcher = new btCollisionDispatcher( collisionConfiguration );
	btConstraintSolver * solver = new btSequentialImpulseConstraintSolver;

	btVector3 worldAabbMin( -10000, -10000, -10000 );
	btVector3 worldAabbMax( 10000, 10000, 10000 );
	btBroadphaseInterface * inter = new btAxisSweep3( worldAabbMin, worldAabbMax, 1000 );

	btDynamicsWorld * dynamicsWorld = new btDiscreteDynamicsWorld( dispatcher, inter, solver, collisionConfiguration );
	dynamicsWorld->setGravity( btVector3( 0, 0, -9.8 ) );

	return( dynamicsWorld );
}
//End Initialize

//Begin createTerrain
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
//End createTerrain


btVector3 loadCarData1(const int i )//Position Data
{
	vector< wrapper<btVector3> > carPos;
	carPos.push_back( btVector3( 10, 20, 10) );
	carPos.push_back( btVector3( 20, 40, -1) );
	carPos.push_back( btVector3( 30, 50, 1) );
	carPos.push_back( btVector3( 40, 60, 1) );
	carPos.push_back( btVector3( 50, 50, 1) );
	carPos.push_back( btVector3( 60, 30, 1) );
	carPos.push_back( btVector3( 70, 40, 1) );
	carPos.push_back( btVector3( 80, 40, 1) );
	carPos.push_back( btVector3( 90, 50, 1) );
	carPos.push_back( btVector3( 100, 70, 1) );
	carPos.push_back( btVector3( 110, 60, 1) );
	carPos.push_back( btVector3( 120, 70, 1) );
	carPos.push_back( btVector3( 130, 80, 1) );
	carPos.push_back( btVector3( 140, 90, 1) );
	carPos.push_back( btVector3( 150, 101, 1) );
	carPos.push_back( btVector3( 160, 90, 1) );
	carPos.push_back( btVector3( 170, 101, 1) );
	carPos.push_back( btVector3( 180, 101, 1) );
	carPos.push_back( btVector3( 190, 90, 1) );
	carPos.push_back( btVector3( 200, 90, 1) );
	carPos.push_back( btVector3( 210, 90, 1) );
	carPos.push_back( btVector3( 220, 101, 1) );
	carPos.push_back( btVector3( 230, 101, 1) );
	carPos.push_back( btVector3( 240, 101, 1) );
	int j = i/300;
	if( j < carPos.size())
		return carPos[j];
	else
		return btVector3( 0, 0, 0);
}
btVector3 loadCarData2(const int i )//
{
	vector< wrapper<btVector3> > carPos;
	carPos.push_back( btVector3( 10, 70, 10) );
	carPos.push_back( btVector3( 20, 80, -1) );
	carPos.push_back( btVector3( 30, 80, 1) );
	carPos.push_back( btVector3( 40, 80, 1) );
	carPos.push_back( btVector3( 50, 90, 1) );
	carPos.push_back( btVector3( 60, 80, 1) );
	carPos.push_back( btVector3( 70, 90, 1) );
	carPos.push_back( btVector3( 80, 90, 1) );
	carPos.push_back( btVector3( 90, 90, 1) );
	carPos.push_back( btVector3( 100, 90, 1) );
	carPos.push_back( btVector3( 110, 90, 1) );
	carPos.push_back( btVector3( 120, 90, 1) );
	carPos.push_back( btVector3( 130, 80, 1) );
	carPos.push_back( btVector3( 140, 70, 1) );
	carPos.push_back( btVector3( 150, 80, 1) );
	carPos.push_back( btVector3( 160, 80, 1) );
	carPos.push_back( btVector3( 170, 80, 1) );
	carPos.push_back( btVector3( 180, 80, 1) );
	carPos.push_back( btVector3( 190, 80, 1) );
	carPos.push_back( btVector3( 200, 80, 1) );
	carPos.push_back( btVector3( 210, 80, 1) );
	carPos.push_back( btVector3( 220, 80, 1) );
	carPos.push_back( btVector3( 230, 80, 1) );
	carPos.push_back( btVector3( 240, 80, 1) );
	int j = i/300;
	if( j < carPos.size())
		return carPos[j];
	else
		return btVector3( 0, 0, 0);
}

btVector3 loadManData1(const int i )//
{
	vector< wrapper<btVector3> > manPos;
	manPos.push_back( btVector3( 35, 20, 1) );
	manPos.push_back( btVector3( 35, 20, 1) );
	manPos.push_back( btVector3( 70, 40, 1) );
	manPos.push_back( btVector3( 30, 80, 1) );
	manPos.push_back( btVector3( 40, 80, 1) );
	manPos.push_back( btVector3( 50, 90, 1) );
	manPos.push_back( btVector3( 60, 80, 1) );
	manPos.push_back( btVector3( 70, 90, 1) );
	manPos.push_back( btVector3( 80, 90, 1) );
	manPos.push_back( btVector3( 90, 90, 1) );
	manPos.push_back( btVector3( 100, 90, 1) );
	manPos.push_back( btVector3( 110, 90, 1) );
	manPos.push_back( btVector3( 120, 90, 1) );
	manPos.push_back( btVector3( 130, 80, 1) );
	manPos.push_back( btVector3( 140, 70, 1) );
	manPos.push_back( btVector3( 150, 80, 1) );
	manPos.push_back( btVector3( 160, 80, 1) );
	manPos.push_back( btVector3( 170, 80, 1) );
	manPos.push_back( btVector3( 180, 80, 1) );
	manPos.push_back( btVector3( 190, 80, 1) );
	manPos.push_back( btVector3( 200, 80, 1) );
	manPos.push_back( btVector3( 210, 80, 1) );
	manPos.push_back( btVector3( 220, 80, 1) );
	manPos.push_back( btVector3( 230, 80, 1) );
	manPos.push_back( btVector3( 240, 80, 1) );
	int j = i/300;
	if( j < manPos.size())
		return manPos[j];
	else
		return btVector3( 10, 0, 1);
}

osg::MatrixTransform * createOSGBox( osg::Vec3 size )//return MatrixTransform pointer
{
	osg::Box * box = new osg::Box();//new a osg box
	box->setHalfLengths( size );
	osg::ShapeDrawable * shape = new osg::ShapeDrawable( box );
	osg::Geode * geode = new osg::Geode();
	geode->addDrawable( shape );
	osg::MatrixTransform * transform = new osg::MatrixTransform();
	transform->addChild( geode );
	return( transform );
}

btRigidBody * createBTBox( osg::MatrixTransform * box,
	osg::Vec3 center )//return the Bullet with the center as the center of mass
{
	btCollisionShape * collision = osgbCollision::btBoxCollisionShapeFromOSG( box );//create a collision shape from box
	osgbDynamics::MotionState * motion = new osgbDynamics::MotionState();
	motion->setTransform( box );
	motion->setParentTransform( osg::Matrix::translate( center ) );
	btScalar mass( 0.0 );
	btVector3 inertia( 0, 0, 0 );//
	btRigidBody::btRigidBodyConstructionInfo rb( mass, motion, collision, inertia );//init the rigidBody
	btRigidBody * body = new btRigidBody( rb );
	return( body );
}

int main(int argc, char* argv[])
{
	fstream vehicle1;
	string cars;
	vehicle1.open("vehicle1.txt", ios::in);
	if(!vehicle1)
	{
		cout<<"Error"<<endl;
		exit(1);
	}
	//vehicle1.getline(cars);

	int i = 0;
	bool terChoice = true;
	osg::ArgumentParser	arguments( &argc, argv );
	osgViewer::Viewer	viewer;	
	viewer.setUpViewInWindow( 30, 30, 768, 480 );
	osgGA::TrackballManipulator* tb = new osgGA::TrackballManipulator();
	tb->setHomePosition( 
		osg::Vec3( 90, 90, 20),	//osg::Vec3( 10., 10., 0.),
		osg::Vec3( 0, 0, 0),	//osg::Vec3( 0., 0., 1.),
		osg::Z_AXIS );	//Camera Position: From eye to center
	viewer.setCameraManipulator( tb );

	osg::ref_ptr< osg::Group > root = new osg::Group;
	viewer.setSceneData( root );

	btDynamicsWorld* dynamicsWorld = initPhysics();
	osg::ref_ptr< osg::MatrixTransform > terrain;
	if(terChoice)
	{
		terrain = createTerrain( dynamicsWorld );
		root->addChild( terrain );
	}
	else
	{
		btRigidBody* groundBody;

		float thin = .01;//
		terrain = createOSGBox( osg::Vec3( 500, 500, thin ) );//ground area
		root->addChild( terrain );
		groundBody = createBTBox( terrain, osg::Vec3( 0, 0, 0 ) );//make the ground rigid
		dynamicsWorld->addRigidBody( groundBody );
	}


	createCar car(dynamicsWorld);
	root->addChild( car.getPosition() );
	createCar car2(dynamicsWorld, btVector3( 10, 0, 2));
 	root->addChild( car2.getPosition() );

	Character* man = new Character(dynamicsWorld);
	root->addChild(man->getMan());


	viewer.realize();

 	double prevSimTime = 0.;
	while( !viewer.done() )
	{
		const double currSimTime = viewer.getFrameStamp()->getSimulationTime();
		dynamicsWorld->stepSimulation( currSimTime - prevSimTime );
		prevSimTime = currSimTime;

		car.setNextPos( loadCarData1( i ) );
		car2.setNextPos( loadCarData2( i ) );
		man->setNextPos( loadManData1(i));
		man->updateTrans();
		i++;
		viewer.frame();
		
	}
	return -1;
}