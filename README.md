#3D Miner Visualization

To combine rigid car and kinematic character and static terrain into one.

## Terrain ##

 Convert from `.osg` file to bullet static rigidBody by `osgWorks` and `osgBullet` methods ( `osgbCollision::btCollisionFromOSG` ). Better TriMeshGrid than MeshGrid, I think.

## Vehicle Motion ##

 Create from rectangles and cylinders.
 
 Modify the origin and rotation by code step by step.

## Character Motion ##

Kinematic Character Controller can do collision detecting itself.

