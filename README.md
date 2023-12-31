# Bullet Multilevel Projected Heightmap
This is a heightmap collision shape implementation for bullet physics.

Features:
* Multilevels (optional) - you can have caves, tunnels and floating islands by adding layers
* Projected (optional) - you can project this onto a
* Thickness - This collision shape isn't an infintely thin strip of triangles. The shape has `extraThickness` so you can force a collision with the AABB. Objects that penetrate the surface collide with a convex hull instead of a triangle.
* Optimised - Layers and triangles are aabb and height checked against incident collision objects
* Anti-Tunnelling  (optional) - Fast moving objects that penetrate the shape are teleported to the nearest surface (above/below the shape).
* Holes - You can have holes in your terrain. These holes have walls

# Why?
* Wanting holes and tunnels is a common feature wanted in heightmaps. Tunnels can be made by cutting and cover - set the heightmap to the base of the tunnel and put a second layer to make the roof of the tunnel and the original ground.
* Projection is useful for, eg, planetary surfaces. btTriangleMeshShape is overkill for this (3 verts per tri), and requires more memory and a triangle bvh.
* A heightmap, while not convex, has a clearly defined forbidden region where you don't want things to be. Anti-tunneling is conceptually simple - the forbidden zone is anything below the heightmap (or between layers of a multilevel heightmap), if something is in the forbidden zone then move it to the surface and collide as normal 

# How to use
You must patch bullet:

```sh
# Inject CUSTOM_CONCAVE_SHAPE_TYPE2, into broadphase proxy
sed -i 's/= CUSTOM_CONCAVE_SHAPE_TYPE,/\t= CUSTOM_CONCAVE_SHAPE_TYPE,\n\tCUSTOM_CONCAVE_SHAPE_TYPE2,/g' src/BulletCollision/BroadphaseCollision/btBroadphaseProxy.h

# Inject type info into 
sed -i 's/virtual void processTriangle/inline virtual bool isConvexTriangleCallback() const { return true; }\n\tvirtual void processTriangle/g' src/BulletCollision/CollisionDispatch/btConvexConcaveCollisionAlgorithm.h
sed -i 's/virtual void processTriangle/inline virtual bool isConvexTriangleCallback() const { return false; }\n\tvirtual void processTriangle/g' src/BulletCollision/CollisionShapes/btTriangleCallback.h

# Make private members "public"
sed -i 's/btVector3 m_aabbMin;/public:\n\tbtVector3 m_aabbMin;/g' src/BulletCollision/CollisionDispatch/btConvexConcaveCollisionAlgorithm.h

```

In your headers:
```c++
#define BTMH_DATA_TYPE uint16_t
#define BTMH_DATA_IS_NORMALISED_INT 1 // uint16_t is a int type! This means a value of 65354 ~= 1.0 units
#define BTMH_ANTI_TUNNELING 1
#define BTMH_HOLE_HEIGHT_VALUE 65355

// example alternative values 
//#define BTMH_DATA_TYPE btScalar
//#define BTMH_DATA_IS_NORMALISED_INT 0 // we are using a floating point type here!

#include <btMultilevelProjectedHeightmap/btMultilevelProjectedHeightmap.h>
#include <btMultilevelProjectedHeightmap/cubeSphereProjectionUtils.h>
```

In *one* implementation (`.cpp`) file
```c++
#include <btMultilevelProjectedHeightmap/btMultilevelProjectedHeightmap.cpp>
#include <btMultilevelProjectedHeightmap/cubeSphereProjectionUtils.cpp>
```
(or add the files to your build system)

# Example Collision Shape

In this example, mapData is a `uint16_t[LEAF_SIZE*LEAFSIZE]`

```c++
	void genCollisionShape(btCollisionShape*& shapeOut, btTransform& transformOut, const TerrainQuadTreeLeaf * const QuadTreeLeaf) {
		shapeOut = NULL;
		transformOut.setIdentity();
		
		btMultilevelProjectedHeightmap::sphereParams_t sp;
		if (1) { // we are making a projected cube sphere shape
			sp.r = QuadTreeLeaf->TOP->mTerrainQuadTree->worldRadius;
			sp.u = QuadTreeLeaf->faceU;
			sp.v = QuadTreeLeaf->faceV;
			sp.usz = QuadTreeLeaf->faceUSz;
			sp.vsz = QuadTreeLeaf->faceVSz;
			sp.cx = QuadTreeLeaf->TOP->mTerrainQuadTree->mFaces[0]->topLeaf->centorid.x;
			sp.cy = QuadTreeLeaf->TOP->mTerrainQuadTree->mFaces[0]->topLeaf->centorid.y;
			sp.cz = QuadTreeLeaf->TOP->mTerrainQuadTree->mFaces[0]->topLeaf->centorid.z;
			}
				
		//shapeOut = new btBvhTriangleMeshShape(triangleMeshTerrain, true);
		const TerrainQuadFace * const QuadTreeFace = QuadTreeLeaf->TOP;
		
		btVector3 xAxis;
		btVector3 zAxis;
		btVector3 heightAxis;
		bool forwardWinding;
		TerrainQuadFace::getAxes (QuadTreeFace->face, xAxis, zAxis, heightAxis, forwardWinding);
		btMatrix3x3 mtx(xAxis, heightAxis, zAxis);
		
		transformOut = btTransform(mtx.transpose(), btVector3(QuadTreeLeaf->centorid.x, QuadTreeLeaf->centorid.y, QuadTreeLeaf->centorid.z));
		
		
		const double vertScale = QuadTreeLeaf->vertScale;
		btMultilevelProjectedHeightmap::sublevel_t arr[2];
		arr[0] = btMultilevelProjectedHeightmap::sublevel_t(mapData, 0, 0, LEAF_SIZE, LEAF_SIZE, false);
		arr[1] = btMultilevelProjectedHeightmap::sublevel_t(next_layer->mapData, 0, 0, LEAF_SIZE, LEAF_SIZE, true);
		
		//shapeOut = new btMultilevelProjectedHeightmap(mapData, LEAF_SIZE, LEAF_SIZE, 2.0, vertScale, 30.0, sp); // Single Layer
		shapeOut = new btMultilevelProjectedHeightmap(arr, 2, 2.0, vertScale, 30.0, TerrainQuadFace::getWindingForPhysics(QuadTreeFace->face), sp); // Multi-layer
		}

```

# How it Works

Anti-tunneling works by checking the smallest signed distance to relevent triangles. If we are on the surface then this is zero. If we are in the surface this is non-zero, and we shift the object up or down to bring it close to the surface. Anti-tunneling is only applied if the object distance-per-tick (speed*60Hz) is greater than its aabb size, or if the object is completely embeded in the heightmap.

# Limitations & Warnings
- Y is the height axis. Rotate your shape if you want a different orientation.
- Anti-tunnelling mutates the collision object (it violates const-correctness). 
- `processAllTriangles` implements collision & anti-tunnelling using a `btConvexHullTriangleCallback` class that extends `btConvexTriangleCallback`. It detects if the passed callback is a `btConvexTriangleCallback` by calling `btConvexTriangleCallback::is

# Contact
Liam Twigger
@SnapperTheTwig

# License
MIT License

