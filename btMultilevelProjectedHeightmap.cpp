#include "btMultilevelProjectedHeightmap.h"
#include <BulletCollision/CollisionShapes/btTriangleShape.h>
#include <BulletCollision/CollisionDispatch/btConvexConcaveCollisionAlgorithm.h>


class btConvexHullTriangleCallback  : public btConvexTriangleCallback {
public:
	/// conservative test for overlap between 6 points and aabb
	static bool TestPoints6AgainstAabb2(const btVector3* vertices,
										const btVector3& aabbMin, const btVector3& aabbMax) {
		// there is probably a fast SIMD branchless way of doing this
		btVector3 minv = vertices[0];
		btVector3 maxv = vertices[0];
		constexpr int N_POINTS = 6;
		for (int i = 1; i < N_POINTS; ++i) {
			minv[0] = btMin(minv[0], vertices[i][0]);
			minv[1] = btMin(minv[1], vertices[i][1]);
			minv[2] = btMin(minv[2], vertices[i][2]);
			
			maxv[0] = btMax(maxv[0], vertices[i][0]);
			maxv[1] = btMax(maxv[1], vertices[i][1]);
			maxv[2] = btMax(maxv[2], vertices[i][2]);
			}
		
		return TestAabbAgainstAabb2(minv, maxv, aabbMin, aabbMax);
		}
		
	void processTriangle (btVector3* points, int partId, int triangleIndex) {
		btConvexHullTriangleCallback::processTriangleOrConvexHull(points, partId, triangleIndex, false, false);
		} 
	
	btScalar processTriangleOrConvexHull (btVector3* points, int partId, int triangleIndex, bool testTop, bool testBottom) {
		// expects 6 points
		// the convex hull is two triangles - the first triangle is the top layer, the second is the bottom
		
		// bool: testTop - if true test the top as a triangle, don't do narrowphase for the bottom
		// bool: testBottom - if true test the bottom as a triangle, don't do narrowphase for the top
		// if both are false or both are true then test a convex hull
		
		// for a single sided height map it is recommended that you make the second triangle slightly bigger than the first and some depth
		// so the object looks like an upwards pointing wedge. This is so that any penetrating collisions are "pushed" up
		
		// RETURN VALUE - the smallest (negative) penetration of a supporting vertex into a triangle
		// this is useful for nudging stuck objects out of terrain
		// If the object is penetrating the top by -1.3m and the bottom by -21.6m then -1.3m is returned
		// Returns zero if we are touching or if we are cleanly above the triangle
		// Returns BT_LARGE_FLOAT if the triangle(s) fail the AABB test or the early out test (no collision)
		
		BT_PROFILE("btConvexTriangleCallback::processTriangle");

		const bool doConvexHullTest = !(testTop xor testBottom);

		if (doConvexHullTest) {
			if (!TestPoints6AgainstAabb2(points, m_aabbMin, m_aabbMax))
				return BT_LARGE_FLOAT;
			}
		else {
			if (testTop)
				if (!TestTriangleAgainstAabb2(points, m_aabbMin, m_aabbMax))
					return BT_LARGE_FLOAT;
			if (testBottom)
				if (!TestTriangleAgainstAabb2(&points[3], m_aabbMin, m_aabbMax))
					return BT_LARGE_FLOAT;
			}

		//abort();
		btScalar smallestPenetration = 0;
#ifndef BT_DISABLE_CONVEX_CONCAVE_EARLY_OUT		
		//an early out optimisation if the object is separated from the triangle
		//projected on the triangle normal)
		{
			const btVector3 v0 = m_triBodyWrap->getWorldTransform()*points[0];
			const btVector3 v1 = m_triBodyWrap->getWorldTransform()*points[2];
			const btVector3 v2 = m_triBodyWrap->getWorldTransform()*points[1];

			btVector3 triangle_normal_world = ( v1 - v0).cross(v2 - v0);
			triangle_normal_world.normalize();

		    btConvexShape* convex = (btConvexShape*)m_convexBodyWrap->getCollisionShape();
			
			btVector3 localPt = convex->localGetSupportingVertex(m_convexBodyWrap->getWorldTransform().getBasis().inverse()*triangle_normal_world);
			btVector3 worldPt = m_convexBodyWrap->getWorldTransform()*localPt;
			//now check if this is fully on one side of the triangle
			btScalar proj_distPt = triangle_normal_world.dot(worldPt);
			btScalar proj_distTr = triangle_normal_world.dot(v0);
			btScalar contact_threshold = m_manifoldPtr->getContactBreakingThreshold()+ m_resultOut->m_closestPointDistanceThreshold;
			btScalar dist = proj_distTr - proj_distPt;
			if (dist > contact_threshold) {
				//~//Logger::ldbg("EARLY OUT ABOVE {}", dist);
				return BT_LARGE_FLOAT;
				}
			smallestPenetration = dist;
			//~//Logger::ldbg("triangle_normal_world UP: {}", btToString(triangle_normal_world)); 

			//also check the other side of the triangle
			const btVector3 v3 = m_triBodyWrap->getWorldTransform()*points[3];
			const btVector3 v4 = m_triBodyWrap->getWorldTransform()*points[5];
			const btVector3 v5 = m_triBodyWrap->getWorldTransform()*points[4];
			
			triangle_normal_world = ( v4 - v3).cross(v5 - v3);
			triangle_normal_world.normalize();

			localPt = convex->localGetSupportingVertex(m_convexBodyWrap->getWorldTransform().getBasis().inverse()*triangle_normal_world);
			worldPt = m_convexBodyWrap->getWorldTransform()*localPt;
			//now check if this is fully on one side of the triangle
			proj_distPt = triangle_normal_world.dot(worldPt);
			proj_distTr = triangle_normal_world.dot(v3);
			
			//~//Logger::ldbg("triangle_normal_world DOWN: {}", btToString(triangle_normal_world));
			//~//Logger::ldbg("Early Out Test: {:.2f} {:.2f}, thresh: {:.2f}", dist, proj_distTr - proj_distPt, contact_threshold);
			
			dist = proj_distTr - proj_distPt;
			if (dist > contact_threshold) {
				//~//Logger::ldbg("EARLY OUT BELOW {}", dist);
				return BT_LARGE_FLOAT;
				}
			
			// +ve penetration -> go up, -ve penetration -> go down
			if (dist > smallestPenetration)
				smallestPenetration = dist;
			else
				smallestPenetration = -smallestPenetration;
			//~//Logger::ldbg("smallestPenetration: {}", smallestPenetration);
        }
#endif //BT_DISABLE_CONVEX_CONCAVE_EARLY_OUT
		//static int ctr = 0;
		//static int ctr2 = 0;
		//static double triTime = 0;
		//static double hullTime = 0;
		
		// Using triangles for collisions is ~20% faster sometimes
		if (doConvexHullTest) {
			constexpr int N_POINTS = 6;
			btVector3 centre = points[0];
			for (int i = 1; i < N_POINTS; ++i)
				centre += points[i];
			centre /= N_POINTS;
			
			//double t1 = timer::getRealTime();
			btConvexHullShape chs(&points[0][0], N_POINTS);
			chs.setMargin(0*m_collisionMarginTriangle); // margin has to be zero otherwise things bounce

			processCollision(chs, partId, triangleIndex);

			//double t2 = timer::getRealTime();
			//hullTime += t2 - t1;
			//ctr2++;
			}
		else {
			//double t1 = timer::getRealTime();
			const int i = testBottom ? 3 : 0;
			btTriangleShape chs(points[0+i], points[1+i], points[2+i]);
			chs.setMargin(0*m_collisionMarginTriangle);

			processCollision(chs, partId, triangleIndex);

			//double t2 = timer::getRealTime();
			//triTime += t2 - t1;
			//ctr++;
			}
		
		//~//Logger::ldbg("Times: {:.4f}ms {:.4f}ms {} {}, hull test? {} {} {}", triTime*1000*2/ctr, hullTime*1000*2/ctr2, ctr, ctr2, doConvexHullTest, testTop, testBottom);
		return smallestPenetration;
		}
		
	void processCollision(btCollisionShape & chs, int partId, int triangleIndex) {
		btCollisionAlgorithmConstructionInfo ci;
		ci.m_dispatcher1 = m_dispatcher;

		btCollisionObjectWrapper triObWrap(m_triBodyWrap, &chs, m_triBodyWrap->getCollisionObject(), m_triBodyWrap->getWorldTransform(), partId, triangleIndex);  //correct transform?
																																								  // we should probably translate by convex hull centroid
		btCollisionAlgorithm* colAlgo = 0;

		if (m_resultOut->m_closestPointDistanceThreshold > 0)
			colAlgo = ci.m_dispatcher1->findAlgorithm(m_convexBodyWrap, &triObWrap, 0, BT_CLOSEST_POINT_ALGORITHMS);
		else
			colAlgo = ci.m_dispatcher1->findAlgorithm(m_convexBodyWrap, &triObWrap, m_manifoldPtr, BT_CONTACT_POINT_ALGORITHMS);
			
		const btCollisionObjectWrapper* tmpWrap = 0;
		if (m_resultOut->getBody0Internal() == m_triBodyWrap->getCollisionObject()) {
				tmpWrap = m_resultOut->getBody0Wrap();
				m_resultOut->setBody0Wrap(&triObWrap);
				m_resultOut->setShapeIdentifiersA(partId, triangleIndex);
				}
		else {
			tmpWrap = m_resultOut->getBody1Wrap();
			m_resultOut->setBody1Wrap(&triObWrap);
			m_resultOut->setShapeIdentifiersB(partId, triangleIndex);
			}

		{
			BT_PROFILE("processCollision (GJK?)");
			colAlgo->processCollision(m_convexBodyWrap, &triObWrap, *m_dispatchInfoPtr, m_resultOut);
		}

		if (m_resultOut->getBody0Internal() == m_triBodyWrap->getCollisionObject())
			m_resultOut->setBody0Wrap(tmpWrap);
		else
			m_resultOut->setBody1Wrap(tmpWrap);

		colAlgo->~btCollisionAlgorithm();
		ci.m_dispatcher1->freeCollisionAlgorithm(colAlgo);
		}
	};


/*

inline btScalar btMinAbs(const btScalar a, const btScalar b) {
	if (btFabs(a) < btFabs(b))
		return a;
	return b;
	}
	
inline btScalar btMinAbs(const btScalar a, const btScalar b, const btScalar c) {
	if (btFabs(a) < btFabs(b))
		return btMinAbs(a, c);
	return btMinAbs(b, c);
	}*/

inline int btMinI(const int a, const int b) {
	return a < b ? a : b;
	}
inline int btMaxI(const int a, const int b) {
	return a > b ? a : b;
	}

inline btScalar btMin(const btScalar a, const btScalar b, const btScalar c) {
	return btMin(btMin(a,b), c);
	}
inline btScalar btMax(const btScalar a, const btScalar b, const btScalar c) {
	return btMax(btMax(a,b), c);
	}

	
void btMultilevelProjectedHeightmap::sublevel_t::computeMinMaxHeight(const btMultilevelProjectedHeightmap & hm) {
	levelMinHeight = BT_LARGE_FLOAT;
	levelMaxHeight = -BT_LARGE_FLOAT;
	
	if (nSides < 2) {
		levelMinHeight = -hm.extraThickness;
		levelMaxHeight = -hm.extraThickness;
		}
	
	int sz = xs * zs * nSides;
	for (int i = 0; i < sz; ++i) {
		if (m_heightfieldData[i] == BTMH_HOLE_HEIGHT_VALUE)
			continue;
		btScalar h = getHeightAtIdx(hm, i);
		if (h < levelMinHeight) levelMinHeight = h;
		if (h > levelMaxHeight) levelMaxHeight = h;
		}
	}
	
bool btMultilevelProjectedHeightmap::sublevel_t::regionIsHoles(int xStart, int zStart, int xSz, int zSz) const {
	// Returns true iff this every grid point in a region are holes
	// note that if a grid point is a hole then all four quads touching it
	// are missing
	int is = btMaxI(xStart, 0);
	int ie = btMinI(xStart+xSz+1, xs-1);
	int js = btMaxI(zStart, 0);
	int je = btMinI(zStart+zSz+1, zs-1);
	for (int i = is; i < ie; ++i) {
		for (int j = js; j < je; ++j) {
			if (!isHoleAt(i, j)) {
				// test adjacent for holes
				if (i>0)
					if (isHoleAt(i-1, j))
						continue;
				if (j>0)
					if (isHoleAt(i, j-j))
						continue;
				if (i<xs-1)
					if (isHoleAt(i+1, j))
						continue;
				if (j<zs-1)
					if (isHoleAt(i, j+1))
						continue;
				// no holes adjacent - this is part of a surface polygon
				return false;
				}
			}
		}
	return true;
	}
			
			
// Single level constructor
btMultilevelProjectedHeightmap::btMultilevelProjectedHeightmap(const BTMH_DATA_TYPE* _m_heightfieldData, const int _xs, const int _zs, const btScalar _horzScale, const btScalar _vertScale, const btScalar _extraThickness, const bool _flippedNormals, const btMultilevelProjectedHeightmap::sphereParams_t & m_sphereSurfaceParams) :
	btConcaveShape(), baseLevel(_m_heightfieldData, 0, 0, _xs, _zs, false), horzScale(_horzScale), vertScale(_vertScale), extraThickness(_extraThickness), sphereParams(m_sphereSurfaceParams), flippedNormals(_flippedNormals) {
		m_shapeType = CUSTOM_CONCAVE_SHAPE_TYPE2;
		maxHeight = vertScale;
		
		setMargin(0.04);
		init();
		}

// Multilevel constructor, levels[0] becomes the baseLevel
btMultilevelProjectedHeightmap::btMultilevelProjectedHeightmap(btMultilevelProjectedHeightmap::sublevel_t* levels, const int nLevels, const btScalar _horzScale, const btScalar _vertScale, const btScalar _extraThickness, const bool _flippedNormals, const btMultilevelProjectedHeightmap::sphereParams_t & m_sphereSurfaceParams) :
	btConcaveShape(), baseLevel(levels[0].m_heightfieldData, 0, 0, levels[0].xs, levels[0].zs, false), horzScale(_horzScale), vertScale(_vertScale), extraThickness(_extraThickness), sphereParams(m_sphereSurfaceParams), flippedNormals(_flippedNormals) {
		m_shapeType = CUSTOM_CONCAVE_SHAPE_TYPE2;
		maxHeight = vertScale;
		
		for (int i = 1; i < nLevels; ++i)
			addSublevel(levels[i]);
		
		setMargin(0.04);
		init();
		}
	
void btMultilevelProjectedHeightmap::getSublevelsIntersecting(int x, int z, int xs, int zs, const btMultilevelProjectedHeightmap::sublevel_t** arrOut, int* idxArrOut, int & countOut) const {
	// arrOut needs to be at least sublevels.size()
	const int sz = sublevels.size();
	countOut = 0;
	for (int i = 0; i < sz; ++i) {
		//~//Logger::ldbg("test {} {} {} {} -- {} {} {} {}", sublevels[i].x, sublevels[i].z, sublevels[i].xs, sublevels[i].zs, x, z, xs, zs);
		if (sublevels[i].x > x + xs) continue;
		if (sublevels[i].z > z + zs) continue;
		if (sublevels[i].x+sublevels[i].xs < x) continue;
		if (sublevels[i].z+sublevels[i].zs < z) continue; 
		arrOut[countOut] = &sublevels[i];
		idxArrOut[countOut] = i;
		countOut++;
		}
	}
		
void btMultilevelProjectedHeightmap::init() {
	// This needs to be re-inited if more layers are added
	// set scaling factor
	if constexpr(BTMH_DATA_IS_NORMALISED_INT) {
		vertScale*=(1.0/btScalar(std::numeric_limits<BTMH_DATA_TYPE>::max()));
		}

	// compute max height
	baseLevel.computeMinMaxHeight(*this);

	maxHeight = baseLevel.levelMaxHeight;
	for (int i = 0; i < sublevels.size(); ++i) {
		sublevels[i].computeMinMaxHeight(*this);
		btScalar h = sublevels[i].levelMaxHeight;
		if (h > maxHeight) maxHeight = h;
		}

	// compute half extents & offset
	int xs = baseLevel.xs;
	int zs = baseLevel.zs;
	if (sphereParams.r > 0) {
		btVector3 testPoints[17];
		testPoints[0] = project(btVector3(0, maxHeight, 0));
		testPoints[1] = project(btVector3((xs-1)/2.0, maxHeight, 0));
		testPoints[2] = project(btVector3((xs-1), maxHeight, 0));
		testPoints[3] = project(btVector3((xs-1), maxHeight, (zs-1)/2.0));
		testPoints[4] = project(btVector3((xs-1), maxHeight, (zs-1)));
		testPoints[5] = project(btVector3((xs-1)/2.0, maxHeight, (zs-1)));
		testPoints[6] = project(btVector3(0, maxHeight, (zs-1)));
		testPoints[7] = project(btVector3(0, maxHeight, (zs-1)/2.0));
		
		testPoints[8] = project(btVector3(0, -extraThickness, 0));
		testPoints[9] = project(btVector3((xs-1)/2.0, -extraThickness, 0));
		testPoints[10] = project(btVector3((xs-1), -extraThickness, 0));
		testPoints[11] = project(btVector3((xs-1), -extraThickness, (zs-1)/2.0));
		testPoints[12] = project(btVector3((xs-1), -extraThickness, (zs-1)));
		testPoints[13] = project(btVector3((xs-1)/2.0, -extraThickness, (zs-1)));
		testPoints[14] = project(btVector3(0, -extraThickness, (zs-1)));
		testPoints[15] = project(btVector3(0, -extraThickness, (zs-1)/2.0));
		
		testPoints[16] = project(btVector3((xs-1)/2.0, maxHeight, (zs-1)/2.0));
		
		btVector3 min(BT_LARGE_FLOAT, BT_LARGE_FLOAT, BT_LARGE_FLOAT);
		btVector3 max(-BT_LARGE_FLOAT, -BT_LARGE_FLOAT, -BT_LARGE_FLOAT);
		for (uint i = 0; i < 17; ++i) {
			min = btVector3(btMin(testPoints[i].x(), min.x()), btMin(testPoints[i].y(), min.y()), btMin(testPoints[i].z(), min.z()));
			max = btVector3(btMax(testPoints[i].x(), max.x()), btMax(testPoints[i].y(), max.y()), btMax(testPoints[i].z(), max.z()));
			}
		halfExtents = (max-min)/2.0;
		internalOffset = -(max+min)/2.0;
		}
	else {
		halfExtents = btVector3(horzScale*(xs-1)/2, maxHeight/2 + extraThickness, horzScale*(zs-1)/2);	
		internalOffset = -btVector3(horzScale*(xs-1)*0.5, maxHeight/2 - extraThickness, horzScale*(zs-1)*0.5);
		}
		
	}
	
	
void btMultilevelProjectedHeightmap::calculateLocalInertia(btScalar unused, btVector3& inertia) const {
	//moving concave objects not supported
	inertia.setValue(btScalar(0.), btScalar(0.), btScalar(0.));
	}

void btMultilevelProjectedHeightmap::setLocalScaling(const btVector3& scaling) {
	// scaling not supported ( because I am lazy )
	//m_localScaling = scaling;
	}
		
const btVector3& btMultilevelProjectedHeightmap::getLocalScaling() const {
	// scaling not supported
	static btVector3 oneV(1.,1.,1.); 
	return oneV;//m_localScaling;
	}
	
const char* btMultilevelProjectedHeightmap::getName() const { return "btMultilevelProjectedHeightmap"; }
	
void btMultilevelProjectedHeightmap::getAabb(const btTransform& t, btVector3& aabbMin, btVector3& aabbMax) const {
	// we use maxHeight as extents so that there is some aabb space below the actual shape
	if (sphereParams.r > 0) {
		btTransformAabb(halfExtents, getMargin(), t, aabbMin, aabbMax);
		const btVector3 offset = t.getBasis() * internalOffset;
		aabbMin -= offset;
		aabbMax -= offset;
		}
	else
		btTransformAabb(halfExtents, getMargin(), t, aabbMin, aabbMax);
		
	//~//Logger::ldbg("getAabb {}, halfExtents: {}, aabb {} -> {}", btToString(t), btToString(halfExtents), btToString(aabbMin), btToString(aabbMax));
	}
		
	
void btMultilevelProjectedHeightmap::processAllTriangles(btTriangleCallback* callback, const btVector3& aabbMin, const btVector3& aabbMax) const {
	processAllTriangles_worker(callback, aabbMin, aabbMax, true);
	}
		
void btMultilevelProjectedHeightmap::processAllTriangles_worker(btTriangleCallback* callback, const btVector3& aabbMin, const btVector3& aabbMax, bool firstCall) const {
	btVector3 localAabbMin = aabbMin ;//- halfExtents;
	btVector3 localAabbMax = aabbMax;// + halfExtents;
	
	int xi = 0;//localAabbMin.x();
	int zi = 0;//localAabbMin.z();
	int xm = baseLevel.xs-2;//localAabbMax.x();
	int zm = baseLevel.zs-2;//localAabbMax.z();
	
	const bool isProjected = (sphereParams.r > 0);
	btVector3 localUp(0,1,0);
	
	if (aabbMin.x() > -BT_LARGE_FLOAT + 1) {
		// only fix AABB if its not huge
		if (isProjected) {
			
			// inverse project the points of the AABB to convert spherical coordinates -> heightmap coordinates
			btVector3 p[7];
			//p[0] = localAabbMin;
			p[0] = btVector3(localAabbMax.x(), localAabbMin.y(), localAabbMin.z());
			p[1] = btVector3(localAabbMin.x(), localAabbMax.y(), localAabbMin.z());
			p[2] = btVector3(localAabbMax.x(), localAabbMax.y(), localAabbMin.z());
			p[3] = btVector3(localAabbMin.x(), localAabbMin.y(), localAabbMax.z());
			p[4] = btVector3(localAabbMin.x(), localAabbMax.y(), localAabbMax.z());
			p[5] = btVector3(localAabbMax.x(), localAabbMin.y(), localAabbMax.z());
			p[6] = localAabbMax;
			
			btVector3 minv = inverseProject(localAabbMin);
			btVector3 maxv = minv;
			
			for (uint i = 0; i < 7; ++i) {
				p[i] = inverseProject(p[i]);
				minv[0] = btMin(minv[0], p[i][0]);
				minv[1] = btMin(minv[1], p[i][1]);
				minv[2] = btMin(minv[2], p[i][2]);
				maxv[0] = btMax(maxv[0], p[i][0]);
				maxv[1] = btMax(maxv[1], p[i][1]);
				maxv[2] = btMax(maxv[2], p[i][2]);
				}
			
			localAabbMin = minv;
			localAabbMax = maxv;
			
			btVector3 mid1((localAabbMin[0] + localAabbMax[0])/2, 0, (localAabbMin[2] + localAabbMax[2])/2);
			btVector3 mid2(mid1.x(), 1, mid1.z());
			localUp = (project(mid2) - project(mid1)).normalized();
			}
		else {
			const btScalar iHorzScale = 1.0/horzScale;
			localAabbMin -= internalOffset;
			localAabbMax -= internalOffset;
			localAabbMin[0] *= iHorzScale;
			localAabbMin[2] *= iHorzScale;
			localAabbMax[0] *= iHorzScale;
			localAabbMax[2] *= iHorzScale;
			// shift y to heightmap scale
			localAabbMin[1] += internalOffset[1];
			localAabbMax[1] += internalOffset[1];
			}
			
		// early out - out of bounds
		if (localAabbMax.x() < xi) return;
		if (localAabbMax.z() < zi) return;
		if (localAabbMin.x() > xm+1) return;
		if (localAabbMin.z() > zm+1) return;
		
		// Clamp to bounds
		if (localAabbMin.x() > xi)
			xi = localAabbMin.x();
		if (localAabbMin.z() > zi)
			zi = localAabbMin.z();
		if (localAabbMax.x() < xm)
			xm = localAabbMax.x();
		if (localAabbMax.z() < zm)
			zm = localAabbMax.z();
			
		//Logger::ldbg("localAabbMin {} {} {} {} [{}] [{}], input: [{}] [{}]", xi, zi, xm, zm, btToString(localAabbMin), btToString(localAabbMax), btToString(aabbMin), btToString(aabbMax));	
		//Logger::ldbg("local up {}", btToString(localUp));
		}
	
	if (localAabbMin.y() < baseLevel.levelMaxHeight && localAabbMax.y() > baseLevel.levelMinHeight) // early out height check
		if (processAllTriangles_layer(baseLevel, -1, callback, aabbMin, aabbMax, localAabbMin, localAabbMax, localUp, firstCall, false, xi, zi, xm, zm))
			return;
	
	if (sublevels.size()) {
		const btMultilevelProjectedHeightmap::sublevel_t* sublevelsIcpt[sublevels.size()];
		int layerIds[sublevels.size()];
		int sublevelCnt = 0;
		
		getSublevelsIntersecting(xi, zi, xm - xi, zm - zi, &sublevelsIcpt[0], &layerIds[0], sublevelCnt);
		
		//~//Logger::ldbg("st localAabbMin {} {} {} {} [{}] [{}], input: [{}] [{}]", xi, zi, xm, zm, btToString(localAabbMin), btToString(localAabbMax), btToString(aabbMin), btToString(aabbMax));	
		//~//Logger::ldbg("sublevels: {} {}", sublevelCnt, sublevels.size());
		
		for (int i = 0; i < sublevelCnt; ++i) {
			if (localAabbMin.y() < sublevels[i].levelMaxHeight && localAabbMax.y() > sublevels[i].levelMinHeight)
				if (processAllTriangles_layer(sublevels[i], layerIds[i], callback, aabbMin, aabbMax, localAabbMin, localAabbMax, localUp, firstCall, true, xi, zi, xm, zm))
					return;
			}
		}
	}
		
		
bool btMultilevelProjectedHeightmap::processAllTriangles_layer(const btMultilevelProjectedHeightmap::sublevel_t & sl, const int layerId, btTriangleCallback* callback, const btVector3& aabbMin, const btVector3& aabbMax, const btVector3& localAabbMin, const btVector3& localAabbMax, const btVector3& localUp, const bool firstCall, const bool isDoubleSideded, int xi, int zi, int xm, int zm) const {
	// return true to exit calling functon
	
	const bool isProjected = (sphereParams.r > 0);
	
	// winding order of triangles [0-2] is the top triangle, [3-6] is the bottom triangle
	int indices[6] = { 0, 1, 2, 3, 5, 4 };
	if (flippedNormals) {
		indices[1] = 2;
		indices[2] = 1;
		indices[4] = 4;
		indices[5] = 5;
		}
	
	const Range aabbUpRange(localAabbMin.y(), localAabbMax.y());
	
	// Anti-tunnelling
	#if BTMH_ANTI_TUNNELING
	btConvexTriangleCallback * ctcallback = callback->isConvexTriangleCallback() ?  (btConvexTriangleCallback*) callback : NULL;
	const bool doAntiTunneling = ctcallback && firstCall;
	Range topMinMaxRange(BT_LARGE_FLOAT, -BT_LARGE_FLOAT);
	Range bottomMinMaxRange(BT_LARGE_FLOAT, -BT_LARGE_FLOAT);
	btConvexHullTriangleCallback* chtc = (btConvexHullTriangleCallback*) ctcallback;
	#else
	constexpr btConvexHullTriangleCallback * chtc = NULL;
	#endif
			
	bool hasNonHoles = false; // have we checked any non-hole triangles?
	bool hasHoles = false; // we need both bools
	
	// x/z are in baseLevel coordinates	
	btScalar smallestPenetration = BT_LARGE_FLOAT;
	
	for (int x = xi; x <= xm; ++x) {
		for (int j = zi; j <= zm; ++j) {
			// Heightmap detection
			// get the triangle at
			if (sl.isHoleAt(x, j) || sl.isHoleAt(x+1, j)
				 || sl.isHoleAt(x+1, j+1) || sl.isHoleAt(x, j+1)) {
				hasHoles = true;
				continue;
				}
			hasNonHoles = true;
			
			btVector3 vertices[6];	 // position in local space
			btScalar heights[6];	 // heightmap heights (might be different to verticies.y() if projected!)
			
			if (isProjected) {
				sl.getVertexProjected(*this, x, j, vertices[indices[0]], heights[indices[0]]);
				sl.getVertexProjected(*this, x, j + 1, vertices[indices[1]], heights[indices[1]]);
				sl.getVertexProjected(*this, x + 1, j, vertices[indices[2]], heights[indices[2]]);
				
				if (isDoubleSideded) {
					sl.getVertexProjected2(*this, x, j, vertices[indices[3]], heights[indices[3]]);
					sl.getVertexProjected2(*this, x, j + 1, vertices[indices[4]], heights[indices[4]]);
					sl.getVertexProjected2(*this, x + 1, j, vertices[indices[5]], heights[indices[5]]);
					}
				else {
					sl.getVertexAtBaseProjected(*this, x, j, vertices[indices[3]], heights[indices[3]]);
					sl.getVertexAtBaseProjected(*this, x, j + 1, vertices[indices[4]], heights[indices[4]]);
					sl.getVertexAtBaseProjected(*this, x + 1, j, vertices[indices[5]], heights[indices[5]]);
					}
				}
			else {
				sl.getVertexNonProjected(*this, x, j, vertices[indices[0]]);
				sl.getVertexNonProjected(*this, x, j + 1, vertices[indices[1]]);
				sl.getVertexNonProjected(*this, x + 1, j, vertices[indices[2]]);
				
				if (isDoubleSideded) {
					sl.getVertexNonProjected2(*this, x, j, vertices[indices[3]]);
					sl.getVertexNonProjected2(*this, x, j + 1, vertices[indices[4]]);
					sl.getVertexNonProjected2(*this, x + 1, j, vertices[indices[5]]);
					}
				else {
					sl.getVertexAtBaseNonProjected(*this, x, j, vertices[indices[3]]);
					sl.getVertexAtBaseNonProjected(*this, x, j + 1, vertices[indices[4]]);
					sl.getVertexAtBaseNonProjected(*this, x + 1, j, vertices[indices[5]]);
					}
				
				for (int i = 0; i < 6; ++i) {
					heights[indices[i]] = vertices[indices[i]].y();
					}
				}
			
			// Skip triangle processing if the triangle is out-of-AABB.
			Range topRange = Range::minmaxRange(heights[0], heights[1], heights[2]);
			Range bottomRange = Range::minmaxRange(heights[3], heights[4], heights[5]);
			Range pillarRange = Range::merge(topRange, bottomRange);
			
			// range check is not needed as the callback does an AABB test
			if (pillarRange.overlaps(aabbUpRange)) {
				if (chtc) {
					btScalar p = chtc->processTriangleOrConvexHull(vertices, 2 * x, j, aabbUpRange.getMidpoint() > topRange.getMidpoint(), aabbUpRange.getMidpoint() < bottomRange.getMidpoint() );
					if (btFabs(p) < btFabs(smallestPenetration))
						smallestPenetration = p;
					}
				else {
					callback->processTriangle(vertices, 2 * x, j);
					callback->processTriangle(&vertices[3], 2 * x, j);
					}
				}

			vertices[indices[0]] = vertices[indices[2]];
			vertices[indices[3]] = vertices[indices[5]];
			heights[indices[0]] = heights[indices[2]];
			heights[indices[3]] = heights[indices[5]];
			
			#if BTMH_ANTI_TUNNELING
			if (doAntiTunneling) {
				//~//Logger::ldbg("verts [{}] [{}]", btToString(vertices[indices[0]]), btToString(vertices[indices[3]]));
				//~//Logger::ldbg("heights: {:.2f} {:.2f} {:.2f} {:.2f} {:.2f} {:.2f}", heights[0], heights[1], heights[2], heights[3], heights[4], heights[5]);
				}
			#endif
			
			if (isProjected) {
				sl.getVertexProjected(*this, x + 1, j + 1, vertices[indices[2]], heights[indices[2]]);
				
				if (isDoubleSideded)
					sl.getVertexProjected2(*this, x + 1, j + 1, vertices[indices[5]], heights[indices[5]]);
				else
					sl.getVertexAtBaseProjected(*this, x + 1, j + 1, vertices[indices[5]], heights[indices[5]]);
				}
			else {
				sl.getVertexNonProjected(*this, x + 1, j + 1, vertices[indices[2]]);
				
				if (isDoubleSideded)
					sl.getVertexNonProjected2(*this, x + 1, j + 1, vertices[indices[5]]);
				else
					sl.getVertexAtBaseNonProjected(*this, x + 1, j + 1, vertices[indices[5]]);
				
				heights[indices[2]] = vertices[indices[2]].y();
				heights[indices[5]] = vertices[indices[5]].y();
				}

			topRange.min = btMin(topRange.min, heights[indices[2]]);
			topRange.max = btMax(topRange.max, heights[indices[2]]);
			bottomRange.min = btMin(bottomRange.min, heights[indices[5]]);
			bottomRange.max = btMax(bottomRange.max, heights[indices[5]]);
			pillarRange = Range::merge(topRange, bottomRange);
			
			#if BTMH_ANTI_TUNNELING
			if (doAntiTunneling) {
				topMinMaxRange.min = btMin(topRange.min, topMinMaxRange.min);
				topMinMaxRange.max = btMax(topRange.max, topMinMaxRange.max);
				
				bottomMinMaxRange.min = btMin(bottomRange.min, bottomMinMaxRange.min);
				bottomMinMaxRange.max = btMax(bottomRange.max, bottomMinMaxRange.max);
				}
			#endif
				
			// range check is not needed as the callback does an AABB test
			if (pillarRange.overlaps(aabbUpRange)) {
				if (chtc) {
					btScalar p = chtc->processTriangleOrConvexHull(vertices, 2 * x + 1, j, aabbUpRange.getMidpoint() > topRange.getMidpoint(), aabbUpRange.getMidpoint() < bottomRange.getMidpoint());
					if (btFabs(p) < btFabs(smallestPenetration))
						smallestPenetration = p;
					}
				else {
					callback->processTriangle(vertices, 2 * x + 1, j);
					callback->processTriangle(&vertices[3], 2 * x + 1, j);
					}
				}
			
			}
		}
	
	//if (ctcallback)
	//	//~//Logger::ldbg("Range: {} {}, aabb In: {:.2f}, {:.2f}, {:.2f}", topMinMaxRange.min, topMinMaxRange.max, ctcallback->m_aabbMin.y(), localAabbMin.y(), aabbMin.y());
	
	#if BTMH_ANTI_TUNNELING
	// Only do anti-tunneling if we are stuck in the shape + have actually tested triangles (hasNonHoles) and are not intersecting a gap (hasHoles)
	if (doAntiTunneling && hasNonHoles && !hasHoles) {
		// Are we stuck colliding with the wrong side of a triangle?
		Range totalColumnRange = Range::merge(topMinMaxRange, bottomMinMaxRange);
		
		// Checking margin == this margin + other object's margin
		const btScalar effectiveMargin = getMargin()*2;
		
		// Small penetration test - use the depth info from the early-out check in the triangle collision callback to get
		// some idea on how much we're stuck in the ground
		bool smallPenetrationTest = (btFabs(smallestPenetration) > effectiveMargin) && (btFabs(smallestPenetration) < totalColumnRange.max-totalColumnRange.min);
		//smallPenetrationTest = false;
		
		// AABB Test - An object is assumed to have clipped if its aabb.min is below the triangleAabb.min
		// (and vice verse for the bottom side)
		// NOTE - This doesn't work if the object
		#if 0
		bool aabbTest = ((localAabbMin.y() < topMinMaxRange.min - effectiveMargin) && (localAabbMax.y() > bottomMinMaxRange.max + effectiveMargin));
		aabbTest = false; // aabbTest is bad - there might be a large empty space around an object
			
		// Are we intesecting both top and bottom?
		// If we are don't aabb eject
		if (topMinMaxRange.overlaps(aabbUpRange) && bottomMinMaxRange.overlaps(aabbUpRange)) {
			// only eject in the direction that has the smallest ejection
			// Are the sides close together?
			aabbTest = false;
			}
		#else
		constexpr bool aabbTest = false;
		#endif
		
		// Strict AABB Test - If an object's aabb is completely inside the heightmap aabb then we're really stuck
		bool strictAabbTest = ((localAabbMax.y() < topMinMaxRange.min) && (localAabbMin.y() > bottomMinMaxRange.max));
		
		//Logger::ldbg("aabbTest: {} {} {}", aabbTest, strictAabbTest, smallPenetrationTest);
		//Logger::ldbg("!!!! aabbTest {} {} {}" , aabbTest, topMinMaxRange.min - localAabbMin.y() - effectiveMargin, bottomMinMaxRange.max - localAabbMax.y() + effectiveMargin);
		//Logger::ldbg("!!!! smallestPenetation {} {} {}" , smallPenetrationTest, smallestPenetration, effectiveMargin);
		//aabbTest = false;
		
		if (smallPenetrationTest || aabbTest || strictAabbTest) { 
			btScalar yShift = topMinMaxRange.min - localAabbMin.y() - effectiveMargin;
			
			// !!! Very naughty const_cast !!! This makes this not thread-safe!
			btCollisionObject* co = const_cast<btCollisionObject*>(ctcallback->m_convexBodyWrap->getCollisionObject());
			btRigidBody* rb = btRigidBody::upcast(co);
			
			// Only anti-tunnel if we're moving fast (relative to aabb size) or stopped
			if (rb && !strictAabbTest) {
				const btScalar v2 = rb->getLinearVelocity().length2();
				if (!((v2 / 3600.0) > (localAabbMin - localAabbMax).length2() || v2 < 0.0000001))
					return false;
				}
				
				
			const btMatrix3x3 thisMat = (ctcallback->m_triBodyWrap->m_worldTransform.getBasis());
			
			// what is the shift? we need the move the collision object AABB to the surface
			// NOTE- this is bad as when you try to move out of a stuck position you get pulled in
			//btVector3 dir = rb ? -rb->getLinearVelocity()*thisMat : btVector3(0,1,0); // -v*M.inverse() -> direction opposite to incident direction
			
			// distance to surface
			btVector3 yShiftV(0, yShift, 0);
			btVector3 yShiftVL; // for debugging
			//bool kill = false;
			
			
			// Eject to the top or bottom
			if (aabbTest || strictAabbTest) {
				// aabb ejection (coarse adjustment, usually good enough)
				if (isDoubleSideded && (aabbUpRange.getMidpoint() < 0.5*(topMinMaxRange.min + bottomMinMaxRange.max)) ) {
					// Eject down
					yShift = bottomMinMaxRange.max - localAabbMax.y();
					yShiftV = localUp * yShift;
					}
				else {
					// Eject up
					yShiftV = localUp * yShift;
					}
				}
			if (smallPenetrationTest && (!aabbTest || btFabs(smallestPenetration) > btFabs(yShift*2))) {
				// small penetration ejection (fine adjustment, catches some situations where we're on a steep slope or the object is smaller than the triangle)
				yShift = smallestPenetration / 2;
				yShiftV = localUp * yShift;
				}
				
				
			#if 0
			// Eject to the top or sides.
			// This works okay for the non-projected singles sided case
			if (yShift < topAabbUpRange.max - topAabbUpRange.min) {
				// Ejecting to the top as we're near the top
				yShiftV = localUp * yShift;
				}
			else {
				// Ejecting to the nearest side or top
				// with convex hull collisions this is not necessary
				int x = (xi + xm)/2;
				int j = (zi + zm)/2;
				// We could be entering the AABB of the heightmap from the underside
				// or if this is a multilevel heightmap we could be on the side of a piece
				
				// search radius - we should only look in the radius limited by the x,z distance to the edge
				// saves expensive long search lookups
				int rLimit = btMinI(btMinI(xs-1 - xm, xs), btMinI(zs-1 - zm, zs));
				
				////////////////////////////////////////////////////////
				//     | iAABB |          ^ x
				// xx  +-------+          | 
				//     .       .          +--> z
				//     V zi    V zm     hole checking region
				
				// TBD - *holes must fit aabb shape* - we are only checking the corners of the AABB!
				// for small objects (less than 2x2 coordinates (4x4 triangles) in size this is correct)
				
				int ctr = 1;
				int xf = xs-1;
				int xLimitH = btMinI(xs - 1, xm + rLimit);
				for (int xx = xi; xx <= xLimitH; ++xx) {
					ctr++;
					if ((isHoleAt(xx, zi) || (zi < zs-1 && isHoleAt(xx, zi+1)) || (zi > 0 && isHoleAt(xx, zi-1)))
						&& (zi == zm || isHoleAt(xx, zm) || (zm < zs-1 && isHoleAt(xx, zm+1)) || (zm > 0 && isHoleAt(xx, zm-1)))) {
						xf = xx-2; // TODO- Check that hole fits object!
								   // We set the position to (xx-2) as we
								   // *want* to collide with the non-null triangles here
						break;
						}
					}
				
				rLimit = btMinI(rLimit, ctr); // reduce the search radius if we have detected a hole nearby above
												// if was smart I'll be able to solve for ctr analytically
												// ctr starts at 1 because the projected coordinates might by non-linear
				ctr = 1;
				int xLimitL = btMaxI(0, xi - rLimit);
				int xb = 0;
				for (int xx = xm; xx >= xLimitL; --xx) {
					ctr++;
					if ((isHoleAt(xx, zi) || (zi < zs-1 && isHoleAt(xx, zi+1)) || (zi > 0 && isHoleAt(xx, zi-1)))
						&& (zi == zm || isHoleAt(xx, zm) || (zm < zs-1 && isHoleAt(xx, zm+1)) || (zm > 0 && isHoleAt(xx, zm-1)))) {
						xb = xx+2; // manually tuned to work
						break;
						}
					}
					
				rLimit = btMinI(rLimit, ctr);
				ctr = 1;
				int zLimitH = btMinI(zs - 1, zm + rLimit);
				int zf = zs - 1;
				for (int zz = zm; zz <= zLimitH; ++zz) {
					ctr++;
					if ((isHoleAt(xi, zz) || (xi < xs-1 && isHoleAt(xi+1, zz)) || (xi > 0 && isHoleAt(xi-1, zz)))
						&& (xi == xm || isHoleAt(xm, zz) || (xm < xs-1 && isHoleAt(xm+1, zz)) || (xm > 0 && isHoleAt(xm-1, zz)))) {
						zf = zz-2;
						break;
						}
					}
					
				rLimit = btMinI(rLimit, ctr);
				ctr = 1;
				int zLimitL = btMaxI(0, zi - rLimit);
				int zb = 0;
				for (int zz = zi; zz >= zLimitL; --zz) {
					ctr++;
					if ((isHoleAt(xi, zz) || (xi < xs-1 && isHoleAt(xi+1, zz)) || (xi > 0 && isHoleAt(xi-1, zz)))
						&& (xi == xm || isHoleAt(xm, zz) || (xm < xs-1 && isHoleAt(xm+1, zz)) || (xm > 0 && isHoleAt(xm-1, zz)))) {
						zb = zz+2;
						break;
						}
					}
					
				// diagonal tests
				// these tests are for the special case where an object is wegded in a corner somehow
				// diagonal ejections tend to be buggy
				int diagHHi = xs+1; // set to a large, invalid value that is larger than the heightmap
				int diagHLi = xs+1;
				int diagLLi = xs+1;
				int diagLHi = xs+1;
					
				rLimit = btMinI(rLimit, ctr);
				ctr = 1;
				for (int i = 0; i <= rLimit; ++i) {
					ctr++;
					//Logger::ldbg("Scanning {} {}, sz {} {}", xi+i, zi+i, xm-xi, zm-zi);
					if (regionIsHoles(xi+i, zi+i, xm-xi, zm-zi)) {
						diagHHi = i-1; // we WANT to intersect the non-hole triangles
						//Logger::ldbg("holes found at: {}", i);
						break;
						}
					}
				
				rLimit = btMinI(rLimit, ctr);
				ctr = 1;
				for (int i = 0; i <= rLimit; ++i) {
					ctr++;
					if (regionIsHoles(xi+i, zi-i, xm-xi, zm-zi)) {
						diagHLi = i-1;
						break;
						}
					}
				rLimit = btMinI(rLimit, ctr);
				ctr = 1;
				for (int i = 0; i <= rLimit; ++i) {
					ctr++;
					//Logger::ldbg("Scanning {} {}, sz {} {}", xi+i, zi+i, xm-xi, zm-zi);
					if (regionIsHoles(xi-i, zi-i, xm-xi, zm-zi)) {
						diagLLi = i-1;
						//Logger::ldbg("holes found at: {}", i);
						break;
						}
					}
				rLimit = btMinI(rLimit, ctr);
				ctr = 1;
				for (int i = 0; i <= rLimit; ++i) {
					ctr++;
					if (regionIsHoles(xi-i, zi+i, xm-xi, zm-zi)) {
						diagLHi = i-1;
						break;
						}
					}
				
				
				//~//Logger::ldbg("x,j {} {}", x, j);
				//~//Logger::ldbg("xi,xm {} {} {} {}", xi, xm, zi, zm);
				//~//Logger::ldbg("xb, etc: {} {} {} {} r: {}, diags: {} {} {} {}", xb, xf, zb, zf, rLimit, diagHHi, diagHLi, diagLLi, diagLHi);
				
				btVector3 xH, xL, zH, zL, diagHHV, diagHLV, diagLLV, diagLHV;
				
				if (isProjected) {
					sl.getVertexProjected(*this, xf, j, xH);
					sl.getVertexProjected(*this, xb, j, xL);
					sl.getVertexProjected(*this, x, zf, zH);
					sl.getVertexProjected(*this, x, zb, zL);
					if (diagHHi < xs) sl.getVertexAtZeroProjected(*this, xi+diagHHi, zi+diagHHi, diagHHV);
					if (diagHLi < xs) sl.getVertexAtZeroProjected(*this, xi+diagHLi, zm-diagHLi, diagHLV);
					if (diagLLi < xs) sl.getVertexAtZeroProjected(*this, xm-diagLLi, zm-diagLLi, diagLLV);
					if (diagLHi < xs) sl.getVertexAtZeroProjected(*this, xm-diagLHi, zi+diagLHi, diagLHV);
					}
				else {
					sl.getVertexNonProjected(*this, xf, j, xH);
					sl.getVertexNonProjected(*this, xb, j, xL);
					sl.getVertexNonProjected(*this, x, zf, zH);
					sl.getVertexNonProjected(*this, x, zb, zL);
					if (diagHHi < xs) sl.getVertexAtZeroNonProjected(*this, xi+diagHHi, zi+diagHHi, diagHHV);
					if (diagHLi < xs) sl.getVertexAtZeroNonProjected(*this, xi+diagHLi, zm-diagHLi, diagHLV);
					if (diagLLi < xs) sl.getVertexAtZeroNonProjected(*this, xm-diagLLi, zm-diagLLi, diagLLV);
					if (diagLHi < xs) sl.getVertexAtZeroNonProjected(*this, xm-diagLHi, zi+diagLHi, diagLHV);
					}
				
				//btScalar xHDist = xH.x() - (ctcallback->m_aabbMax.x()+3*ctcallback->m_aabbMin.x())/4;
				//btScalar zHDist = zH.z() - (ctcallback->m_aabbMax.z()+3*ctcallback->m_aabbMin.z())/4;
				//btScalar xLDist = -( xL.x() - (ctcallback->m_aabbMax.x()*3+ctcallback->m_aabbMin.x())/4 );
				//btScalar zLDist = -( zL.z() - (ctcallback->m_aabbMax.z()*3+ctcallback->m_aabbMin.z())/4 );
				
				btScalar xHDist = xH.x() - ctcallback->m_aabbMin.x();
				btScalar zHDist = zH.z() - ctcallback->m_aabbMin.z();
				btScalar xLDist = -( xL.x() - ctcallback->m_aabbMax.x() );
				btScalar zLDist = -( zL.z() - ctcallback->m_aabbMax.z() );
				
				if (diagHHi < xs) diagHHV = btVector3(diagHHV.x() - ctcallback->m_aabbMin.x(), 0, diagHHV.z() - ctcallback->m_aabbMin.z());
				if (diagHLi < xs) diagHLV = btVector3(diagHLV.x() - ctcallback->m_aabbMin.x(), 0, diagHLV.z() - ctcallback->m_aabbMax.z());
				if (diagLLi < xs) diagLLV = btVector3(diagLLV.x() - ctcallback->m_aabbMax.x(), 0, diagLLV.z() - ctcallback->m_aabbMax.z());
				if (diagLHi < xs) diagLHV = btVector3(diagLHV.x() - ctcallback->m_aabbMax.x(), 0, diagLHV.z() - ctcallback->m_aabbMin.z());
				
				//~//Logger::ldbg("ejection amounts {} {} {} {} , y: {}, diag ej: {} {} {} {}", xLDist, xHDist, zLDist, zHDist, yShift,
							(diagHHi < xs) ? diagHHV.length() : 0,
							(diagHLi < xs) ? diagHLV.length() : 0,
							(diagLLi < xs) ? diagLLV.length() : 0,
							(diagLHi < xs) ? diagLHV.length() : 0
							);
				
				// eject to the closest side
				// which is the smallest of 5?
				btScalar minVal = yShift;
				int idx = 0;
				if (xHDist < minVal) { idx = 1; minVal = xHDist; }
				if (zHDist < minVal) { idx = 2; minVal = zHDist; }
				if (xLDist < minVal) { idx = 3; minVal = xLDist; }
				if (zLDist < minVal) { idx = 4; minVal = zLDist; }
				if (diagHHi < xs && diagHHV.x() >= 0 && diagHHV.z() >= 0 && diagHHV.length2() < minVal*minVal) { idx = 5; minVal = diagHHV.length(); }
				if (diagHLi < xs && diagHLV.x() >= 0 && diagHLV.z() <= 0 && diagHLV.length2() < minVal*minVal) { idx = 6; minVal = diagHLV.length(); }
				if (diagLLi < xs && diagLLV.x() <= 0 && diagLLV.z() <= 0 && diagLLV.length2() < minVal*minVal) { idx = 7; minVal = diagLLV.length(); }
				if (diagLHi < xs && diagLHV.x() <= 0 && diagLHV.z() >= 0 && diagLHV.length2() < minVal*minVal) { idx = 8; minVal = diagLHV.length(); }
				
				//~//Logger::ldbg("diagLLV.x, z, {} {}", diagLLV.x(), diagLLV.z()); 
				
				switch (idx) {
					case 1:
						yShiftV = btVector3(xHDist, 0, 0);
						break;
					case 2:
						yShiftV = btVector3(0, 0, zHDist);
						break;
					case 3:
						yShiftV = btVector3(-xLDist, 0, 0);
						break;
					case 4:
						yShiftV = btVector3(0, 0, -zLDist);
						break;
					case 5:
						yShiftV = diagHHV;
						break;
					case 6:
						yShiftV = diagHLV;
						break;
					case 7:
						yShiftV = diagLLV;
						break;
					case 8:
						yShiftV = diagLHV;
						break;
					case 0:
					default:
						yShiftV = btVector3(0, yShift, 0);
					}
				
				//return; // view ejections without applying them
				//~//Logger::ldbg("mode {}, yShiftV {}", idx, btToString(yShiftV));
				//if (idx != 7) return; // test diagonals
				if (minVal < 0) return; // do not shift if the required shift is negative
				//if (idx != 0)
				//	kill = true;
				}
			#endif				
			// Move the object so its no longer underground
			const btMatrix3x3 thisMatI = thisMat.inverse();
			yShiftVL = yShiftV;
			yShiftV = yShiftV*thisMatI;
				
			ctcallback->m_aabbMin += yShiftV;
			ctcallback->m_aabbMax += yShiftV;
			
			
			//~//Logger::ldbg("yShift {}", yShift);
			//Logger::ldbg("DIR!!! {}", btToString(dir));
			//~//Logger::ldbg("MOVE!!! [{}] [{}]", btToString(yShiftVL), btToString(yShiftV));
			//~//Logger::ldbg("ctc_aabb: [{}] \t[{}]", btToString(ctcallback->m_aabbMin), btToString(ctcallback->m_aabbMax));
			
			btTransform & t = co->getWorldTransform();
			t.setOrigin(t.getOrigin() + yShiftV);
			
			// Now that we've teleported to the surface do a proper collision check
			//processAllTriangles_worker(ctcallback, ctcallback->m_aabbMin, ctcallback->m_aabbMax, false);
			return false;
			}
							
		//~//Logger::ldbg("ctc_aabb: {} \t{}", btToString(ctcallback->m_aabbMin), btToString(ctcallback->m_aabbMax));
		}
	#endif
	return false;
	}
	
btVector3 btMultilevelProjectedHeightmap::project(const btVector3 & pos) const {
	// converts a heightfield space coordinate into a spherical surface coordinate
	// coordinate system: pos.y is up
	if (sphereParams.r > 0) {
		// we are using doubles for precision
		double u = 2.0*pos.x()*sphereParams.usz/(baseLevel.xs-1);
		double v = 2.0*pos.z()*sphereParams.vsz/(baseLevel.zs-1);
		u += sphereParams.u*2.0 - 1.0;
		v += sphereParams.v*2.0 - 1.0;
		
		double h = 1.0; // height = height from surface of cube. Cube surface is at (0,1,0);
		double x,y,z;
		
		CubeSphereProjectionUtils::cubeToSphere(u,h,v,x,y,z);
		
		double rr = double(sphereParams.r) + pos.y();
		double ox = rr*x - sphereParams.cx;
		double oy = rr*y - sphereParams.cy; 
		double oz = rr*z - sphereParams.cz;
		
		return btVector3(ox, oy, oz);
		}
	return pos;
	}
	
btVector3 btMultilevelProjectedHeightmap::inverseProject(const btVector3 & pos) const {
	// converts spherical surface coordinates to heightfield space coordinates
	if (sphereParams.r > 0) {
		// calculations in doubles for precision
		double u,h,v;
		double px,py,pz;
		px = pos.x() + sphereParams.cx;
		py = pos.y() + sphereParams.cy;
		pz = pos.z() + sphereParams.cz;
		double r = sqrt(px*px + py*py + pz*pz);
		px /= r;
		py /= r;
		pz /= r;
		
		CubeSphereProjectionUtils::sphereToCubeSimple(px, py, pz, u,h,v);
		
		u += 1 - 2*sphereParams.u;
		v += 1 - 2*sphereParams.v;
		u = u*(baseLevel.xs-1)/(2.0*sphereParams.usz);
		v = v*(baseLevel.zs-1)/(2.0*sphereParams.vsz);
		return btVector3(u,r - sphereParams.r,v);
		}
	return pos;
	}


