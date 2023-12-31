#ifndef HEADER_btMultilevelProjectedHeightmap_h
#define HEADER_btMultilevelProjectedHeightmap_h

// Defines
#ifndef BTMH_DATA_TYPE
	#error Please define macro BTMH_DATA_TYPE -- \
example: "#define BTMH_DATA_TYPE btScalar" \
or "#define BTMH_DATA_TYPE uint16_t"
#endif

#ifndef BTMH_DATA_IS_NORMALISED_INT
	#error Please define macro BTMH_DATA_IS_NORMALISED_INT -- \
example: if BTMH_DATA_TYPE is float, #define BTMH_DATA_IS_INT 0 \
or if BTMH_DATA_TYPE is uint16_t, #define BTMH_DATA_IS_NORMALISED_INT 1 
#endif

#ifndef BTMH_HOLE_HEIGHT_VALUE
	#error Please define macro BTMH_HOLE_HEIGHT_VALUE to some value (such as 0, 65355, etc).\
This is the hightmap value that denotes an invalid vertex (a hole in the map) 
#endif

#ifndef BTMH_ANTI_TUNNELING
	#error Please define macro BTMH_ANTI_TUNNELING to 0 to disable, 1 to enable
#endif

#include <BulletCollision/CollisionShapes/btHeightfieldTerrainShape.h>

// Classes
class btMultilevelProjectedHeightmap : public btConcaveShape {
public:
	struct sphereParams_t {
		// if this heightmap part of the surface of a projected-cube-sphere?
		// set r = 0 to not use projection
		// r is the radius of the sphere, u,v (0-1) is the uv coordinates along the face (x = u, z = v), (usz,vsz) is the size of the section
		btScalar r,u,v,usz,vsz;
		double cx, cy, cz;	// offset from the centre of the sphere to here
		inline sphereParams_t() : r(0), u(0), v(0), usz(0), vsz(0), cx(0), cy(0), cz(0) {}
		};
	
	struct Range {
		// local implementation of btHeightfieldTerrainShape::Range
		btScalar min;
		btScalar max;
		
		inline Range(btScalar _min, const btScalar _max) : min(_min), max(_max) {}
		
		inline static btMultilevelProjectedHeightmap::Range merge(const Range & a, const Range & b) {
			return Range(btMin(a.min, b.min), btMax(a.max, b.max));
			}
		
		inline static btMultilevelProjectedHeightmap::Range minmaxRange(btScalar a, btScalar b, btScalar c) {
			if (a > b) {
				if (b > c)
					return Range(c, a);
				else if (a > c)
					return Range(b, a);
				else
					return Range(b, c);
				}
			else {
				if (a > c)
					return Range(c, b);
				else if (b > c)
					return Range(a, b);
				else
					return Range(a, c);
				}
			}

		inline bool overlaps(const Range & other) const {
				return !(min > other.max || max < other.min);
			}
			
		inline btScalar getMidpoint() const {
			return (min+max)/2;
			}
			
		inline static bool isBetwenMidpoints(const Range & upper, const Range & lower, const btScalar test) {
			return (test < upper.getMidpoint() && test > lower.getMidpoint());
			}
		};
	
	struct sublevel_t {
		const BTMH_DATA_TYPE* m_heightfieldData; // data layout: { top0, bottom0, top1, bottom1, ...etc }
		int x, z;	// relative to (0,0) of the parent
		int xs, zs;
		int nSides; // 1 = single sided, 2 = double sided
		btScalar levelMinHeight, levelMaxHeight; // cached height values. Used to early-out a collision
		
		inline sublevel_t() : m_heightfieldData(NULL), x(0), z(0), xs(0), zs(0), nSides(1), levelMinHeight(-BT_LARGE_FLOAT), levelMaxHeight(BT_LARGE_FLOAT) {} // don't use this. Not having it be default-constructable can make this have issues in containers
		inline sublevel_t(const BTMH_DATA_TYPE* _m_heightfieldData, const int _x, const int _z, const int _xs, const int _zs, const bool isDoubleSided)
			: m_heightfieldData(_m_heightfieldData), x(_x), z(_z), xs(_xs), zs(_zs), nSides(isDoubleSided ? 2 : 1), levelMinHeight(-BT_LARGE_FLOAT), levelMaxHeight(BT_LARGE_FLOAT) {}
		
		inline uint32_t idxAt(int i, int j, int layer) const {
			return (i*zs + j) * nSides + layer;
			}
			
		void computeMinMaxHeight(const btMultilevelProjectedHeightmap & hm);
		
		inline btScalar getHeightAtIdx (const btMultilevelProjectedHeightmap & hm, int idx) const {
			#if 0
				#warning btMultilevelProjectedHeightmap::sublevel_t range checking is on
				if (idx >= xs * zs * nSides) abort(); // Test out-of range errors
			#endif
			BTMH_DATA_TYPE h = m_heightfieldData[idx];
			return hm.vertScale*btScalar(h);
			}
			
		inline btScalar getHeightAt (const btMultilevelProjectedHeightmap & hm, int i, int j, int layer = 0) const {
			return getHeightAtIdx(hm, idxAt(i, j, layer));
			}
			
		inline bool isHoleAtIdx(int idx) const {
			return (m_heightfieldData[idx] == BTMH_HOLE_HEIGHT_VALUE);
			}
		inline bool isHoleAt (int i, int j) const {
			return isHoleAtIdx(idxAt(i, j, 0));
			}
		
		// used for top of shape
		// returns the (x, height, z) coordinate
		inline void getVertexNonProjected(const btMultilevelProjectedHeightmap & hm, int i, int j, btVector3& out) const {
			out = btVector3((i-x)*hm.horzScale, getHeightAt(hm, i,j, 0), (j-z)*hm.horzScale) + hm.internalOffset;
			}
		inline void getVertexProjected(const btMultilevelProjectedHeightmap & hm, int i, int j, btVector3& out, btScalar & height) const {
			const btVector3 v(i-x, getHeightAt(hm, i,j, 0), j-z);
			height = v.y();
			out = hm.project(v);
			}
			
		// returns the (x, height2, z) coordinate (bottom layer of a double sided level)
		inline void getVertexNonProjected2(const btMultilevelProjectedHeightmap & hm, int i, int j, btVector3& out) const {
			out = btVector3((i-x)*hm.horzScale, getHeightAt(hm, i,j, 1), (j-z)*hm.horzScale) + hm.internalOffset;
			}
		inline void getVertexProjected2(const btMultilevelProjectedHeightmap & hm, int i, int j, btVector3& out, btScalar & height) const {
			const btVector3 v(i-x, getHeightAt(hm, i,j, 1), j-z);
			height = v.y();
			out = hm.project(v);
			}
			
		// used for edge of shape
		// returns the (x, -extraThickness, z) coordinate
		inline void getVertexAtBaseNonProjected(const btMultilevelProjectedHeightmap & hm, int i, int j, btVector3& out) const {
			out = btVector3((i-x)*hm.horzScale, -hm.extraThickness, (j-z)*hm.horzScale) + hm.internalOffset;
			}
		inline void getVertexAtBaseProjected(const btMultilevelProjectedHeightmap & hm, int i, int j, btVector3& out, btScalar & height) const {
			height = -hm.extraThickness;
			out = hm.project(btVector3((i-x), height, (j-z)));
			}
		
		// used to get the coordinates of the base of holes
		// returns the (x, 0, z) coordinate
		inline void getVertexAtZeroNonProjected(const btMultilevelProjectedHeightmap & hm, int i, int j, btVector3& out) const {
			out = btVector3((i-x)*hm.horzScale, 0, (j-z)*hm.horzScale) + hm.internalOffset;
			}
		inline void getVertexAtZeroProjected(const btMultilevelProjectedHeightmap & hm, int i, int j, btVector3& out, btScalar & height) const {
			height = 0;
			out = hm.project(btVector3(i-x, 0, j-z));
			}
			
		// Returns true iff this every grid point in a region are holes
		// note that if a grid point is a hole then all four quads touching it
		// are missing
		bool regionIsHoles(int xStart, int zStart, int xSz, int zSz) const;
		
		};
	
	sublevel_t baseLevel;
	btScalar horzScale; // is ignored if this is a sphere surface section. instead r is used
	btScalar vertScale;
	btScalar maxHeight; // The maximum height. This is calculated internally by init();
	btScalar extraThickness; // how much thickness is added to the aabb on the y axis below the shape? This is used to detect tunneling through the shape
	btVector3 internalOffset; // two meanings depending if this is projected or not
	btVector3 halfExtents;
	
	// note: the "centre" of the height map is at (xs, maxHeight/2, zs)
	btAlignedObjectArray<btMultilevelProjectedHeightmap::sublevel_t> sublevels;
	btMultilevelProjectedHeightmap::sphereParams_t sphereParams;
	bool flippedNormals;
	
	// Single level constructor
	btMultilevelProjectedHeightmap(const BTMH_DATA_TYPE* _m_heightfieldData, const int _xs, const int _zs, const btScalar _horzScale, const btScalar _vertScale, const btScalar _extraThickness, const bool _flippedNormals, const btMultilevelProjectedHeightmap::sphereParams_t & m_sphereSurfaceParams);
	
	// Multilevel constructor, levels[0] becomes the baseLevel
	btMultilevelProjectedHeightmap(btMultilevelProjectedHeightmap::sublevel_t* levels, const int nLevels, const btScalar _horzScale, const btScalar _vertScale, const btScalar _extraThickness, const bool _flippedNormals, const btMultilevelProjectedHeightmap::sphereParams_t & m_sphereSurfaceParams);
	
	inline void addSublevel(const btMultilevelProjectedHeightmap::sublevel_t & lv) {
		// You need to call init(); after adding all the sublevels!
		sublevels.push_back(lv);
		}
	
	void getSublevelsIntersecting(int x, int z, int xs, int zs, const btMultilevelProjectedHeightmap::sublevel_t** arrOut, int* idxArrOut, int & countOut) const;
			
	void init();
	
	void calculateLocalInertia(btScalar unused, btVector3& inertia) const;

	// scaling not supported ( because I am lazy )
	void setLocalScaling(const btVector3& scaling);
		
	const btVector3& getLocalScaling() const;
	
	const char* getName() const;
	
	void getAabb(const btTransform& t, btVector3& aabbMin, btVector3& aabbMax) const;
	
	void processAllTriangles(btTriangleCallback* callback, const btVector3& aabbMin, const btVector3& aabbMax) const;
	
	// worker functions for processAllTriangles
	void processAllTriangles_worker(btTriangleCallback* callback, const btVector3& aabbMin, const btVector3& aabbMax, bool firstCall) const;
	bool processAllTriangles_layer(const btMultilevelProjectedHeightmap::sublevel_t & sl, const int layerId, btTriangleCallback* callback, const btVector3& aabbMin, const btVector3& aabbMax, const btVector3& localAabbMin, const btVector3& localAabbMax, const btVector3& localUp, const bool firstCall, const bool isDoubleSideded, int xi, int zi, int xm, int zm) const;
	
	// projection functions (projects cube face <-> sphere if sphereParams.r > 0)
	btVector3 project(const btVector3 & pos) const;
	btVector3 inverseProject(const btVector3 & pos) const;
	};

#endif
