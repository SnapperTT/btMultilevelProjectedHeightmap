// cubeSphereProjectionUtils.cpp
#include "cubeSphereProjectionUtils.h"

void CubeSphereProjectionUtils::sphereToCube_cubify (double const sx, double const sy, double const sz, double & x, double & y, double & z) {
	// sy is the "up" value
	constexpr double isqrt2 = 0.70710676908493042;
	double a2 = sx * sx * 2.0;
	double b2 = sz * sz * 2.0;
	double inner = -a2 + b2 -3;
	double innersqrt = -sqrt((inner * inner) - 12.0 * a2);
	
	// Fix rounding errors
	double xterm = innersqrt + a2 - b2 + 3.0;
	if (xterm < 0.0) xterm = 0.0;
	double zterm = innersqrt - a2 + b2 + 3.0;
	if (zterm < 0.0) zterm = 0.0;
	
	x = sgnd(sx)*sqrt(xterm) * isqrt2;
	y = sgnd(sy);
	z = sgnd(sz)*sqrt(zterm) * isqrt2;
	}

void CubeSphereProjectionUtils::sphereToCube_bmph (double const sx, double const sy, double const sz, double & x, double & y, double & z) {
	/// Converts normalized spherical coordinates to normalized cube coordinates
	/// https://stackoverflow.com/questions/2656899/mapping-a-sphere-to-a-cube/
	/// https://petrocket.blogspot.com/2010/04/sphere-to-cube-mapping.html
	/// cubeToSphere -> sphereToCube will not create the correct mapping, but this one works for this library
	double fx = abs(sx);
	double fy = abs(sy);
	double fz = abs(sz);

	bool a = (fy >= fx) && (fy >= fz);
	bool b = (fx >= fz);

	//btVector3 spherev(sx, sy, sz);
	//return a ? sphereToCube_cubify(swizzleXZY(spherev)).xzy : b ? sphereToCube_cubify(swizzleYZX(spherev)).zxy : sphereToCube_cubify(spherev);
	a ? sphereToCube_cubify(sx, sy, sz, x, z, y) : ( b ? sphereToCube_cubify(sy, sx, sz, y, z, x) : sphereToCube_cubify(sx, sz, sy, x, y, z) );
	}

void CubeSphereProjectionUtils::sphereToCubeSimple (double const sx, double const sy, double const sz, double & x, double & y, double & z)  {
	// handles the simplified special case where (x,z) are the plane
	sphereToCube_cubify(sx, sy, sz, x, y, z);	
	}
	
void CubeSphereProjectionUtils::sphereToCube (double const sx, double const sy, double const sz, double &x, double &y, double &z) {
    double fx = std::abs(sx);
    double fy = std::abs(sy);
    double fz = std::abs(sz);

    double tx, ty, tz;
    if (fy >= fx && fy >= fz) {
        // Y is dominant
        sphereToCube_cubify(sx, sy, sz, tx, ty, tz);
        x = tx;
        y = ty;
        z = tz;
		}
    else if (fx >= fz) {
        // X is dominant → swap X↔Y
        sphereToCube_cubify(sy, sx, sz, tx, ty, tz);
        // UNSWIZZLE
        x = ty;
        y = tx;
        z = tz;
		}
    else {
        // Z is dominant → swap Y↔Z
        sphereToCube_cubify(sx, sz, sy, tx, ty, tz);
        // UNSWIZZLE
        x = tx;
        y = tz;
        z = ty;
		}
	}

void CubeSphereProjectionUtils::computeCentroid (double const u, double const v, double const usz, double const vsz, int const iXSz, int const iZSz, double & cx, double & cy, double & cz) {
	// Returns the position centre of the arc, assuming the world is at the origin		
	double umin = 0;
	double vmin = 0;
	umin += u*2.0 - 1.0; // u/v now has range [-1, 1]
	vmin += v*2.0 - 1.0;
	
	double umax = 2.0*iXSz*usz/(iXSz-1);
	double vmax = 2.0*iZSz*vsz/(iZSz-1);
	umax += u*2.0 - 1.0; // u/v now has range [-1, 1]
	vmax += v*2.0 - 1.0;
	
	double umid = (umin + umax)/2.0;
	double vmid = (vmin + vmax)/2.0;
	
	double sx, sy, sz;
	CubeSphereProjectionUtils::cubeToSphere (umid, 1.0, vmid, sx, sy, sz);
	
	const double segSz = usz;
	const double rr = (segSz*0.5*(1.0 + 1.0/sqrt(2.0)) + (1.0 - segSz));
	
	cx = rr*sx;
	cy = rr*sy;
	cz = rr*sz;
	}

