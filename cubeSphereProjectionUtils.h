// cubeSphereProjectionUtils.h
#ifndef HEADER_cubeSphereProjectionUtils_hh
#define HEADER_cubeSphereProjectionUtils_hh
class CubeSphereProjectionUtils {
public:
	static void sphereToCube_cubify (double const sx, double const sy, double const sz, double & x, double & y, double & z);
	static void sphereToCube (double const sx, double const sy, double const sz, double & x, double & y, double & z);
	static void sphereToCubeSimple (double const sx, double const sy, double const sz, double & x, double & y, double & z);
	static void computeCentroid (double const u, double const v, double const usz, double const vsz, int const iXSz, int const iZSz, double & cx, double & cy, double & cz);
	
	inline static void cubeToSphere (double const x, double const y, double const z, double & sx, double & sy, double & sz) {
		/// Converts normalized cube coordinates to normalized spherical coordinates
		/// https://mathproofs.blogspot.com/2005/07/mapping-cube-to-sphere.html
		sx = x * sqrt(1.0 - y * y * 0.5 - z * z * 0.5 + y * y * z * z / 3.0);
		sy = y * sqrt(1.0 - z * z * 0.5 - x * x * 0.5 + z * z * x * x / 3.0);
		sz = z * sqrt(1.0 - x * x * 0.5 - y * y * 0.5 + x * x * y * y / 3.0);
		}
	};
#endif
