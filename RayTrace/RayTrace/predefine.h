#ifndef __predefine_H__
#define __predefine_H__
#include <cmath>
#include <cstdlib>
#include <limits>
#include <memory>
#include<random>
#include<Eigen/Dense>
using namespace Eigen;
// Usings

using std::shared_ptr;
using std::make_shared;
using std::sqrt;

// Constants

const double infinity = std::numeric_limits<double>::infinity();
const double pi = 3.1415926535897932385;
const int SAMPLES = 20;
const int max_depth = 20;

// Utility Functions

inline double degrees_to_radians(double degrees) {
	return degrees * pi / 180.0;
}

inline double random_double() {
	static std::uniform_real_distribution<double> distribution(0.0, 1.0);
	static std::mt19937 generator;
	return distribution(generator);
}

inline double random_double(double min,double max) {
	return min+(max-min)*random_double();
}

inline double clamp(double x, double min, double max) {
	if (x < min) return min;
	if (x > max) return max;
	return x;
}

inline Vector4d random_vec() {
	return Vector4d(random_double(), random_double(), random_double(), 0);
}
inline Vector4d random_vec(double min,double max) {
	return Vector4d(random_double(min,max), random_double(min, max), random_double(min, max), 0);
}

inline Vector4d random_in_unit_sphere() {
	while (true){
		auto p = random_vec(-1, 1);
		if (p.norm() >= 1)
			continue;
		return p;
	}
}

inline Vector4d random_unit_vector() {
	auto a = random_double(0, 2 * pi);
	auto z = random_double(-1, 1);
	auto r = sqrt(1 - z * z);
	return Vector4d(r*cos(a), r*sin(a), z,0);
}

inline Vector4d reflect(const Vector4d& v, const Vector4d& n) {
	return v - 2 * v.dot(n)*n;
}

inline Vector4d random_in_unit_disk() {
	while (true) {
		auto p = Vector4d(random_double(-1, 1), random_double(-1, 1), 0,0);
		if (p.norm()>= 1) continue;
		return p;
	}
}

Vector4d refract(const Vector4d& uv, const Vector4d& n, double eta) {
	Vector3d N = n.head(3);
	Vector3d L = uv.head(3);
	double cosi = (-N).dot(L);
	double cost2 = 1.0 - eta * eta*(1.0 - cosi * cosi);
	Vector4d t = Vector4d::Zero();
	t.head(3)= eta * L + (eta*cosi-sqrt(abs(cost2)))*N;
	return t;
}


inline double schlick(double cosine, double ref_idx) {
	auto r0 = (1 - ref_idx) / (1 + ref_idx);
	r0 = r0 * r0;
	return r0 + (1 - r0)*pow((1 - cosine), 5);
}

#endif // !__predefine_H__

