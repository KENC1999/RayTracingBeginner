#ifndef __ray_H__
#define __ray_H__
#include<Eigen/Dense>
using namespace Eigen;
class ray {
public:
	Vector4d orig;
	Vector4d dir;
	ray(){}
	ray(const Vector4d& origin, const Vector4d& direction) :orig(origin), dir(direction.normalized()){}
	Vector4d get_orig()const { return orig; }
	Vector4d get_dir()const { return dir; }
	Vector4d get_pos(double t)const { return orig + t * dir; }
};

#endif // !__RAY_H__

