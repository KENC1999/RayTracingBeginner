#ifndef __hittable_H__
#define __hittable_H__
#include<Eigen/Dense>
#include"ray.h"
#include"predefine.h"
using namespace Eigen;

class material;

struct hit_record{
	Vector4d pos;
	Vector4d norm;
	shared_ptr <material> mat_ptr;
	double t;
	bool front_face;
	inline void set_face_normal(const ray& r, const Vector4d& outward_normal) {
		front_face = r.get_dir().dot(outward_normal)<0;
		Vector4d anti = -outward_normal;
		norm = front_face ? outward_normal : anti;
	}
};
class hittable {
public:
	virtual bool hit(const ray& r, double tmin, double tmax, hit_record& rec) const = 0;
};



#endif // !__hittable_H__

