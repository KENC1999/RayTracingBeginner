#ifndef __sphere_H__
#define __sphere_H__
#include"hittable.h"
#include<Eigen/Dense>
using namespace Eigen;
class sphere:public hittable {
public:
	Vector4d center;
	double radius;
public:
	sphere();
	sphere(Vector4d pos, double r) :center(pos), radius(r) {};
	virtual bool hit(const ray& r, double tmin, double tmax, hit_record& rec) const override;
};

bool sphere::hit(const ray& r, double tmin, double tmax, hit_record& rec) const {
	Vector3d oc = (r.get_orig() - center).head(3);
	auto dir = r.get_dir().head(3).normalized();
	auto a = r.get_dir().dot(dir);
	auto c = oc.dot(oc) - radius * radius;
	auto b_prime = oc.dot(dir);
	auto discriminant = b_prime * b_prime - a * c;	
	if (discriminant > 0) {
		//printf("%f\n", discriminant);
		auto root = sqrt(discriminant);
		auto temp = (-b_prime - root) / a;
		if (temp<tmax&&temp>tmin) {
			rec.t = temp;
			rec.pos = r.get_pos(temp);
			Vector4d out_norm= (rec.pos - center) / radius;
			rec.set_face_normal(r, out_norm);
			return true;
		}
		temp= (-b_prime + root) / a;
		if (temp<tmax&&temp>tmin) {
			rec.t = temp;
			rec.pos = r.get_pos(temp);
			Vector4d out_norm = (rec.pos - center) / radius;
			rec.set_face_normal(r, out_norm);
			return true;
		}
	}
	return false;
}

#endif // ! __sphere_H__

