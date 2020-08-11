#ifndef __sphere_H__
#define __sphere_H__
#include"hittable.h"
#include<Eigen/Dense>
#include<iostream>
using namespace std;
using namespace Eigen;
class sphere:public hittable {
public:
	Vector4d center;
	double radius;
	shared_ptr<material> mat_ptr;
public:
	sphere();
	sphere(Vector4d pos, double r) :center(pos), radius(r) {};
	sphere(Vector4d pos, double r, shared_ptr<material> ptr) :center(pos), radius(r),mat_ptr(ptr){};
	virtual bool hit(const ray& r, double tmin, double tmax, hit_record& rec) const override;
};

bool sphere::hit(const ray& r, double tmin, double tmax, hit_record& rec) const {
	Vector3d oc = (r.get_orig() - center).head(3);
	Vector3d dir = r.get_dir().head(3);
	//cout << dir << endl << endl;
	auto a = dir.dot(dir);
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
			Vector4d out_norm= (rec.pos - center).normalized();
			rec.set_face_normal(r, out_norm);
			return true;
		}
		temp= (-b_prime + root) / a;
		if (temp<tmax&&temp>tmin) {
			rec.t = temp;
			rec.pos = r.get_pos(temp);
			Vector4d out_norm = (rec.pos - center).normalized();
			rec.set_face_normal(r, out_norm);
			return true;
		}
	}
	return false;
}

#endif // ! __sphere_H__

