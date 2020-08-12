#ifndef __material_H__
#define __material_H__

#include"ray.h"
#include"predefine.h"
#include"hittable.h"
#include<iostream>
using namespace std;
struct hit_record;

class material {
public:
	virtual bool scatter(const ray& r_in, const hit_record& rec, 
		Vector3d& attenuation_color, ray& scattered) const = 0;
};

class lambertian : public material {
public:
	Vector3d albedo;
public:
	lambertian(const Vector3d& a) : albedo(a) {}
	virtual bool scatter(
		const ray& r_in, const hit_record& rec, Vector3d& attenuation_color, ray& scattered
	) const override {
		Vector4d scatter_direction = rec.norm + random_unit_vector();
		scattered = ray(rec.pos, scatter_direction);//���������
		attenuation_color = albedo;//������ɫ
		return true;
	}
};

class metal :public material {
public:
	Vector3d albedo;
	double fuzz;
public:
	metal(const Vector3d& a,double f) : albedo(a),fuzz(f<1?f:1){}
	virtual bool scatter(
		const ray& r_in, const hit_record& rec, Vector3d& attenuation_color, ray& scattered
	) const override {
		Vector4d r = reflect(r_in.get_dir(), rec.norm);
		scattered = ray(rec.pos, r+fuzz*random_unit_vector());//fuzz�Ŷ�ϵ��0~1
		attenuation_color = albedo;
		return (r.dot(rec.norm)>0);
	}
};

class dielectric :public material {
public:
	Vector3d albedo;
	double ref_idx;
public:
	dielectric(const Vector3d& a,double ri) : albedo(a),ref_idx(ri) {}
	virtual bool scatter(
		const ray& r_in, const hit_record& rec, Vector3d& attenuation_color, ray& scattered
	) const override {
		attenuation_color = albedo;
		double etai = rec.front_face ? (1.f / ref_idx) : ref_idx;
		Vector4d uv = r_in.get_dir();
		Vector4d n = rec.norm;
		double cos_theta = fmin(1, (-uv).dot(n));
		double sin_theta = sqrt(1 - cos_theta * cos_theta);
		if (sin_theta*etai > 1) {//ȫ����
			Vector4d reflected = reflect(uv, n);
			scattered = ray(rec.pos, reflected);
			return true;
		}
		double reflect_prob = schlick(cos_theta, etai);
		if (random_double() < reflect_prob)//��������ͷ���ͬʱ���������
		{
			Vector4d reflected = reflect(uv, n);
			scattered = ray(rec.pos, reflected);
			return true;
		}
		Vector4d refracted = refract(uv,n, etai);//����
		scattered = ray(rec.pos, refracted);
		return true;
	}
};

#endif // !__material_H__

