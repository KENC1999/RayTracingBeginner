#ifndef __camera_H__
#define __camera_H__
#include"predefine.h"
#include<Eigen/Dense>
#include <Eigen/StdVector>
#include"ray.h"
using namespace Eigen;

class camera {
public:
	camera() {
		auto aspect_ratio = 16.0 / 9.0;
		auto viewport_height = 2.0;
		auto viewport_width = aspect_ratio * viewport_height;
		auto focal_length = Vector4d(0, 0, 1, 0);
		origin = Vector4d(0, 0, 0, 1);
		horizontal = Vector4d(viewport_width, 0, 0, 0);
		vertical = Vector4d(0, viewport_height, 0, 0);
		lower_left_corner = origin - horizontal / 2 - vertical / 2 - focal_length;
	}
	camera(double vfov,double aspect_ratio,Vector4d lookfrom, Vector4d lookat, Vector4d up, double aperture,
		double focus_dist) {
		auto theta = degrees_to_radians(vfov);
		auto h = tan(theta / 2);
		auto viewport_height = 2.0*h;
		auto viewport_width = aspect_ratio * viewport_height;
		horizontal = Vector4d::Zero();
		vertical= Vector4d::Zero();

		Vector4d focal_length = (lookfrom-lookat).normalized();
		Vector3d wup;
		wup = up.normalized().head(3);
		w = focal_length.head(3);
		u = wup.cross(w);
		v = w.cross(u);	
		horizontal.head(3) = u;
		vertical.head(3) = v;
		horizontal = focus_dist*viewport_width * horizontal;
		vertical = focus_dist*viewport_height * vertical;
		origin = lookfrom;
		lower_left_corner = origin - horizontal / 2 - vertical / 2 - focus_dist*focal_length;
		lens_radius = aperture / 2;
	}
	ray get_ray(double s,double t)const{
		auto rd= lens_radius * random_in_unit_disk();
		Vector4d offset = Vector4d::Zero();
		offset.head(3) = u * rd(0) + v * rd(1);
		return ray(origin+offset, lower_left_corner - origin-offset + s * horizontal + t * vertical);
	}
	
private:
	Vector4d origin;
	Vector4d lower_left_corner;
	Vector4d horizontal;
	Vector4d vertical;
	Vector3d u, v, w;
	double lens_radius=0;
};

#endif // !__camera_H__

