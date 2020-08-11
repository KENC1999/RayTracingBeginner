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
	ray get_ray(double u,double v)const{
		//std::cout << horizontal << std::endl << std::endl;
		return ray(origin, lower_left_corner - origin + u * horizontal + v * vertical);
	}
	
private:
	Vector4d origin;
	Vector4d lower_left_corner;
	Vector4d horizontal;
	Vector4d vertical;
};

#endif // !__camera_H__

