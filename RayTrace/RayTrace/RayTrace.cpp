#include <iostream>
#include<fstream>
#include<algorithm>
#include<Eigen/Dense>
#include"ray.h"
#include"hittable_list.h"
#include"sphere.h"
#include"predefine.h"
#include"camera.h"
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"
using namespace Eigen;
using namespace std;




const auto aspect_ratio = 16.0 / 9.0;
//Image
const int image_width = 600;
const int image_height = static_cast<int>(image_width / aspect_ratio);
const int channel = 3;	 
auto *img_data = (unsigned char *)malloc(image_width * image_height * channel);
const string outputPath = "output3.png";
void write2png() {
	stbi_write_png(outputPath.c_str(), image_width, image_height, channel, img_data, 0);
}
// World
hittable_list world;

//Camera
camera cam;
auto viewport_height = 2.0;
auto viewport_width = aspect_ratio * viewport_height;
auto origin = Vector4d(0, 0, 0, 1);
auto horizontal = Vector4d(viewport_width, 0, 0, 0);
auto vertical = Vector4d(0, viewport_height, 0, 0);
auto focal_length = Vector4d(0, 0, 1, 0);
auto lower_left_corner = origin - horizontal / 2 - vertical / 2-focal_length;

//Render
void shading();
void pixel_shading(int w,int h);


Vector3d trans_color(Vector3d pixel_color) {
	for(int i=0;i<3;i++)
		pixel_color(i) = static_cast<int>(255.999*pixel_color(i));
	return pixel_color;
}
Vector3d trans_color(Vector3d pixel_color,int samples_per_pixel) {
	for (int i = 0; i < 3; i++)
		pixel_color(i) = static_cast<int>(255.999*clamp(sqrt(pixel_color(i)/samples_per_pixel),0,0.999));
	return pixel_color;
}
double hit_sphere(const Vector4d& center, double radius, const ray& r) {
	Vector3d oc = (r.get_orig() - center).head(3);
	auto dir = r.get_dir().head(3).normalized();
	auto a = r.get_dir().dot(dir);
	auto c = oc.dot(oc) - radius * radius;
	auto b_prime = oc.dot(dir);
	auto discriminant = b_prime*b_prime - a*c;
	//auto b = 2.0 * oc.dot(dir);
	//auto discriminant = b * b - 4 * a*c;
	if (discriminant > 0) {
		return (-b_prime - sqrt(discriminant)) / a;
		//return (-b - sqrt(discriminant)) / (2.0*a);
	}
	return -1;
}
Vector3d ray_color(const ray& r) {
	double t0 = hit_sphere(Vector4d(0, 0, -1, 1), 0.5, r);//球心（0，0，-1）半径0.5
	if (t0 != -1) {
		Vector3d norm = (r.get_pos(t0) - Vector4d(0, 0, -1, 1)).normalized().head(3);
		Vector3d bias(1, 1, 1);
		return 0.5*(norm + bias);
		//return Vector3d(1, 0, 0);
	}
	Vector3d d = r.dir.head(3);
	d.normalize();
	auto t = 0.5*(d(1) + 1.0);
	return (1 - t)*Vector3d(1.0, 1.0, 1.0) + t * Vector3d(0.5, 0.7, 1.0);
}

Vector3d ray_color(const ray& r,const hittable& world) {
	hit_record rec;
	if (world.hit(r, 0, infinity, rec)) {
		Vector3d bias(1, 1, 1);
		return 0.5*(rec.norm.head(3) + bias);
	}
	Vector3d d = r.dir.head(3);
	d.normalize();
	auto t = 0.5*(d(1) + 1.0);
	return (1 - t)*Vector3d(1.0, 1.0, 1.0) + t * Vector3d(0.5, 0.7, 1.0);
}

Vector3d ray_color(const ray& r, const hittable& world,int depth) {
	hit_record rec;
	if (depth <= 0)
		return Vector3d(0.0, 0.0, 0.0);
	if (world.hit(r, 0, infinity, rec)) {
		Vector4d target = rec.pos + rec.norm+ random_unit_vector();
		return 0.5*ray_color(ray(rec.pos, target - rec.pos), world,depth-1);
	}
	Vector3d d = r.dir.head(3);
	d.normalize();
	auto t = 0.5*(d(1) + 1.0);
	return (1 - t)*Vector3d(1.0, 1.0, 1.0) + t * Vector3d(0.5, 0.7, 1.0);
}

void pixel_shading(int w,int h) {
	auto u = double(w) / (image_width - 1);
	auto v = double(h) / (image_height - 1);
	//ray r(origin, lower_left_corner - origin + u * horizontal + v * vertical);
	ray r = cam.get_ray(u, v);
	Vector3d p_color = trans_color(ray_color(r,world));
	//cout << p_color << endl;
	int start = (image_height - h - 1)*channel*image_width+ channel * w;
	img_data[start] = p_color(0);
	img_data[start+1] = p_color(1);
	img_data[start+2] = p_color(2);
}
void pixel_shading(int w, int h,int samples_per_pixel) {
	int start = (image_height - h - 1)*channel*image_width + channel * w;
	Vector3d p_color;
	for (int i = 0; i < samples_per_pixel; i++) {
		auto u = double(w+random_double()) / (image_width - 1);
		auto v = double(h + random_double()) / (image_height - 1);
		ray r = cam.get_ray(u, v);
		p_color += ray_color(r, world,max_depth);
	}
	p_color = trans_color(p_color, samples_per_pixel);
	img_data[start] += p_color(0);
	img_data[start + 1] += p_color(1);
	img_data[start + 2] += p_color(2);
}
void shading() {
	int range = image_width * image_height;
	for (int h = image_height - 1; h >= 0; h--) {
		for (int w = 0; w < image_width; w++) {
			pixel_shading(w, h, SAMPLES);
			//pixel_shading(w, h);
		}
	}
}


int main() {
	world.add(make_shared<sphere>(Vector4d(0, 0, -1,1), 0.5));
	world.add(make_shared<sphere>(Vector4d(0, -100.5, -1,1), 100));
	
	shading();
	write2png();
	cerr << "\ndone\n";
}