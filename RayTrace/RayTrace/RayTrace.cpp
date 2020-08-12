#include <iostream>
#include<fstream>
#include<algorithm>
#include<Eigen/Dense>
#include"ray.h"
#include"hittable_list.h"
#include"sphere.h"
#include"predefine.h"
#include"camera.h"
#include"material.h"
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
const string outputPath = "output7.png";
void write2png() {
	stbi_write_png(outputPath.c_str(), image_width, image_height, channel, img_data, 0);
}
// World
hittable_list world;

//Camera
//Vector4d lookfrom(0,0,1,1);
//Vector4d lookat(0,0,-1,1);
//Vector4d up(0,1,0,0);
//auto dist_to_focus = 1;
//auto aperture = 0;
//camera cam(90,aspect_ratio,lookfrom,lookat,up,aperture,dist_to_focus);
Vector4d lookfrom(13, 2, 3, 1);
Vector4d lookat(0, 0, 0, 1);
Vector4d up(0, 1, 0, 0);
auto dist_to_focus = 10;
auto aperture = 0.1;
camera cam(20, aspect_ratio, lookfrom, lookat, up, aperture, dist_to_focus);

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
		return Vector3d(0,0,0);
	if (world.hit(r, 0.001, infinity, rec)) {
		ray scattered;
		Vector3d att_color;
		if (rec.mat_ptr->scatter(r, rec, att_color, scattered)) {
			return att_color.cwiseProduct(ray_color(scattered, world, depth - 1));	
		}
		return Vector3d(0.0, 0.0, 0.0);
		//Vector4d target = rec.pos + rec.norm+ random_unit_vector();
		//return 0.5*ray_color(ray(rec.pos, target - rec.pos), world,depth-1);
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
	int start = (image_height - h - 1)*channel*image_width+ channel * w;//像素着色
	img_data[start] = p_color(0);
	img_data[start+1] = p_color(1);
	img_data[start+2] = p_color(2);
}
void pixel_shading(int w, int h,int samples_per_pixel) {//samples为采样光线数
	int start = (image_height - h - 1)*channel*image_width + channel * w;
	Vector3d p_color;
	for (int i = 0; i < samples_per_pixel; i++) {
		auto u = double(w+random_double()) / (image_width - 1);
		auto v = double(h + random_double()) / (image_height - 1);
		ray r = cam.get_ray(u, v);//计算光线
		p_color += ray_color(r, world,max_depth);//计算着色，多光线着色累加
	}
	p_color = trans_color(p_color, samples_per_pixel);//像素着色，求平均
	img_data[start] += p_color(0);
	img_data[start + 1] += p_color(1);
	img_data[start + 2] += p_color(2);
}
void shading() {
	int range = image_width * image_height;
	for (int h = image_height - 1; h >= 0; h--) {
		cout << h << endl;
		for (int w = 0; w < image_width; w++) {
			pixel_shading(w, h, SAMPLES);
			//pixel_shading(w, h);
		}
	}
}

hittable_list random_scene() {
	hittable_list world;

	auto ground_material = make_shared<lambertian>(Vector3d(0.5, 0.5, 0.5));
	world.add(make_shared<sphere>(Vector4d(0, -1000, 0,1), 1000, ground_material));

	for (int a = -11; a < 11; a++) {
		for (int b = -11; b < 11; b++) {
			auto choose_mat = random_double();
			Vector4d center(a + 0.9*random_double(), 0.2, b + 0.9*random_double(),1);

			Vector4d d1 = center - Vector4d(4, 0.2, 0, 1);//当前球心到第一个大球的连线
			Vector4d d2 = center - Vector4d(0, 0.2, 0, 1);//当前球心到第二个大球的连线
			Vector4d d3 = center - Vector4d(-4, 0.2, 0, 1);//当前球心到第三个大球的连线
			if (d1.norm() > 0.9&&d2.norm()>0.9&&d3.norm()>0.9) {
				shared_ptr<material> sphere_material;

				if (choose_mat < 0.4) {
					// diffuse
					Vector3d albedo = random_vec(0, 1).head(3);
					sphere_material = make_shared<lambertian>(albedo);
					world.add(make_shared<sphere>(center, 0.2, sphere_material));
				}
				else if (choose_mat < 0.75) {
					// metal
					Vector3d albedo = random_vec(0.2, 1).head(3);
					auto fuzz = random_double(0, 0.5);
					sphere_material = make_shared<metal>(albedo, fuzz);
					world.add(make_shared<sphere>(center, 0.2, sphere_material));
				}
				else {
					// glass
					sphere_material = make_shared<dielectric>(random_vec(0.4, 1).head(3),random_double(1.3,2.3));
					world.add(make_shared<sphere>(center, 0.2, sphere_material));
				}
			}
		}
	}

	auto material1 = make_shared<dielectric>(Vector3d(1, 1, 1),1.5);
	world.add(make_shared<sphere>(Vector4d(0, 1, 0,1), 1.0, material1));

	auto material2 = make_shared<lambertian>(Vector3d(0.4, 0.8, 0.2));
	world.add(make_shared<sphere>(Vector4d(-4, 1, 0,1), 1.0, material2));

	auto material3 = make_shared<metal>(Vector3d(0.9, 0.6, 0.4), 0.0);
	world.add(make_shared<sphere>(Vector4d(4, 1, 0,1), 1.0, material3));

	return world;
}

int main() {
	//auto material_ground = make_shared<lambertian>(Vector3d(0.8, 0.8, 0.0));
	//auto material_center = make_shared<lambertian>(Vector3d(0.1, 0.2, 0.5));
	//auto material_left = make_shared<dielectric>(Vector3d(1, 1, 1),1.5);
	//auto material_right = make_shared<metal>(Vector3d(0.8, 0.6, 0.2),0.8);
	//auto material_right_1 = make_shared<metal>(Vector3d(0.4, 0.1, 0.2), 0.5);
	//world.add(make_shared<sphere>(Vector4d(0.0, -100.5, -1.0,1), 100.0, material_ground));
	//world.add(make_shared<sphere>(Vector4d(0.0, 0.0, -1.0, 1), 0.5, material_center));
	//world.add(make_shared<sphere>(Vector4d(-1.0, 0.0, -1.0, 1), 0.5, material_left));
	//world.add(make_shared<sphere>(Vector4d(-1.0, 0.0, -1.0,1), -0.45, material_left));
	//world.add(make_shared<sphere>(Vector4d(1.0, 0.0, -1.0, 1), 0.5, material_right));
	world = random_scene();

	shading();
	write2png();
	cerr << "\ndone\n";
}