#ifndef __hittable_list_H__
#define __hittable_list_H__
#include<vector>
#include<memory>
#include<Eigen/Dense>
#include <Eigen/StdVector>
#include"hittable.h"
using namespace Eigen;
using std::vector;
using std::shared_ptr;
using std::make_shared;

class hittable_list :public hittable {
public:
	std::vector<shared_ptr<hittable>, aligned_allocator<shared_ptr<hittable>>> objects;
public:
	hittable_list(){}
	hittable_list(shared_ptr<hittable> obj) { add(obj); }
	void clear() { objects.clear(); }
	void add(shared_ptr<hittable> obj) { objects.push_back(obj); }
	virtual bool hit(const ray& r, double tmin, double tmax, hit_record& rec) const override;
};

bool hittable_list::hit(const ray& r, double tmin, double tmax, hit_record& rec) const {
	hit_record temp_rec;
	bool hit_anything = false;
	auto closest_so_far = tmax;
	for (const auto& obj: objects) {
		if (obj->hit(r, tmin, closest_so_far, temp_rec)) {
			//printf("1111\n");
			hit_anything = true;
			closest_so_far = temp_rec.t;
			rec = temp_rec;
		}
	}
	return hit_anything;
}



#endif // !__hittable_list_H__

