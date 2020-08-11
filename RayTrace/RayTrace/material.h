#ifndef __material_H__
#define __material_H__

#include"ray.h"
#include"predefine.h"

struct hit_record;

class material {
public:
	virtual bool scatter(const ray& r_in, const hit_record& rec, 
		Vector3d& attenuation_color, ray& scattered) const = 0;
};
#endif // !__material_H__

