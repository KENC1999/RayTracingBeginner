# Ray Tracing in One Weekend 实践

根据 [Ray Tracing in One Weekend ](https://raytracing.github.io/books/RayTracingInOneWeekend.html#dielectrics/schlickapproximation)教程实现了一个入门的光线追踪渲染器，使用的矩阵库为Eigen3，最后完成的效果如下：
![img](https://github.com/KENC1999/RayTracingBeginner/blob/master/RayTrace/RayTrace/output6.png?raw=true)
![img](https://github.com/KENC1999/RayTracingBeginner/blob/master/RayTrace/RayTrace/output7.png)

由于原书步骤比较详细，而且很多大神已经写了这本书的笔记了，我就只挑重点内容和coding中的一些问题写一个简要的参考。

##  1 图片保存成PNG

​	书中把渲染的图片保存成ppm格式，但这种格式不方便查看而且文件较大，所以我这里将图片存储为png格式，用到了开源库stb_image来操作图片（仅使用了"stb_image_write.h"头文件）。用char数组来存储像素值，存储时用接口函数将数组转换成png保存即可。注意数组中的相邻的每三个元素对应一个像素上的RGB值。主要代码如下。

```c++
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
		}
	}
}
void write2png() {
	stbi_write_png(outputPath.c_str(), image_width, image_height, channel, img_data, 0);
}
```



## 2 光线与球体相交

光线追踪器从每个像素上发送光线，并计算在这些光线方向上看到的颜色，步骤为：

(1)计算从相机到像素的光线；

(2)确定光线与哪些物体相交；

(3)计算交点的颜色。

计算与球体交点时，转化为以下的一元二次方程求根问题：

![img](https://raytracing.github.io/images/fig-1.04-ray-sphere.jpg)

$(\mathbf{A}+t \mathbf{b}-\mathbf{C}) \cdot(\mathbf{A}+t \mathbf{b}-\mathbf{C})=r^{2}$

$t^{2} \mathbf{b} \cdot \mathbf{b}+2 t \mathbf{b} \cdot(\mathbf{A}-\mathbf{C})+(\mathbf{A}-\mathbf{C}) \cdot(\mathbf{A}-\mathbf{C})-r^{2}=0 $

A是光线发出点，b是光线方向，C是球心，r是球的半径，这几个量都是已知的，先求$Δ=b^{2}-4ac $，判断光线和球有几个交点，为简单起见只考虑光线与球相交即Δ>0的情况。由于遮挡的存在，所以返回离光源近的交点。

当场景中有多个球体时，将所有球体保存在一个列表中，依次计算光线和每个球体的交点，并增加tmin和tmax两个阈值。tmin为离光源的最小距离限制（球体内部计算折射时，光源在球体表面，tmin将剔除掉t=0的交点），tmax用于保存与球体的最近的交点。仅当tmin<t<tmax时，更新交点位置信息和tmax的值。主要代码如下。

```c++
bool sphere::hit(const ray& r, double tmin, double tmax, hit_record& rec) const {
	Vector3d oc = (r.get_orig() - center).head(3);
	Vector3d dir = r.get_dir().head(3);
	//cout << dir << endl << endl;
	auto a = dir.dot(dir);
	auto c = oc.dot(oc) - radius * radius;
	auto b_prime = oc.dot(dir);
	auto discriminant = b_prime * b_prime - a * c;	
	if (discriminant > 0) {//有两个交点
		//printf("%f\n", discriminant);
		auto root = sqrt(discriminant);
		auto temp = (-b_prime - root) / a;//先取近的交点
		if (temp<tmax&&temp>tmin) {//比较阈值
			rec.t = temp;//交点方向位置
			rec.pos = r.get_pos(temp);//交点空间位置
			Vector4d out_norm= (rec.pos - center)/radius;//交点处的球面法线
			rec.set_face_normal(r, out_norm);
			rec.mat_ptr = mat_ptr;//交点处的球面材质信息
			return true;
		}
		temp= (-b_prime + root) / a;
		if (temp<tmax&&temp>tmin) {
			rec.t = temp;
			rec.pos = r.get_pos(temp);
			Vector4d out_norm = (rec.pos - center) / radius;
			rec.set_face_normal(r, out_norm);
			rec.mat_ptr = mat_ptr;
			return true;
		}
	}
	return false;
}
```



## 3 漫反射

漫反射原理比较简单，即随机生成反射光线，且保证反射光线和法线夹角不大于90°。可以利用以入射点为切点的单位球生成，起点为切点，另外一点单位球内随机一点。

![img](https://raytracing.github.io/images/fig-1.10-rand-unitvec.png)

```c++
	virtual bool scatter(
		const ray& r_in, const hit_record& rec, Vector3d& attenuation_color, ray& scattered
	) const override {
		Vector4d scatter_direction = rec.norm + random_unit_vector();
		scattered = ray(rec.pos, scatter_direction);//漫反射光线
		attenuation_color = albedo;//材质颜色
		return true;
	}
```

## 4 镜面反射

![img](https://raytracing.github.io/images/fig-1.11-reflection.jpg)

如上图计算反射光线$ v^{'}=v-2(v·n)*n $

另外可以对反射光线增加扰动来实现磨砂质感的镜面。

```c++
	virtual bool scatter(
		const ray& r_in, const hit_record& rec, Vector3d& attenuation_color, ray& scattered
	) const override {
		Vector4d r = reflect(r_in.get_dir(), rec.norm);
		scattered = ray(rec.pos, r+fuzz*random_unit_vector());//fuzz扰动系数0~1
		attenuation_color = albedo;
		return (r.dot(rec.norm)>0);
	}
```

## 5 折射

​		我用作者的公式计算折射光线渲染结果会出现黑边，最后参考这篇文章：[三维空间折射向量计算](https://www.cnblogs.com/night-ride-depart/p/7429618.html)修改了折射公式。

![image](https://images2017.cnblogs.com/blog/429727/201708/429727-20170825204804183-893590663.png)

![image](https://images2017.cnblogs.com/blog/429727/201708/429727-20170825224942371-1578553322.png)



```c++
Vector4d refract(const Vector4d& uv, const Vector4d& n, double eta) {
	Vector3d N = n.head(3);
	Vector3d L = uv.head(3);
	double cosi = (-N).dot(L);
	double cost2 = 1.0 - eta * eta*(1.0 - cosi * cosi);
	Vector4d t = Vector4d::Zero();
	t.head(3)= eta * L + (eta*cosi-sqrt(abs(cost2)))*N;
	return t;
}
```

此外折射时还要考虑全反射、同时发生反射和折射的情况（使用Schlick公式来近似）。

```c++
	virtual bool scatter(
		const ray& r_in, const hit_record& rec, Vector3d& attenuation_color, ray& scattered
	) const override {
		attenuation_color = albedo;
		double etai = rec.front_face ? (1.f / ref_idx) : ref_idx;
		Vector4d uv = r_in.get_dir();
		Vector4d n = rec.norm;
		double cos_theta = fmin(1, (-uv).dot(n));
		double sin_theta = sqrt(1 - cos_theta * cos_theta);
		if (sin_theta*etai > 1) {//全反射
			Vector4d reflected = reflect(uv, n);
			scattered = ray(rec.pos, reflected);
			return true;
		}
		double reflect_prob = schlick(cos_theta, etai);
		if (random_double() < reflect_prob)//近似折射和反射同时发生的情况
		{
			Vector4d reflected = reflect(uv, n);
			scattered = ray(rec.pos, reflected);
			return true;
		}
		Vector4d refracted = refract(uv,n, etai);//折射
		scattered = ray(rec.pos, refracted);
		return true;
	}
};
```



## 6 相机

相机原理参考书中的教程。



## 7 一些问题

这个入门级别的光线追踪渲染器只按光线追踪的思想进行了初步的渲染，没有涉及辐射度量学和brdf的概念，未实现加速结构和并行化(渲染分辨率1000*562以上的图片就比较耗时了），以及相机中要手动调整的参数过多，某些角度透视有问题。目前打算先读一阵PBRT和RTR，之后结合OpenGL和Qt写一个交互性好一点的光线追踪渲染器。
