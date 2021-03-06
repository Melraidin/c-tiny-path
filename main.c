#include <math.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>

/* #define DEBUG 1 */
#ifdef DEBUG
#define debug(M, ...) printf(M, ##__VA_ARGS__)
#else
#define debug(M, ...)
#endif

/* const int spp = 64; */
/* const int spp = 4; */
const int spp = 1024;
const int maxDepth = 8;
const int width = 512;
const int height = 512;

const double invertedPi = 1.0 / 3.1415926535;
const double twoPi = 2.0 * 3.1415926535;
const double epsilon = 1e-14;

typedef struct SimpleHit SimpleHit;
typedef struct Object Object;
typedef struct Material Material;
typedef struct Ray Ray;
typedef struct Triple Triple;
typedef struct Camera Camera;
typedef struct Plane Plane;
typedef struct Triangle Triangle;

struct Triple {
	double v[3];
};

Triple BLACK;
Triple MAGENTA;

struct SimpleHit {
	double t;
	Object* object;
	Triple normal;
};

struct Camera {
	Triple eye;
	Triple focal;
	double viewDist;
	Triple up;
	Triple w;
	Triple u;
	Triple v;
	Triple wMultViewDist;
};

Triple orientedHemiDir(double u1, double u2, Triple *normal, double exp);
SimpleHit intersectObjects(int numObjects, Object* objects, Ray *ray);
Camera newCamera(Triple eye, Triple focal, double viewDist, Triple up);
void cameraCalcOrthonormalBasis(Camera* camera);
Ray cameraSpawnRay(Camera* camera, double x, double y);
double clamp(double x);
unsigned char gammaTransform(double x);
void render(int numObjects, Object* objects, Camera* camera, char* buffer);
void writePPM(char* path, char* buffer);
void normalize(Triple* v);
Triple radiance(int numObjects, Object* objects, Ray* ray, int depth);
Triple diRadiance(int numObjects, Object* objects, Ray* ray, Triple* normal, Material* mat, int depth);

double doubleRand() {
	// TODO Implement our own less random but faster rand().
	return (double)rand() / (double)RAND_MAX;
}

Triple scale(Triple* inp, double x) {
	return (Triple){
		{inp->v[0] * x, inp->v[1] * x, inp->v[2] * x}
	};
}

void scalePointer(Triple* inp, double x) {
	inp->v[0] *= x;
	inp->v[1] *= x;
	inp->v[2] *= x;
}

Triple multiplyParts(Triple *x1, Triple *x2) {
	return (Triple){
		{x1->v[0] * x2->v[0], x1->v[1] * x2->v[1], x1->v[2] * x2->v[2]}
	};
}

Triple add(Triple* x1, Triple* x2) {
	return (Triple){
		{x1->v[0] + x2->v[0], x1->v[1] + x2->v[1], x1->v[2] + x2->v[2]}
	};
}

void addPointer(Triple* x1, Triple* x2) {
	x1->v[0] += x2->v[0];
	x1->v[1] += x2->v[1];
	x1->v[2] += x2->v[2];
}

Triple subtract(Triple* x1, Triple* x2) {
	return (Triple){
		{x1->v[0] - x2->v[0], x1->v[1] - x2->v[1], x1->v[2] - x2->v[2]}
	};
}

void subtractPointer(Triple* x1, Triple* x2) {
	x1->v[0] -= x2->v[0];
	x1->v[1] -= x2->v[1];
	x1->v[2] -= x2->v[2];
}

double innerProduct(Triple* x1, Triple* x2) {
	return (x1->v[0] * x2->v[0]) + (x1->v[1] * x2->v[1]) + (x1->v[2] * x2->v[2]);
}

Triple crossProduct(Triple* x1, Triple* x2) {
	return (Triple){{
		(x1->v[1] * x2->v[2]) - (x1->v[2] * x2->v[1]),
		(x1->v[2] * x2->v[0]) - (x1->v[0] * x2->v[2]),
		(x1->v[0] * x2->v[1]) - (x1->v[1] * x2->v[0])
	}};
}

typedef struct {
	Triple dir;
	double pdf;
} SampleHit;

struct Ray {
	Triple org;
	Triple dir;
};

Triple rayHit(Ray* ray, double t) {
	Triple scaled = scale(&ray->dir, t);
	return add(&ray->org, &scaled);
}

struct Material {
	void* data;
	Triple (*f)(Material* material, Triple *wi, Triple *wo, Triple *normal);
	SampleHit (*sampleF)(Material* material, Triple *normal, Triple *wo);
	Triple (*emiss)(Material *material);
	bool (*dielectric)(Material* material);
};

struct Object {
	void* data;
	Material* material;
	SimpleHit (*intersect)(Object *object, Ray *ray);
	Triple (*normal)(Object *object, Ray *ray, double t);
	void (*print)(Object *object);
};

typedef struct {
	Triple col;
	Triple emiss;
	bool dielectric;
} Emitter;

Triple emitF(Material* material, Triple *wi, Triple *wo, Triple *normal);
SampleHit emitSampleF(Material* material, Triple *normal, Triple *wo);
Triple emitEmiss(Material *materal);

Triple emitF(Material* material, Triple *wi, Triple *wo, Triple *normal) {
	return scale(&((Emitter *)material)->col, invertedPi);
}

SampleHit emitSampleF(Material* material, Triple *normal, Triple *wo) {
	Triple wi = orientedHemiDir(doubleRand(), doubleRand(), normal, 0.0);
	double inner = innerProduct(normal, &wi);
	double pdf = inner * invertedPi;
	return (SampleHit){
		.dir = wi,
		.pdf = pdf
	};
}

Triple emitEmiss(Material *material) {
	Emitter* emitter = (Emitter*)(material->data);
	return emitter->emiss;
}

bool emitDielectric(Material *material) {
	Emitter* emitter = (Emitter*)(material->data);
	return emitter->dielectric;
}

Material emitterBase;

void emitterInit() {
	emitterBase = (Material){
		.f = emitF,
		.sampleF = emitSampleF,
		.emiss = emitEmiss,
		.dielectric = emitDielectric
	};
}

Material newEmitter(Triple emiss, bool dielectric) {
	Emitter* emitterData = malloc(sizeof(Emitter));
	emitterData->col = emiss;
	emitterData->emiss = emiss;
	emitterData->dielectric = dielectric;

	Material emitter = emitterBase;
	emitter.data = emitterData;

	return emitter;
}

typedef struct {
	Triple col;
	Triple emiss;
	bool dielectric;
} Diffuse;

Triple diffuseF(Material* material, Triple *wi, Triple *wo, Triple *normal);
SampleHit diffuseSampleF(Material* material, Triple *normal, Triple *wo);
Triple diffuseEmiss(Material* materal);

Triple diffuseF(Material* material, Triple *wi, Triple *wo, Triple *normal) {
	return scale(&((Diffuse*)material->data)->col, invertedPi);
}

SampleHit diffuseSampleF(Material* material, Triple *normal, Triple *wo) {
	Triple wi = orientedHemiDir(doubleRand(), doubleRand(), normal, 0.0);
	double inner = innerProduct(normal, &wi);
	double pdf = inner * invertedPi;
	return (SampleHit){
		.dir = wi,
		.pdf = pdf
	};
}

Triple diffuseEmiss(Material *material) {
	Diffuse* diffuse = (Diffuse*)(material->data);
	return diffuse->emiss;
}

bool diffuseDielectric(Material *material) {
	Diffuse* diffuse = (Diffuse*)(material->data);
	return diffuse->dielectric;
}

Material diffuseBase;

void diffuseInit() {
	diffuseBase = (Material){
		.f = diffuseF,
		.sampleF = diffuseSampleF,
		.emiss = diffuseEmiss,
		.dielectric = diffuseDielectric
	};
}

Material newDiffuse(Triple color, Triple emiss, bool dielectric) {
	Diffuse* diffuseData = malloc(sizeof(Diffuse));
	diffuseData->col = color;
	diffuseData->emiss = emiss;
	diffuseData->dielectric = dielectric;

	Material diffuse = diffuseBase;
	diffuse.data = diffuseData;

	return diffuse;
}

typedef struct {
	Triple col;
	Triple emiss;
	bool dielectric;
} Specular;

Triple specularF(Material* material, Triple *wi, Triple *wo, Triple *normal);
SampleHit specularSampleF(Material* material, Triple *normal, Triple *wo);
Triple specularEmiss(Material* materal);

Triple specularF(Material* material, Triple *wi, Triple *wo, Triple *normal) {
	return ((Specular*)material->data)->col;
}

SampleHit specularSampleF(Material* material, Triple *normal, Triple *wo) {
	Triple inverse = scale(wo, -1);
	Triple normalDoubled = scale(normal, 2);
	scalePointer(&normalDoubled, innerProduct(normal, wo));
	addPointer(&inverse, &normalDoubled);
	normalize(&inverse);
	double pdf = innerProduct(normal, &inverse);

	return (SampleHit){
		.dir = inverse,
		.pdf = pdf
	};
}

Triple specularEmiss(Material *material) {
	return ((Specular*)(material->data))->emiss;
}

bool specularDielectric(Material *material) {
	return ((Specular*)(material->data))->dielectric;
}

Material specularBase;

void specularInit() {
	specularBase = (Material){
		.f = specularF,
		.sampleF = specularSampleF,
		.emiss = specularEmiss,
		.dielectric = specularDielectric
	};
}

Material newSpecular(Triple color, Triple emiss, bool dielectric) {
	Specular* specularData = malloc(sizeof(Specular));
	specularData->col = color;
	specularData->emiss = emiss;
	specularData->dielectric = dielectric;

	Material specular = specularBase;
	specular.data = specularData;

	return specular;
}

typedef struct {
	Triple col;
} Refractive;

Triple refractiveF(Material* material, Triple *wi, Triple *wo, Triple *normal);
SampleHit refractiveSampleF(Material* material, Triple *normal, Triple *wo);
Triple refractiveEmiss(Material* materal);

Triple refractiveF(Material* material, Triple *wi, Triple *wo, Triple *normal) {
	return ((Refractive*)material->data)->col;
}

SampleHit refractiveSampleF(Material* material, Triple *normal, Triple *wo) {
	Triple wi = scale(wo, -1);
	Triple newNormal = scale(normal, 2);
	scalePointer(&newNormal, innerProduct(normal, wo));
	addPointer(&wi, &newNormal);
	normalize(&wi);
	return (SampleHit){
		.dir = wi,
		.pdf = innerProduct(normal, &wi)
	};
}

Triple refractiveEmiss(Material *material) {
	// TODO Worth supporting this?
	return (Triple){0, 0, 0};
}

bool refractiveDielectric(Material *material) {
	return true;
}

Material refractiveBase;

void refractiveInit() {
	refractiveBase = (Material){
		.f = refractiveF,
		.sampleF = refractiveSampleF,
		.emiss = refractiveEmiss,
		.dielectric = refractiveDielectric
	};
}

Material newRefractive(Triple color) {
	Refractive* refractiveData = malloc(sizeof(Refractive));
	refractiveData->col = color;

	Material refractive = refractiveBase;
	refractive.data = refractiveData;

	return refractive;
}

typedef struct {
	Triple pos;
	double rad;
	double invRad;
} Sphere;

SimpleHit sphereIntersect(Object *object, Ray *ray) {
	Sphere* sphere = (Sphere*)(object->data);

	Triple op = subtract(&sphere->pos, &ray->org);
	double b = innerProduct(&op, &ray->dir);
	double deter = b * b - innerProduct(&op, &op) + sphere->rad * sphere->rad;

	if (deter < 0.0) {
		return (SimpleHit){.t = INFINITY};
	}

	deter = sqrt(deter);

	double t = b - deter;

	if (t > epsilon) {
		return (SimpleHit){.t = t, .object = object, .normal = object->normal(object, ray, t)};
	}

	t = b + deter;

	if (t > epsilon) {
		return (SimpleHit){.t = t, .object = object, .normal = object->normal(object, ray, t)};
	}

	return (SimpleHit){.t = INFINITY};
}

Triple sphereNormal(Object *object, Ray *ray, double t) {
	Sphere* sphere = (Sphere*)(object->data);

	Triple hitPoint = rayHit(ray, t);
	Triple unnormalized = subtract(&hitPoint, &sphere->pos);
	return scale(&unnormalized, sphere->invRad);
}

void spherePrint(Object *object) {
	Sphere* sphere = (Sphere*)(object->data);
	debug(
		"Sphere - center: (%f, %f, %f), radius: %f",
		sphere->pos.v[0], sphere->pos.v[1], sphere->pos.v[2],
		sphere->rad);
}

Object sphereBase;

void sphereInit() {
	sphereBase = (Object){
		.intersect = sphereIntersect,
		.normal = sphereNormal,
		.print = spherePrint
	};
}

Object newSphere(Triple position, double radius, Material* material) {
	Sphere* sphereData = malloc(sizeof(Sphere));
	sphereData->pos = position;
	sphereData->rad = radius;
	sphereData->invRad = 1.0 / radius;

	Object sphere = sphereBase;
	sphere.data = sphereData;
	sphere.material = material;

	return sphere;
}

struct Plane {
	Triple pos;
	Triple normal;
};

SimpleHit planeIntersect(Object *object, Ray *ray) {
	// Based on equation at https://www.cl.cam.ac.uk/teaching/1999/AGraphHCI/SMAG/node2.html

	Plane* plane = (Plane*)(object->data);

	double bottom = innerProduct(&plane->normal, &ray->dir);

	if (bottom > -epsilon && bottom < epsilon) {
		return (SimpleHit){.t = INFINITY};
	}

	Triple posSubOrg = subtract(&plane->pos, &ray->org);
	double top = innerProduct(&plane->normal, &posSubOrg);
	top /= bottom;

	if (top < epsilon) {
		return (SimpleHit){.t = INFINITY};
	}

	return (SimpleHit){.t = top, .object = object, .normal = plane->normal};
}

Triple planeNormal(Object *object, Ray *ray, double t) {
	return ((Plane*)(object->data))->normal;
}

void planePrint(Object *object) {
	Plane* plane = (Plane*)(object->data);
	debug("Plane - point: (%f, %f, %f), normal: (%f, %f, %f)",
		plane->pos.v[0], plane->pos.v[1], plane->pos.v[2],
		plane->normal.v[0], plane->normal.v[1], plane->normal.v[2]);
}

Object planeBase;

void planeInit() {
	planeBase = (Object){
		.intersect = planeIntersect,
		.normal = planeNormal,
		.print = planePrint
	};
}

Object newPlane(Triple position, Triple normal, Material* material) {
	Plane* planeData = malloc(sizeof(Plane));
	planeData->pos = position;
	normalize(&normal);
	planeData->normal = normal;

	Object plane = planeBase;
	plane.data = planeData;
	plane.material = material;

	return plane;
}

struct Triangle {
	Triple v[3];
	Triple a;
	Triple b;
	Triple normal;
};

SimpleHit triangleIntersect(Object *object, Ray *ray) {
	// Based on Möller-Trumbore implementation at:
	// http://www.scratchapixel.com/lessons/3d-basic-rendering/ray-tracing-rendering-a-triangle/moller-trumbore-ray-triangle-intersection

	Triangle* triangle = (Triangle*)(object->data);

	Triple pvec = crossProduct(&ray->dir, &triangle->b);
	double det = innerProduct(&triangle->a, &pvec);

	if (abs(det) < epsilon) {
		return (SimpleHit){.t = INFINITY};
	}

	double invDet = 1.0 / det;

	Triple tvec = subtract(&ray->org, &triangle->v[0]);
	double u = innerProduct(&tvec, &pvec) * invDet;
	if (u < 0 || u > 1) {
		return (SimpleHit){.t = INFINITY};
	}

	Triple qvec = crossProduct(&tvec, &triangle->a);
	double v = innerProduct(&ray->dir, &qvec) * invDet;
	if (v < 0 || u + v > 1) {
		return (SimpleHit){.t = INFINITY};
	}

	double t = innerProduct(&triangle->b, &qvec) * invDet;
	if (t > -epsilon && t < epsilon) {
		return (SimpleHit){.t = INFINITY};
	}

	return (SimpleHit){.t = t, .object = object, .normal = triangle->normal};
}

Triple triangleNormal(Object *object, Ray *ray, double t) {
	return ((Triangle*)(object->data))->normal;
}

void trianglePrint(Object *object) {
	Triangle* triangle = (Triangle*)(object->data);
	debug("Triangle - (%f, %f, %f), (%f, %f, %f), (%f, %f, %f)",
		triangle->v[0].v[0], triangle->v[0].v[1], triangle->v[0].v[2],
		triangle->v[1].v[0], triangle->v[1].v[1], triangle->v[1].v[2],
		triangle->v[2].v[0], triangle->v[2].v[1], triangle->v[2].v[2]);
}

Object triangleBase;

void triangleInit() {
	triangleBase = (Object){
		.intersect = triangleIntersect,
		.normal = triangleNormal,
		.print = trianglePrint
	};
}

Object newTriangle(Triple v0, Triple v1, Triple v2, Material* material) {
	Triangle* triangleData = malloc(sizeof(Triangle));
	triangleData->v[0] = v0;
	triangleData->v[1] = v1;
	triangleData->v[2] = v2;
	triangleData->a = subtract(&v1, &v0);
	triangleData->b = subtract(&v2, &v0);
	triangleData->normal = crossProduct(&triangleData->a, &triangleData->b);
	normalize(&triangleData->normal);

	Object triangle = triangleBase;
	triangle.data = triangleData;
	triangle.material = material;

	return triangle;
}

Triple sampleHemi(double u1, double u2, double exp) {
	double z = 1.0 - u1;
	double phi = twoPi * u2;

	double theta = 1.0 - (z * z);
	if (theta < 0.0) {
		theta = 0.0;
	}

	theta = sqrt(theta);

	return (Triple){{theta * cos(phi), theta * sin(phi), z}};
}

void normalize(Triple* v) {
	double s = sqrt((v->v[0] * v->v[0]) + (v->v[1] * v->v[1]) + (v->v[2] * v->v[2]));
	v->v[0] /= s;
	v->v[1] /= s;
	v->v[2] /= s;
}

Triple orientedHemiDir(double u1, double u2, Triple *normal, double exp) {
	Triple p = sampleHemi(u1, u2, exp);
	Triple randVector = (Triple){{doubleRand(), doubleRand(), doubleRand()}};
	Triple v = crossProduct(&randVector, normal);
	normalize(&v);
	Triple u = crossProduct(&v, normal);
	normalize(&u);
	Triple f1 = scale(&u, p.v[0]);
	Triple f2 = scale(&v, p.v[1]);
	Triple f3 = scale(normal, p.v[2]);
	Triple result = add(&f1, &f2);
	// HACK Is it safe to have the receiver also be an arg here?
	result = add(&result, &f3);
	normalize(&result);
	return result;
}

Triple orientNormal(Triple *normal, Triple *wo) {
	// TODO Should this be using epsilon.
	if (innerProduct(normal, wo) < 0) {
		return scale(normal, -1);
	}
	Triple newNormal = *normal;
	return newNormal;
}

int main() {
	BLACK = (Triple){{0, 0, 0}};
	MAGENTA = (Triple){{1.0, 0, 1.0}};

	emitterInit();
	diffuseInit();
	specularInit();
	refractiveInit();

	sphereInit();
	planeInit();
	triangleInit();

	Material whiteLight = newEmitter((Triple){1.25, 1.125, 0.875}, false);

	// Wall, ceiling, and floor materials.
	Material whiteDiffuse = newDiffuse((Triple){1, 1, 1}, (Triple){0, 0, 0}, false);
	Material greenDiffuse = newDiffuse((Triple){0, 1, 0}, (Triple){0, 0, 0}, false);
	Material redDiffuse = newDiffuse((Triple){1, 0, 0}, (Triple){0, 0, 0}, false);
	Material blueDiffuse = newDiffuse((Triple){0, 0, 1}, (Triple){0, 0, 0}, false);

	// Unique materials.
	Material whiteMirror = newSpecular((Triple){1, 1, 1}, (Triple){0, 0, 0}, false);
	Material cyanSpecular = newSpecular((Triple){0.2, 0.6, 0.6}, (Triple){0, 0, 0}, false);
	Material glass = newRefractive((Triple){0.8, 0.999, 0.1});
	Material pinkDiff = newDiffuse((Triple){0.9, 0.25, 0.9}, (Triple){0, 0, 0}, false);

	const int numObjects = 12;

	Object objects[12] = {
		newSphere((Triple){0, -40, 0}, 24.0, (Material*)&whiteLight),

		// Ceiling.
		newPlane((Triple){0, -20, 0}, (Triple){0, 1, 0}, (Material*)&whiteDiffuse),

		// Back wall.
		newPlane((Triple){0, 0, 20}, (Triple){0, 0, -1}, (Material*)&whiteDiffuse),

		// Front wall.
		newPlane((Triple){0, 0, -80}, (Triple){0, 0, 1}, (Material*)&whiteDiffuse),

		// Floor.
		newPlane((Triple){0, 20, 0}, (Triple){0, -1, 0}, (Material*)&whiteDiffuse),

		// Left wall.
		newPlane((Triple){-20, 0, 0}, (Triple){1, 0, 0}, (Material*)&greenDiffuse),

		// Right wall.
		newPlane((Triple){20, 0, 0}, (Triple){-1, 0, 0}, (Material*)&redDiffuse),

		// Blue ball back-right.
		newSphere((Triple){-6, 0, 14}, 6, (Material*)&blueDiffuse),

		// White specular back-left.
		newSphere((Triple){8, 0, 12}, 8, (Material*)&whiteMirror),

		// Plane cutting back-left corner.
		newPlane((Triple){16, -16, 0}, (Triple){1, -1, 1}, (Material*)&cyanSpecular),

		// First refractive.
		newSphere((Triple){-8, 10, 0}, 8, (Material*)&glass),

		// Vertical triangle, front-right.
		newTriangle((Triple){-19, 4, -10}, (Triple){-10, 19, -10}, (Triple){-19, 19, -20}, &pinkDiff),
	};

	Camera camera = newCamera(
		(Triple){{0, 0, -60}},
		(Triple){{0, 0, 0}},
		400,
		(Triple){{0, 1, 0}});

	///////// START TEST CODE
	/* Sphere* sp = (Sphere*)(objects[0].data); */

	/* Ray test = (Ray){ */
	/* 	.org = (Triple){{0, 0, 0}}, */
	/* 	.dir = (Triple){{-8, 10, 0}} */
	/* }; */

	/* Ray test = (Ray){ */
	/* 	.org = (Triple){{-3.002440, 3.753050, 0.000000}}, */
	/* 	.dir = (Triple){{-0.624695, 0.780869, 0.000000}} */
	/* }; */

	/* normalize(&test.dir); */
	/* printf("norm dir: %f, %f, %f\n", test.dir.v[0], test.dir.v[1], test.dir.v[2]); */

	/* Triple rad = radiance(numObjects, objects, &test, 0); */
	/* printf("rad: %f, %f, %f\n", rad.v[0], rad.v[1], rad.v[2]); */
	/* return 0; */

	/* SimpleHit hit = intersectObjects(numObjects, objects, &test); */

	/* printf("First hit: %f\n", hit.t); */
	//////// END TEST CODE

	char* buffer = malloc(sizeof(char) * width * height * 3);

	render(numObjects, objects, &camera, buffer);

	writePPM("output.ppm", buffer);

	free(buffer);
}

void writePPM(char* path, char* buffer) {
	FILE* out = fopen(path, "wb");
	fprintf(out, "P6\n %d\n %d\n 255\n", width, height);
	fwrite(buffer, sizeof(char) * width * height * 3, 1, out);
	fclose(out);
}

SimpleHit intersectObjects(int numObjects, Object* objects, Ray *ray) {
	SimpleHit firstHit = (SimpleHit){.t = INFINITY};

	Sphere* sp = (Sphere*)(objects[0].data);

	SimpleHit hit;
	for (int i = 0; i < numObjects; i++) {
		hit = objects[i].intersect(&objects[i], ray);
		if (hit.t < firstHit.t) {
			firstHit = hit;
		}
	}

	return firstHit;
}

Triple radiance(int numObjects, Object* objects, Ray* ray, int depth) {
	if (depth > maxDepth) {
		return BLACK;
	}

	SimpleHit hit = intersectObjects(numObjects, objects, ray);
	if (isinf(hit.t)) {
		return BLACK;
	}

	Material* mat = hit.object->material;

	if (!mat->dielectric(mat)) {
		Triple wo = scale(&ray->dir, -1);
		Triple normal = orientNormal(&hit.normal, &wo);
		SampleHit sample = mat->sampleF(mat, &normal, &wo);
		Triple f = mat->f(mat, &sample.dir, &wo, &normal);

		Ray newRay = (Ray){.org = rayHit(ray, hit.t), .dir = sample.dir};

		Triple nextDepth = radiance(numObjects, objects, &newRay, depth + 1);

		Triple nextColor = multiplyParts(&nextDepth, &f);

		Triple result = scale(&nextColor, innerProduct(&sample.dir, &normal) / sample.pdf);

		Triple emiss = mat->emiss(mat);
		return add(&result, &emiss);
	}

	Ray newRay = (Ray){.org = rayHit(ray, hit.t), .dir = ray->dir};

	return diRadiance(numObjects, objects, &newRay, &hit.normal, mat, depth);
}

Triple diRadiance(int numObjects, Object* objects, Ray* ray, Triple* normal, Material* mat, int depth) {
	Triple doubleNormal = scale(normal, 2);
	scalePointer(&doubleNormal, innerProduct(normal, &ray->dir));
	Triple reflDir = subtract(&ray->dir, &doubleNormal);

	Triple orientedNormal = orientNormal(normal, &ray->dir);
	scalePointer(&orientedNormal, -1);

	// TODO BUGBUG Epsilon check here?
	bool into = innerProduct(normal, &orientedNormal) > 0.0;

	double nc = 1.0;
	double nt = 1.5;

	double ddn = innerProduct(&ray->dir, &orientedNormal);

	double nnt = into ? nc / nt : nt / nc;

	double cos2t = 1 - nnt * nnt * (1 - ddn * ddn);

	// TODO BUGBUG Another epsilon check?
	if (cos2t < 0.0) {
		Ray newRay = (Ray){.org = ray->org, .dir = reflDir};
		Triple result = mat->emiss(mat);
		Triple nextResult = radiance(numObjects, objects, &newRay, depth + 1);
		addPointer(&result, &nextResult);
		return result;
	}

	double intoTerm = into ? 1.0 : -1.0;

	Triple tdir = scale(&ray->dir, nnt);
	Triple normTerm = scale(normal, intoTerm);
	scalePointer(&normTerm, (ddn * nnt + sqrt(cos2t)));
	subtractPointer(&tdir, &normTerm);
	normalize(&tdir);

	double a = nt - nc;
	double b = nt + nc;
	double r0 = a * a / (b * b);

	double c = into ? 1 + ddn : 1 - innerProduct(&tdir, normal);

	double re = r0 + (1 - r0) * pow(c, 4);
	double tr = 1 - re;
	double p = 0.25 + 0.5 * re;
	double rp = re / p;
	double tp = tr / (1 - p);

	Triple result;

	if (depth <= 2) {
		Ray reflectRay = (Ray){.org = ray->org, .dir = reflDir};
		result = radiance(numObjects, objects, &reflectRay, depth + 1);
		scalePointer(&result, re);

		Ray refractRay = (Ray){.org = ray->org, .dir = tdir};

		Triple refractRad = radiance(numObjects, objects, &refractRay, depth + 1);
		scalePointer(&refractRad, tr);

		addPointer(&result, &refractRad);
	} else {
		if (doubleRand() < p) {
			Ray reflectRay = (Ray){.org = ray->org, .dir = reflDir};
			result = radiance(numObjects, objects, &reflectRay, depth + 1);
			scalePointer(&result, rp);
		} else {
			Ray refractRay = (Ray){.org = ray->org, .dir = tdir};
			result = radiance(numObjects, objects, &refractRay, depth + 1);
			scalePointer(&result, tp);
		}
	}

	// TODO Is this the right thing to do? It looks alright :/
	SampleHit sample = mat->sampleF(mat, normal, &ray->dir);
	scalePointer(&result, innerProduct(&sample.dir, normal) / sample.pdf);

	Triple emiss = mat->emiss(mat);
	addPointer(&result, &emiss);
	return result;
}


void render(int numObjects, Object* objects, Camera* camera, char* buffer) {
	cameraCalcOrthonormalBasis(camera);

	double sppInv = 1.0 / (double)spp;
	double halfWidth = (double)width / 2;
	double halfHeight = (double)height / 2;

#pragma omp parallel for schedule(dynamic, 1)
	for (int y = 0; y < height; y++) {
	  double dY = (double)y;
		for (int x = 0; x < width; x++) {
			Triple px = (Triple){{0, 0, 0}};

			for (int s = 0; s < spp; s++) {
				double sx = ((double)x + doubleRand()) - halfWidth;
				double sy = (dY + doubleRand()) - halfHeight;
				Ray ray = cameraSpawnRay(camera, sx, sy);
				Triple rad = radiance(numObjects, objects, &ray, 0);
				Triple scaled = scale(&rad, sppInv);
				addPointer(&px, &scaled);
			}

			int i = ((y * width) + x) * 3;
			buffer[i] = gammaTransform(px.v[0]);
			buffer[i + 1] = gammaTransform(px.v[1]);
			buffer[i + 2] = gammaTransform(px.v[2]);
		}
	}
}

Camera newCamera(Triple eye, Triple focal, double viewDist, Triple up) {
	return (Camera){
		.eye = eye,
		.focal = focal,
		.viewDist = viewDist,
		.up = up
	};
}

void cameraCalcOrthonormalBasis(Camera* camera) {
	Triple w1 = subtract(&camera->eye, &camera->focal);
	normalize(&w1);
	camera->w = w1;
	camera->wMultViewDist = scale(&camera->w, camera->viewDist);

	Triple u1 = crossProduct(&camera->up, &camera->w);
	normalize(&u1);
	camera->u = u1;

	camera->v = crossProduct(&camera->w, &camera->u);
}

Ray cameraSpawnRay(Camera* camera, double x, double y) {
	Triple dir = scale(&camera->u, x);

	Triple vMultY = scale(&camera->v, y);
	addPointer(&dir, &vMultY);
	subtractPointer(&dir, &camera->wMultViewDist);
	normalize(&dir);
	return (Ray){
		.org = camera->eye,
		.dir = dir
	};
}

double clamp(double x) {
	if (x < 0) {
		return 0;
	}
	if (x > 1) {
		return 1;
	}
	return x;
}

unsigned char gammaTransform(double x) {
	return (unsigned char)floor(pow(clamp(x), 1 / 2.2) * 255 + 0.5);
}
