#ifndef UNICODE
#define UNICODE
#endif
#ifndef _UNICODE
#define _UNICODE
#endif

#include <windows.h>
#include <commctrl.h>
#include <iostream>
#include <vector>
#include <string>
#include <memory>
#include <thread>
#include <atomic>
#include <mutex>
#include <chrono>
#include <cmath>
#include <random>
#include <algorithm>
#include <cstdint>

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"

#pragma comment(lib, "comctl32.lib")
#pragma comment(lib, "comdlg32.lib")
#include <commdlg.h>


// ============================================================================
// 1. MATEMATYKA PATH TRACINGU (Vec3, Ray, HitRecord, Material)
// ============================================================================

struct Vec3 {
    float x = 0.0f, y = 0.0f, z = 0.0f;

    Vec3() = default;
    Vec3(float x_, float y_, float z_) : x(x_), y(y_), z(z_) {}

    Vec3 operator-() const { return Vec3(-x, -y, -z); }
    Vec3 operator+(const Vec3& v) const { return Vec3(x + v.x, y + v.y, z + v.z); }
    Vec3 operator-(const Vec3& v) const { return Vec3(x - v.x, y - v.y, z - v.z); }
    Vec3 operator*(const Vec3& v) const { return Vec3(x * v.x, y * v.y, z * v.z); }
    Vec3 operator*(float t) const { return Vec3(x * t, y * t, z * t); }
    Vec3 operator/(float t) const { return Vec3(x / t, y / t, z / t); }

    float length_squared() const { return x * x + y * y + z * z; }
    float length() const { return std::sqrt(length_squared()); }

    bool near_zero() const {
        constexpr float eps = 1e-8f;
        return (std::fabs(x) < eps) && (std::fabs(y) < eps) && (std::fabs(z) < eps);
    }
};

inline Vec3 operator*(float t, const Vec3& v) { return v * t; }
inline float dot(const Vec3& u, const Vec3& v) { return u.x * v.x + u.y * v.y + u.z * v.z; }
inline Vec3 cross(const Vec3& u, const Vec3& v) {
    return Vec3(u.y * v.z - u.z * v.y,
                u.z * v.x - u.x * v.z,
                u.x * v.y - u.y * v.x);
}
inline Vec3 unit_vector(const Vec3& v) {
    float l = v.length();
    return l > 1e-8f ? v / l : Vec3(0, 0, 0);
}

inline Vec3 random_unit_vector(std::mt19937& rng) {
    std::uniform_real_distribution<float> dist(-1.0f, 1.0f);
    while (true) {
        Vec3 p(dist(rng), dist(rng), dist(rng));
        float len_sq = p.length_squared();
        if (len_sq > 1e-6f && len_sq <= 1.0f) {
            return p / std::sqrt(len_sq);
        }
    }
}

inline Vec3 reflect(const Vec3& v, const Vec3& n) {
    return v - 2.0f * dot(v, n) * n;
}

inline Vec3 refract(const Vec3& uv, const Vec3& n, float etai_over_etat) {
    float cos_theta = std::fmin(dot(-uv, n), 1.0f);
    Vec3 r_out_perp = etai_over_etat * (uv + cos_theta * n);
    Vec3 r_out_parallel = -std::sqrt(std::fabs(1.0f - r_out_perp.length_squared())) * n;
    return r_out_perp + r_out_parallel;
}

inline float schlick_reflectance(float cosine, float ref_idx) {
    float r0 = (1.0f - ref_idx) / (1.0f + ref_idx);
    r0 = r0 * r0;
    return r0 + (1.0f - r0) * std::pow(1.0f - cosine, 5.0f);
}

struct Ray {
    Vec3 origin;
    Vec3 direction;
    Ray() = default;
    Ray(const Vec3& o, const Vec3& d) : origin(o), direction(d) {}
    Vec3 at(float t) const { return origin + t * direction; }
};

class Material;

struct HitRecord {
    Vec3 p;
    Vec3 normal;
    std::shared_ptr<Material> mat;
    float t = 0.0f;
    bool front_face = true;

    void set_face_normal(const Ray& r, const Vec3& outward_normal) {
        front_face = dot(r.direction, outward_normal) < 0;
        normal = front_face ? outward_normal : -outward_normal;
    }
};

class Material {
public:
    virtual ~Material() = default;
    virtual bool scatter(const Ray& r_in, const HitRecord& rec, Vec3& attenuation, Ray& scattered, std::mt19937& rng) const {
        return false;
    }
    virtual Vec3 emitted(const Vec3& p) const { return Vec3(0, 0, 0); }
    virtual bool is_specular() const { return false; }
    virtual bool is_dielectric() const { return false; }
    virtual Vec3 get_albedo() const { return Vec3(0, 0, 0); }
};

class Lambertian : public Material {
public:
    Vec3 albedo;
    explicit Lambertian(const Vec3& a) : albedo(a) {}

    bool scatter(const Ray& r_in, const HitRecord& rec, Vec3& attenuation, Ray& scattered, std::mt19937& rng) const override {
        Vec3 scatter_direction = rec.normal + random_unit_vector(rng);
        if (scatter_direction.near_zero()) scatter_direction = rec.normal;
        scattered = Ray(rec.p, unit_vector(scatter_direction));
        attenuation = albedo;
        return true;
    }
    Vec3 get_albedo() const override { return albedo; }
};

class Metal : public Material {
public:
    Vec3 albedo;
    float roughness;
    Metal(const Vec3& a, float r) : albedo(a), roughness(r < 1.0f ? r : 1.0f) {}

    bool scatter(const Ray& r_in, const HitRecord& rec, Vec3& attenuation, Ray& scattered, std::mt19937& rng) const override {
        Vec3 reflected = reflect(unit_vector(r_in.direction), rec.normal);
        scattered = Ray(rec.p, unit_vector(reflected + roughness * random_unit_vector(rng)));
        attenuation = albedo;
        return (dot(scattered.direction, rec.normal) > 0);
    }
    bool is_specular() const override { return roughness < 0.15f; }
    Vec3 get_albedo() const override { return albedo; }
};

class Dielectric : public Material {
public:
    float ir;
    explicit Dielectric(float index_of_refraction) : ir(index_of_refraction) {}

    bool scatter(const Ray& r_in, const HitRecord& rec, Vec3& attenuation, Ray& scattered, std::mt19937& rng) const override {
        attenuation = Vec3(1.0f, 1.0f, 1.0f);
        float refraction_ratio = rec.front_face ? (1.0f / ir) : ir;

        Vec3 unit_direction = unit_vector(r_in.direction);
        float cos_theta = std::fmin(dot(-unit_direction, rec.normal), 1.0f);
        float sin_theta = std::sqrt(std::max(0.0f, 1.0f - cos_theta * cos_theta));

        bool cannot_refract = refraction_ratio * sin_theta > 1.0f;
        std::uniform_real_distribution<float> dist01(0.0f, 1.0f);

        Vec3 direction;
        if (cannot_refract || schlick_reflectance(cos_theta, refraction_ratio) > dist01(rng)) {
            direction = reflect(unit_direction, rec.normal);
        } else {
            direction = refract(unit_direction, rec.normal, refraction_ratio);
        }

        scattered = Ray(rec.p, direction);
        return true;
    }
    bool is_specular() const override { return true; }
    bool is_dielectric() const override { return true; }
    Vec3 get_albedo() const override { return Vec3(1.0f, 1.0f, 1.0f); }
};

class DiffuseLight : public Material {
public:
    Vec3 emit_color;
    explicit DiffuseLight(const Vec3& c) : emit_color(c) {}
    Vec3 emitted(const Vec3& p) const override { return emit_color; }
    Vec3 get_albedo() const override { return emit_color; }
};

// ============================================================================
// 2. GEOMETRIA: SFERA ORAZ SZLIFOWANY KRYSZTAŁ SZKLANY (CONVEX POLYHEDRON)
// ============================================================================

class Hittable {
public:
    virtual ~Hittable() = default;
    virtual bool hit(const Ray& r, float t_min, float t_max, HitRecord& rec) const = 0;
};

class Sphere : public Hittable {
public:
    Vec3 center;
    float radius;
    std::shared_ptr<Material> mat;

    Sphere(const Vec3& c, float r, std::shared_ptr<Material> m)
        : center(c), radius(r), mat(std::move(m)) {}

    bool hit(const Ray& r, float t_min, float t_max, HitRecord& rec) const override {
        Vec3 oc = r.origin - center;
        float a = r.direction.length_squared();
        float half_b = dot(oc, r.direction);
        float c = oc.length_squared() - radius * radius;
        float discriminant = half_b * half_b - a * c;

        if (discriminant < 0) return false;
        float sqrtd = std::sqrt(discriminant);

        float root = (-half_b - sqrtd) / a;
        if (root < t_min || t_max < root) {
            root = (-half_b + sqrtd) / a;
            if (root < t_min || t_max < root) return false;
        }

        rec.t = root;
        rec.p = r.at(rec.t);
        Vec3 outward_normal = (rec.p - center) / radius;
        rec.set_face_normal(r, outward_normal);
        rec.mat = mat;
        return true;
    }
};

struct PolyPlane {
    Vec3 n;
    float d;
};

class ConvexPolyhedron : public Hittable {
public:
    Vec3 center;
    std::vector<PolyPlane> planes;
    std::shared_ptr<Material> mat;

    ConvexPolyhedron(const Vec3& c, std::shared_ptr<Material> m)
        : center(c), mat(std::move(m)) {}

    void add_plane(const Vec3& normal, float dist) {
        planes.push_back({unit_vector(normal), dist});
    }

    bool hit(const Ray& r, float t_min, float t_max, HitRecord& rec) const override {
        float t_enter = -1e9f;
        float t_exit  = 1e9f;
        Vec3 n_enter(0.0f, 1.0f, 0.0f);
        Vec3 n_exit(0.0f, 1.0f, 0.0f);

        Vec3 ro = r.origin - center;

        for (const auto& plane : planes) {
            float denom = dot(plane.n, r.direction);
            float numer = plane.d - dot(plane.n, ro);

            if (std::fabs(denom) < 1e-7f) {
                if (numer < 0.0f) {
                    return false;
                }
            } else if (denom > 0.0f) {
                float t = numer / denom;
                if (t < t_exit) {
                    t_exit = t;
                    n_exit = plane.n;
                }
            } else {
                float t = numer / denom;
                if (t > t_enter) {
                    t_enter = t;
                    n_enter = plane.n;
                }
            }
            if (t_enter > t_exit) return false;
        }

        float t_hit = -1.0f;
        Vec3 hit_normal;

        if (t_enter >= t_min && t_enter <= t_max) {
            t_hit = t_enter;
            hit_normal = n_enter;
        } else if (t_exit >= t_min && t_exit <= t_max) {
            t_hit = t_exit;
            hit_normal = n_exit;
        } else {
            return false;
        }

        rec.t = t_hit;
        rec.p = r.at(t_hit);
        rec.set_face_normal(r, hit_normal);
        rec.mat = mat;
        return true;
    }
};

std::shared_ptr<ConvexPolyhedron> CreateFacetedCrystal(const Vec3& center, float radius, float height, std::shared_ptr<Material> mat) {
    auto crystal = std::make_shared<ConvexPolyhedron>(center, mat);
    float half_h = height * 0.5f;

    crystal->add_plane(Vec3(0.0f, -1.0f, 0.0f), half_h);
    crystal->add_plane(Vec3(0.0f, 1.0f, 0.0f), half_h);

    for (int i = 0; i < 6; ++i) {
        float angle = i * (3.14159265f / 3.0f);
        crystal->add_plane(Vec3(std::cos(angle), 0.0f, std::sin(angle)), radius);
    }

    for (int i = 0; i < 6; ++i) {
        float angle = (i + 0.5f) * (3.14159265f / 3.0f);
        Vec3 slanted_norm = unit_vector(Vec3(std::cos(angle) * 0.85f, 1.15f, std::sin(angle) * 0.85f));
        crystal->add_plane(slanted_norm, radius * 1.08f);
    }

    for (int i = 0; i < 6; ++i) {
        float angle = i * (3.14159265f / 3.0f);
        Vec3 slanted_norm = unit_vector(Vec3(std::cos(angle) * 0.80f, -1.25f, std::sin(angle) * 0.80f));
        crystal->add_plane(slanted_norm, radius * 1.18f);
    }

    return crystal;
}

class Quad : public Hittable {
public:
    Vec3 q;
    Vec3 u;
    Vec3 v;
    Vec3 normal;
    float d;
    Vec3 w;
    std::shared_ptr<Material> mat;

    Quad(const Vec3& corner, const Vec3& u_, const Vec3& v_, std::shared_ptr<Material> m)
        : q(corner), u(u_), v(v_), mat(std::move(m)) {
        Vec3 n = cross(u, v);
        normal = unit_vector(n);
        d = dot(normal, q);
        float len_sq = dot(n, n);
        w = len_sq > 1e-8f ? (n / len_sq) : Vec3(0, 0, 0);
    }

    bool hit(const Ray& r, float t_min, float t_max, HitRecord& rec) const override {
        float denom = dot(normal, r.direction);
        if (std::fabs(denom) < 1e-8f) return false;

        float t = (d - dot(normal, r.origin)) / denom;
        if (t < t_min || t > t_max) return false;

        Vec3 intersection = r.at(t);
        Vec3 planar_hit = intersection - q;
        float alpha = dot(w, cross(planar_hit, v));
        float beta  = dot(w, cross(u, planar_hit));

        if (alpha < 0.0f || alpha > 1.0f || beta < 0.0f || beta > 1.0f) {
            return false;
        }

        rec.t = t;
        rec.p = intersection;
        rec.mat = mat;
        rec.set_face_normal(r, normal);
        return true;
    }
};

struct Triangle {
    Vec3 v0, v1, v2;
    Vec3 normal;
};

inline bool hit_aabb(const Ray& r, float t_min, float t_max, const Vec3& b_min, const Vec3& b_max) {
    for (int a = 0; a < 3; ++a) {
        float r_dir  = (a == 0) ? r.direction.x : ((a == 1) ? r.direction.y : r.direction.z);
        float r_orig = (a == 0) ? r.origin.x    : ((a == 1) ? r.origin.y    : r.origin.z);
        float min_v  = (a == 0) ? b_min.x       : ((a == 1) ? b_min.y       : b_min.z);
        float max_v  = (a == 0) ? b_max.x       : ((a == 1) ? b_max.y       : b_max.z);
        float invD   = 1.0f / (std::fabs(r_dir) > 1e-8f ? r_dir : (r_dir < 0 ? -1e-8f : 1e-8f));
        float t0 = (min_v - r_orig) * invD;
        float t1 = (max_v - r_orig) * invD;
        if (invD < 0.0f) std::swap(t0, t1);
        t_min = t0 > t_min ? t0 : t_min;
        t_max = t1 < t_max ? t1 : t_max;
        if (t_max <= t_min) return false;
    }
    return true;
}

inline bool hit_triangle(const Ray& r, float t_min, float t_max, const Vec3& v0, const Vec3& v1, const Vec3& v2, const Vec3& normal, HitRecord& rec) {
    Vec3 edge1 = v1 - v0;
    Vec3 edge2 = v2 - v0;
    Vec3 h = cross(r.direction, edge2);
    float a = dot(edge1, h);
    if (std::fabs(a) < 1e-7f) return false;
    float f = 1.0f / a;
    Vec3 s = r.origin - v0;
    float u = f * dot(s, h);
    if (u < 0.0f || u > 1.0f) return false;
    Vec3 q = cross(s, edge1);
    float v = f * dot(r.direction, q);
    if (v < 0.0f || u + v > 1.0f) return false;
    float t = f * dot(edge2, q);
    if (t < t_min || t > t_max) return false;
    rec.t = t;
    rec.p = r.at(t);
    rec.set_face_normal(r, normal);
    return true;
}

class TriangleMesh : public Hittable {
public:
    Vec3 center;
    float scale = 1.0f;
    std::vector<Triangle> triangles;
    Vec3 local_bmin;
    Vec3 local_bmax;
    std::shared_ptr<Material> mat;

    TriangleMesh(const Vec3& c, float s, std::vector<Triangle> tris, const Vec3& bmin, const Vec3& bmax, std::shared_ptr<Material> m)
        : center(c), scale(s), triangles(std::move(tris)), local_bmin(bmin), local_bmax(bmax), mat(std::move(m)) {}

    bool hit(const Ray& r, float t_min, float t_max, HitRecord& rec) const override {
        if (scale <= 1e-6f || triangles.empty()) return false;
        float inv_s = 1.0f / scale;
        Ray r_local((r.origin - center) * inv_s, r.direction * inv_s);

        if (!hit_aabb(r_local, t_min, t_max, local_bmin, local_bmax)) {
            return false;
        }

        HitRecord temp_rec;
        bool hit_any = false;
        float closest = t_max;

        for (const auto& tri : triangles) {
            if (hit_triangle(r_local, t_min, closest, tri.v0, tri.v1, tri.v2, tri.normal, temp_rec)) {
                hit_any = true;
                closest = temp_rec.t;
                rec = temp_rec;
            }
        }

        if (hit_any) {
            rec.p = r.at(rec.t);
            rec.mat = mat;
        }
        return hit_any;
    }
};

inline bool LoadOBJGeometry(const std::wstring& filename, std::vector<Triangle>& out_triangles, Vec3& out_bmin, Vec3& out_bmax) {
    FILE* fp = _wfopen(filename.c_str(), L"r");
    if (!fp) return false;

    std::vector<Vec3> temp_vertices;
    char line[1024];
    while (fgets(line, sizeof(line), fp)) {
        if (line[0] == 'v' && line[1] == ' ') {
            float x, y, z;
            if (sscanf(line + 2, "%f %f %f", &x, &y, &z) == 3) {
                temp_vertices.push_back(Vec3(x, y, z));
            }
        } else if (line[0] == 'f' && line[1] == ' ') {
            std::vector<int> face_indices;
            char* ptr = line + 2;
            while (*ptr) {
                while (*ptr && isspace((unsigned char)*ptr)) ptr++;
                if (!*ptr) break;
                int vi = 0;
                if (sscanf(ptr, "%d", &vi) == 1) {
                    if (vi > 0) vi -= 1;
                    else if (vi < 0) vi = static_cast<int>(temp_vertices.size()) + vi;
                    face_indices.push_back(vi);
                }
                while (*ptr && !isspace((unsigned char)*ptr)) ptr++;
            }
            for (size_t i = 1; i + 1 < face_indices.size(); ++i) {
                int i0 = face_indices[0];
                int i1 = face_indices[i];
                int i2 = face_indices[i + 1];
                if (i0 >= 0 && i0 < static_cast<int>(temp_vertices.size()) &&
                    i1 >= 0 && i1 < static_cast<int>(temp_vertices.size()) &&
                    i2 >= 0 && i2 < static_cast<int>(temp_vertices.size())) {
                    Vec3 v0 = temp_vertices[i0];
                    Vec3 v1 = temp_vertices[i1];
                    Vec3 v2 = temp_vertices[i2];
                    Vec3 norm = unit_vector(cross(v1 - v0, v2 - v0));
                    if (norm.length_squared() < 1e-6f) norm = Vec3(0, 1, 0);
                    out_triangles.push_back({v0, v1, v2, norm});
                }
            }
        }
    }
    fclose(fp);

    if (out_triangles.empty()) return false;

    Vec3 bmin(1e9f, 1e9f, 1e9f), bmax(-1e9f, -1e9f, -1e9f);
    for (const auto& tri : out_triangles) {
        bmin.x = std::min({bmin.x, tri.v0.x, tri.v1.x, tri.v2.x});
        bmin.y = std::min({bmin.y, tri.v0.y, tri.v1.y, tri.v2.y});
        bmin.z = std::min({bmin.z, tri.v0.z, tri.v1.z, tri.v2.z});
        bmax.x = std::max({bmax.x, tri.v0.x, tri.v1.x, tri.v2.x});
        bmax.y = std::max({bmax.y, tri.v0.y, tri.v1.y, tri.v2.y});
        bmax.z = std::max({bmax.z, tri.v0.z, tri.v1.z, tri.v2.z});
    }

    Vec3 center = 0.5f * (bmin + bmax);
    Vec3 extents = bmax - bmin;
    float max_dim = std::max({extents.x, extents.y, extents.z});
    if (max_dim < 1e-6f) max_dim = 1.0f;
    float norm_scale = 1.0f / max_dim;

    out_bmin = Vec3(1e9f, 1e9f, 1e9f);
    out_bmax = Vec3(-1e9f, -1e9f, -1e9f);

    for (auto& tri : out_triangles) {
        tri.v0 = (tri.v0 - center) * norm_scale;
        tri.v1 = (tri.v1 - center) * norm_scale;
        tri.v2 = (tri.v2 - center) * norm_scale;

        out_bmin.x = std::min({out_bmin.x, tri.v0.x, tri.v1.x, tri.v2.x});
        out_bmin.y = std::min({out_bmin.y, tri.v0.y, tri.v1.y, tri.v2.y});
        out_bmin.z = std::min({out_bmin.z, tri.v0.z, tri.v1.z, tri.v2.z});
        out_bmax.x = std::max({out_bmax.x, tri.v0.x, tri.v1.x, tri.v2.x});
        out_bmax.y = std::max({out_bmax.y, tri.v0.y, tri.v1.y, tri.v2.y});
        out_bmax.z = std::max({out_bmax.z, tri.v0.z, tri.v1.z, tri.v2.z});
    }
    out_bmin = out_bmin - Vec3(0.01f, 0.01f, 0.01f);
    out_bmax = out_bmax + Vec3(0.01f, 0.01f, 0.01f);

    return true;
}

class HittableList : public Hittable {
public:
    std::vector<std::shared_ptr<Hittable>> objects;

    void add(std::shared_ptr<Hittable> object) {
        objects.push_back(std::move(object));
    }

    void clear() { objects.clear(); }

    bool hit(const Ray& r, float t_min, float t_max, HitRecord& rec) const override {
        HitRecord temp_rec;
        bool hit_anything = false;
        float closest_so_far = t_max;

        for (const auto& object : objects) {
            if (object->hit(r, t_min, closest_so_far, temp_rec)) {
                hit_anything = true;
                closest_so_far = temp_rec.t;
                rec = temp_rec;
            }
        }
        return hit_anything;
    }
};

class Camera {
public:
    Vec3 origin;
    Vec3 lower_left_corner;
    Vec3 horizontal;
    Vec3 vertical;
    Vec3 lookat;
    Vec3 vup;

    Camera() = default;

    Camera(Vec3 lookfrom, Vec3 lookat_in, Vec3 vup_in, float vfov_deg, float aspect_ratio) {
        lookat = lookat_in;
        vup = vup_in;
        float theta = vfov_deg * 3.14159265358979323846f / 180.0f;
        float h = std::tan(theta / 2.0f);
        float viewport_height = 2.0f * h;
        float viewport_width = aspect_ratio * viewport_height;

        Vec3 w = unit_vector(lookfrom - lookat);
        Vec3 u = unit_vector(cross(vup, w));
        Vec3 v = cross(w, u);

        origin = lookfrom;
        horizontal = viewport_width * u;
        vertical = viewport_height * v;
        lower_left_corner = origin - horizontal / 2.0f - vertical / 2.0f - w;
    }

    Ray get_ray(float s, float t) const {
        return Ray(origin, lower_left_corner + s * horizontal + t * vertical - origin);
    }

    bool project_direction(const Vec3& dir, float& out_s, float& out_t) const {
        Vec3 w = unit_vector(origin - lookat);
        float alpha = dot(dir, -w);
        if (alpha <= 1e-5f) return false;

        Vec3 p_plane = (dir / alpha) - (-w);
        float u_len = horizontal.length();
        float v_len = vertical.length();
        if (u_len <= 1e-6f || v_len <= 1e-6f) return false;

        Vec3 u_dir = horizontal / u_len;
        Vec3 v_dir = vertical / v_len;

        out_s = 0.5f + dot(p_plane, u_dir) / u_len;
        out_t = 0.5f + dot(p_plane, v_dir) / v_len;
        return (out_s >= 0.0f && out_s <= 1.0f && out_t >= 0.0f && out_t <= 1.0f);
    }
};

// ============================================================================
// 3. GLOBALNA MAPA FOTONÓW KAUSTYKI (Eliminacja szwów kafelkowania)
// ============================================================================

struct Photon {
    Vec3 pos;
    Vec3 flux;
};

class PhotonMap {
public:
    std::vector<Photon> photons;
    float cell_size = 0.08f;
    float min_x = -5.0f;
    float max_x = 5.0f;
    float min_z = -5.0f;
    float max_z = 5.0f;
    int grid_w = 0;
    int grid_h = 0;
    std::vector<std::vector<uint32_t>> grid;

    void init(float c_size, float x0, float x1, float z0, float z1) {
        cell_size = c_size;
        min_x = x0; max_x = x1;
        min_z = z0; max_z = z1;
        grid_w = static_cast<int>(std::ceil((max_x - min_x) / cell_size));
        grid_h = static_cast<int>(std::ceil((max_z - min_z) / cell_size));
        grid.assign(grid_w * grid_h, std::vector<uint32_t>());
        photons.clear();
    }

    void add_photon(const Vec3& p, const Vec3& flux) {
        uint32_t idx = static_cast<uint32_t>(photons.size());
        photons.push_back({p, flux});
        int gx = static_cast<int>((p.x - min_x) / cell_size);
        int gz = static_cast<int>((p.z - min_z) / cell_size);
        if (gx >= 0 && gx < grid_w && gz >= 0 && gz < grid_h) {
            grid[gz * grid_w + gx].push_back(idx);
        }
    }

    Vec3 estimate_radiance(const Vec3& p, const Vec3& normal, float search_radius) const {
        if (photons.empty()) return Vec3(0, 0, 0);

        int gx0 = std::max(0, static_cast<int>((p.x - search_radius - min_x) / cell_size));
        int gx1 = std::min(grid_w - 1, static_cast<int>((p.x + search_radius - min_x) / cell_size));
        int gz0 = std::max(0, static_cast<int>((p.z - search_radius - min_z) / cell_size));
        int gz1 = std::min(grid_h - 1, static_cast<int>((p.z + search_radius - min_z) / cell_size));

        float r2 = search_radius * search_radius;
        Vec3 accum(0, 0, 0);
        float inv_area = 1.0f / (3.14159265f * r2);

        for (int gz = gz0; gz <= gz1; ++gz) {
            int row = gz * grid_w;
            for (int gx = gx0; gx <= gx1; ++gx) {
                const auto& cell = grid[row + gx];
                for (uint32_t p_idx : cell) {
                    const auto& ph = photons[p_idx];
                    float dx = p.x - ph.pos.x;
                    float dy = p.y - ph.pos.y;
                    float dz = p.z - ph.pos.z;
                    float dist_sq = dx * dx + dy * dy + dz * dz;
                    if (dist_sq < r2) {
                        float dist = std::sqrt(dist_sq);
                        float w = (1.0f - dist / search_radius);
                        accum = accum + ph.flux * (w * inv_area * 3.5f);
                    }
                }
            }
        }
        return accum;
    }
};

Vec3 ray_color_standard(const Ray& r, const HittableList& world, int depth, std::mt19937& rng) {
    if (depth <= 0) return Vec3(0, 0, 0);

    HitRecord rec;
    if (!world.hit(r, 0.001f, 1e9f, rec)) {
        Vec3 unit_direction = unit_vector(r.direction);
        float a = 0.5f * (unit_direction.y + 1.0f);
        return (1.0f - a) * Vec3(0.04f, 0.05f, 0.08f) + a * Vec3(0.12f, 0.16f, 0.25f);
    }

    Vec3 emitted = rec.mat->emitted(rec.p);
    Ray scattered;
    Vec3 attenuation;

    if (!rec.mat->scatter(r, rec, attenuation, scattered, rng)) {
        return emitted;
    }

    return emitted + attenuation * ray_color_standard(scattered, world, depth - 1, rng);
}

Vec3 ray_color_bdpt(const Ray& r, const HittableList& world, const std::vector<Sphere>& light_sources,
                    const PhotonMap& photon_map, int depth, std::mt19937& rng) {
    Vec3 accumulated_color(0.0f, 0.0f, 0.0f);
    Vec3 throughput(1.0f, 1.0f, 1.0f);
    Ray current_ray = r;
    bool last_was_specular = true;

    for (int bounce = 0; bounce < depth; ++bounce) {
        HitRecord rec;
        if (!world.hit(current_ray, 0.001f, 1e9f, rec)) {
            Vec3 unit_direction = unit_vector(current_ray.direction);
            float a = 0.5f * (unit_direction.y + 1.0f);
            Vec3 sky = (1.0f - a) * Vec3(0.04f, 0.05f, 0.08f) + a * Vec3(0.12f, 0.16f, 0.25f);
            accumulated_color = accumulated_color + throughput * sky;
            break;
        }

        Vec3 emitted = rec.mat->emitted(rec.p);
        if (!emitted.near_zero()) {
            if (last_was_specular) {
                accumulated_color = accumulated_color + throughput * emitted;
            }
            break;
        }

        Ray scattered;
        Vec3 albedo;
        if (!rec.mat->scatter(current_ray, rec, albedo, scattered, rng)) {
            break;
        }

        bool is_spec = rec.mat->is_specular();

        if (!is_spec && !light_sources.empty()) {
            const auto& light = light_sources[0];
            Vec3 light_pt = light.center + light.radius * random_unit_vector(rng);
            Vec3 to_light = light_pt - rec.p;
            float dist = to_light.length();
            Vec3 dir_light = to_light / dist;

            float cos_theta = dot(rec.normal, dir_light);
            if (cos_theta > 0.0f) {
                HitRecord shadow_rec;
                if (!world.hit(Ray(rec.p + rec.normal * 0.002f, dir_light), 0.001f, dist - 0.005f, shadow_rec)) {
                    Vec3 light_norm = unit_vector(light_pt - light.center);
                    float cos_light = std::fmax(0.0f, dot(light_norm, -dir_light));
                    float area = 4.0f * 3.14159f * light.radius * light.radius;
                    float G = (cos_theta * cos_light) / (dist * dist + 0.05f);

                    Vec3 direct = light.mat->emitted(light_pt) * albedo * (G * area * 0.159f);
                    direct.x = std::fmin(direct.x, 25.0f);
                    direct.y = std::fmin(direct.y, 25.0f);
                    direct.z = std::fmin(direct.z, 25.0f);
                    accumulated_color = accumulated_color + throughput * direct;
                }
            }
        }

        if (!is_spec && !photon_map.photons.empty()) {
            Vec3 caustic_flux = photon_map.estimate_radiance(rec.p, rec.normal, 0.12f);
            accumulated_color = accumulated_color + throughput * albedo * caustic_flux;
        }

        throughput = throughput * albedo;
        current_ray = scattered;
        last_was_specular = is_spec;

        if (bounce > 3) {
            float p = std::fmax(throughput.x, std::fmax(throughput.y, throughput.z));
            std::uniform_real_distribution<float> dist01(0.0f, 1.0f);
            if (dist01(rng) > p) break;
            throughput = throughput / p;
        }
    }

    return accumulated_color;
}

// ============================================================================
// 3b. METROPOLIS LIGHT TRANSPORT (MLT) - David Cline & Parris Egbert (2005)
// ============================================================================

constexpr size_t MLT_DIM = 128;

struct PrimarySampler {
    const std::vector<float>& u;
    mutable size_t ptr = 0;
    std::mt19937* fallback_rng = nullptr;

    float get_1d() const {
        if (ptr < u.size()) return u[ptr++];
        if (fallback_rng) {
            std::uniform_real_distribution<float> d(0.0f, 1.0f);
            return d(*fallback_rng);
        }
        return 0.5f;
    }

    Vec3 get_unit_vector() const {
        float z = 1.0f - 2.0f * get_1d();
        float r = std::sqrt(std::max(0.0f, 1.0f - z * z));
        float phi = 2.0f * 3.14159265f * get_1d();
        return Vec3(r * std::cos(phi), r * std::sin(phi), z);
    }
};

inline float color_luminance(const Vec3& c) {
    // Sekcja 2.2 z artykułu Cline & Egbert (2005): Luminance = 0.299 R + 0.587 G + 0.114 B
    return 0.299f * c.x + 0.587f * c.y + 0.114f * c.z;
}

Vec3 evaluate_path_mlt(const Ray& initial_ray, const HittableList& world,
                       const std::vector<Sphere>& light_sources, const PhotonMap& photon_map,
                       int max_depth, const PrimarySampler& sampler) {
    Vec3 accumulated_color(0.0f, 0.0f, 0.0f);
    Vec3 throughput(1.0f, 1.0f, 1.0f);
    Ray current_ray = initial_ray;
    bool last_was_specular = true;

    for (int bounce = 0; bounce < max_depth; ++bounce) {
        HitRecord rec;
        if (!world.hit(current_ray, 0.001f, 1e9f, rec)) {
            Vec3 unit_direction = unit_vector(current_ray.direction);
            float a = 0.5f * (unit_direction.y + 1.0f);
            Vec3 sky = (1.0f - a) * Vec3(0.04f, 0.05f, 0.08f) + a * Vec3(0.12f, 0.16f, 0.25f);
            accumulated_color = accumulated_color + throughput * sky;
            break;
        }

        Vec3 emitted = rec.mat->emitted(rec.p);
        if (!emitted.near_zero()) {
            if (last_was_specular) {
                accumulated_color = accumulated_color + throughput * emitted;
            }
            break;
        }

        if (rec.mat->is_dielectric()) {
            auto diel = static_cast<const Dielectric*>(rec.mat.get());
            float ref_ratio = rec.front_face ? (1.0f / diel->ir) : diel->ir;

            Vec3 unit_dir = unit_vector(current_ray.direction);
            float cos_theta = std::fmin(dot(-unit_dir, rec.normal), 1.0f);
            float sin_theta = std::sqrt(std::max(0.0f, 1.0f - cos_theta * cos_theta));

            bool cannot_refract = ref_ratio * sin_theta > 1.0f;
            float refl_prob = schlick_reflectance(cos_theta, ref_ratio);

            Vec3 next_dir;
            if (cannot_refract || refl_prob > sampler.get_1d()) {
                next_dir = reflect(unit_dir, rec.normal);
            } else {
                next_dir = refract(unit_dir, rec.normal, ref_ratio);
            }

            current_ray = Ray(rec.p, unit_vector(next_dir));
            last_was_specular = true;
        }
        else if (rec.mat->is_specular()) {
            auto metal = static_cast<const Metal*>(rec.mat.get());
            Vec3 unit_dir = unit_vector(current_ray.direction);
            Vec3 reflected = reflect(unit_dir, rec.normal);
            Vec3 scatter_dir = reflected + metal->roughness * sampler.get_unit_vector();
            if (dot(scatter_dir, rec.normal) <= 0.0f) break;

            current_ray = Ray(rec.p, unit_vector(scatter_dir));
            throughput = throughput * metal->albedo;
            last_was_specular = (metal->roughness < 0.15f);
        }
        else {
            Vec3 albedo = rec.mat->get_albedo();

            // Direct light connection
            if (!light_sources.empty()) {
                const auto& light = light_sources[0];
                Vec3 light_pt = light.center + light.radius * sampler.get_unit_vector();
                Vec3 to_light = light_pt - rec.p;
                float dist = to_light.length();
                Vec3 dir_light = to_light / dist;

                float cos_theta = dot(rec.normal, dir_light);
                if (cos_theta > 0.0f) {
                    HitRecord shadow_rec;
                    if (!world.hit(Ray(rec.p + rec.normal * 0.002f, dir_light), 0.001f, dist - 0.005f, shadow_rec)) {
                        Vec3 light_norm = unit_vector(light_pt - light.center);
                        float cos_light = std::fmax(0.0f, dot(light_norm, -dir_light));
                        float area = 4.0f * 3.14159f * light.radius * light.radius;
                        float G = (cos_theta * cos_light) / (dist * dist + 0.05f);

                        Vec3 direct = light.mat->emitted(light_pt) * albedo * (G * area * 0.159f);
                        direct.x = std::fmin(direct.x, 25.0f);
                        direct.y = std::fmin(direct.y, 25.0f);
                        direct.z = std::fmin(direct.z, 25.0f);
                        accumulated_color = accumulated_color + throughput * direct;
                    }
                }
            }

            // Global Photon Map caustic estimation (Sekcja 4 i 5: Bidirectional photon-assisted MLT)
            if (!photon_map.photons.empty()) {
                Vec3 caustic_flux = photon_map.estimate_radiance(rec.p, rec.normal, 0.12f);
                accumulated_color = accumulated_color + throughput * albedo * caustic_flux;
            }

            // Diffuse outgoing scatter
            Vec3 rand_v = sampler.get_unit_vector();
            Vec3 scatter_dir = rec.normal + rand_v;
            if (scatter_dir.near_zero()) scatter_dir = rec.normal;

            current_ray = Ray(rec.p, unit_vector(scatter_dir));
            throughput = throughput * albedo;
            last_was_specular = false;
        }

        // Russian roulette dla głębszych odbić
        if (bounce > 3) {
            float p = std::fmax(throughput.x, std::fmax(throughput.y, throughput.z));
            p = std::clamp(p, 0.05f, 0.95f);
            if (sampler.get_1d() > p) break;
            throughput = throughput / p;
        }
    }

    return accumulated_color;
}

// ============================================================================
// 3c. MANIFOLD EXPLORATION (MEMLT / MEPT) - Wenzel Jakob & Steve Marschner (2012)
// "Manifold Exploration: A Markov Chain Monte Carlo Technique for Rendering Scenes
// with Difficult Specular Transport", ACM Trans. Graph. 31(4)
// ============================================================================

struct CrystalRefractResult {
    bool hit = false;
    Vec3 p_enter;
    Vec3 n_enter;
    Vec3 p_exit;
    Vec3 n_exit;
    Vec3 dir_exit;
    float path_length = 0.0f;
};

inline CrystalRefractResult TraceThroughCrystal(
    const Ray& r_in,
    const std::shared_ptr<ConvexPolyhedron>& crystal,
    float ior = 1.55f
) {
    CrystalRefractResult res;
    if (!crystal) return res;

    HitRecord hit_in;
    if (!crystal->hit(r_in, 0.001f, 1e9f, hit_in)) return res;

    res.hit = true;
    res.p_enter = hit_in.p;
    res.n_enter = hit_in.normal;

    Vec3 unit_d = unit_vector(r_in.direction);
    float cos_in = std::fmin(dot(-unit_d, hit_in.normal), 1.0f);
    float sin_in = std::sqrt(std::max(0.0f, 1.0f - cos_in * cos_in));

    float eta_in = 1.0f / ior;
    Vec3 d_int;
    if (eta_in * sin_in > 1.0f) {
        d_int = reflect(unit_d, hit_in.normal);
    } else {
        d_int = refract(unit_d, hit_in.normal, eta_in);
    }
    d_int = unit_vector(d_int);

    // Propagacja promienia wewnątrz kryształu do wyjścia
    Ray r_int(hit_in.p + d_int * 0.001f, d_int);
    HitRecord hit_out;
    if (!crystal->hit(r_int, 0.001f, 1e9f, hit_out)) {
        res.p_exit = hit_in.p + d_int * 0.1f;
        res.n_exit = hit_in.normal;
        res.dir_exit = d_int;
        return res;
    }

    res.p_exit = hit_out.p;
    res.n_exit = hit_out.normal;
    res.path_length = (hit_out.p - hit_in.p).length();

    Vec3 unit_int = unit_vector(d_int);
    float cos_out = std::fmin(dot(-unit_int, hit_out.normal), 1.0f);
    float sin_out = std::sqrt(std::max(0.0f, 1.0f - cos_out * cos_out));

    float eta_out = ior;
    Vec3 d_out;
    if (eta_out * sin_out > 1.0f) {
        d_out = reflect(unit_int, hit_out.normal);
    } else {
        d_out = refract(unit_int, hit_out.normal, eta_out);
    }
    res.dir_exit = unit_vector(d_out);

    return res;
}

// Algorytm WALKMANIFOLD (Jakob & Marschner 2012, Rysunek 4 & Sekcja 4):
// Iteracyjny predyktor-korektor (metoda Newtona) na rozmaitości zwierciadlanej C(x)=0.
inline bool WalkSpecularManifold(
    const Vec3& origin,
    Vec3& inout_dir,
    const std::shared_ptr<ConvexPolyhedron>& crystal,
    const Vec3& target_point,
    const Vec3& target_normal,
    Vec3& out_final_hit,
    float tolerance = 1e-3f,
    int max_iters = 15
) {
    if (!crystal) return false;

    Vec3 cur_dir = unit_vector(inout_dir);
    float beta = 1.0f;

    for (int iter = 0; iter < max_iters; ++iter) {
        Ray r_cur(origin, cur_dir);
        auto spec = TraceThroughCrystal(r_cur, crystal, 1.55f);
        if (!spec.hit) return false;

        float denom = dot(spec.dir_exit, target_normal);
        if (std::fabs(denom) < 1e-5f) return false;

        float t = dot(target_point - spec.p_exit, target_normal) / denom;
        if (t <= 0.001f) return false;

        Vec3 hit_plane = spec.p_exit + spec.dir_exit * t;
        Vec3 diff = target_point - hit_plane;
        float err = diff.length();

        if (err < tolerance) {
            inout_dir = cur_dir;
            out_final_hit = hit_plane;
            return true;
        }

        // Baza styczna na sferze kierunków początkowych
        Vec3 up = (std::fabs(cur_dir.y) > 0.9f) ? Vec3(1.0f, 0.0f, 0.0f) : Vec3(0.0f, 1.0f, 0.0f);
        Vec3 U = unit_vector(cross(cur_dir, up));
        Vec3 V = cross(cur_dir, U);

        // Krok różniczkowy promienia h (Sekcja 3.3 i 4: Jacobian rozmaitości)
        float h = 1e-4f;
        Ray r_u(origin, unit_vector(cur_dir + h * U));
        Ray r_v(origin, unit_vector(cur_dir + h * V));

        auto spec_u = TraceThroughCrystal(r_u, crystal, 1.55f);
        auto spec_v = TraceThroughCrystal(r_v, crystal, 1.55f);
        if (!spec_u.hit || !spec_v.hit) return false;

        float denom_u = dot(spec_u.dir_exit, target_normal);
        float denom_v = dot(spec_v.dir_exit, target_normal);
        if (std::fabs(denom_u) < 1e-5f || std::fabs(denom_v) < 1e-5f) return false;

        float tu = dot(target_point - spec_u.p_exit, target_normal) / denom_u;
        float tv = dot(target_point - spec_v.p_exit, target_normal) / denom_v;
        if (tu <= 0.001f || tv <= 0.001f) return false;

        Vec3 hit_u = spec_u.p_exit + spec_u.dir_exit * tu;
        Vec3 hit_v = spec_v.p_exit + spec_v.dir_exit * tv;

        Vec3 Ju = (hit_u - hit_plane) / h;
        Vec3 Jv = (hit_v - hit_plane) / h;

        // Rozwiązanie układu równań normalnych J * [du, dv]^T = diff
        float a = dot(Ju, Ju);
        float b = dot(Ju, Jv);
        float c = dot(Jv, Jv);
        float ru = dot(diff, Ju);
        float rv = dot(diff, Jv);

        float det = a * c - b * b;
        if (det < 1e-12f) return false;

        float du = (c * ru - b * rv) / det;
        float dv = (a * rv - b * ru) / det;

        // Krok predykcji w przestrzeni stycznej rozmaitości:
        Vec3 next_dir = unit_vector(cur_dir + beta * (du * U + dv * V));

        // Krok korekcji: rzutowanie z powrotem na rozmaitość przez raytracing
        Ray r_next(origin, next_dir);
        auto spec_next = TraceThroughCrystal(r_next, crystal, 1.55f);
        if (spec_next.hit) {
            float d_next = dot(spec_next.dir_exit, target_normal);
            if (std::fabs(d_next) > 1e-5f) {
                float t_next = dot(target_point - spec_next.p_exit, target_normal) / d_next;
                if (t_next > 0.001f) {
                    Vec3 hit_next = spec_next.p_exit + spec_next.dir_exit * t_next;
                    float next_err = (target_point - hit_next).length();
                    if (next_err < err) {
                        cur_dir = next_dir;
                        beta = std::min(1.0f, 2.0f * beta);
                        continue;
                    }
                }
            }
        }

        beta *= 0.5f;
    }

    return false;
}

// ============================================================================
// 4. STRUKTURY DANYCH I ZMIENNE GLOBALNE
// ============================================================================

struct Tile {
    int x0, y0;
    int x1, y1;
};

struct EditableSceneObject {
    std::wstring name;
    Vec3 center;
    float radius = 1.0f;
    float secondary_param = 0.0f;
    bool is_crystal = false;
    bool is_mesh = false;
    bool is_quad = false;

    // For Quad
    Vec3 quad_u = Vec3(1, 0, 0);
    Vec3 quad_v = Vec3(0, 1, 0);

    // For Mesh
    std::shared_ptr<std::vector<Triangle>> mesh_triangles;
    Vec3 mesh_bmin = Vec3(-1, -1, -1);
    Vec3 mesh_bmax = Vec3(1, 1, 1);

    // Material definition & properties
    std::shared_ptr<Material> material;
    int mat_type = 0; // 0=Lambertian, 1=Metal, 2=Dielectric, 3=DiffuseLight
    Vec3 mat_color = Vec3(0.8f, 0.8f, 0.8f);
    float mat_param = 0.05f; // Roughness or IOR or Light intensity
};

inline void SetObjectMaterial(EditableSceneObject& obj, int type, const Vec3& col, float param) {
    obj.mat_type = type;
    obj.mat_color = col;
    obj.mat_param = param;
    if (type == 0) {
        obj.material = std::make_shared<Lambertian>(col);
    } else if (type == 1) {
        obj.material = std::make_shared<Metal>(col, std::clamp(param, 0.001f, 1.0f));
    } else if (type == 2) {
        float ior = (param > 0.1f) ? param : 1.55f;
        obj.material = std::make_shared<Dielectric>(ior);
    } else if (type == 3) {
        float intensity = (param > 0.01f) ? param : 1.0f;
        obj.material = std::make_shared<DiffuseLight>(col * intensity);
    }
}

inline EditableSceneObject make_sphere(const std::wstring& name, const Vec3& center, float r, int mat_type, const Vec3& col, float param = 0.05f) {
    EditableSceneObject obj;
    obj.name = name;
    obj.center = center;
    obj.radius = r;
    SetObjectMaterial(obj, mat_type, col, param);
    return obj;
}

inline EditableSceneObject make_crystal(const std::wstring& name, const Vec3& center, float r, float h, float ior = 1.55f) {
    EditableSceneObject obj;
    obj.name = name;
    obj.center = center;
    obj.radius = r;
    obj.secondary_param = h;
    obj.is_crystal = true;
    SetObjectMaterial(obj, 2, Vec3(1.0f, 1.0f, 1.0f), ior);
    return obj;
}

inline EditableSceneObject make_quad(const std::wstring& name, const Vec3& corner, const Vec3& u, const Vec3& v, int mat_type, const Vec3& col, float param = 0.05f) {
    EditableSceneObject obj;
    obj.name = name;
    obj.center = corner;
    obj.radius = 1.0f;
    obj.is_quad = true;
    obj.quad_u = u;
    obj.quad_v = v;
    SetObjectMaterial(obj, mat_type, col, param);
    return obj;
}

inline std::shared_ptr<Hittable> CreateHittableFromObject(const EditableSceneObject& obj) {
    if (obj.is_mesh && obj.mesh_triangles) {
        return std::make_shared<TriangleMesh>(obj.center, obj.radius, *obj.mesh_triangles, obj.mesh_bmin, obj.mesh_bmax, obj.material);
    } else if (obj.is_quad) {
        return std::make_shared<Quad>(obj.center, obj.quad_u * obj.radius, obj.quad_v * obj.radius, obj.material);
    } else if (obj.is_crystal) {
        float h = obj.secondary_param > 0.1f ? obj.secondary_param : 1.40f;
        return CreateFacetedCrystal(obj.center, obj.radius, h, obj.material);
    } else {
        return std::make_shared<Sphere>(obj.center, obj.radius, obj.material);
    }
}

int g_canvas_view_width  = 800;
int g_canvas_view_height = 450;
constexpr int TIMER_UPDATE_ID = 101;

constexpr int IDC_BTN_RENDER     = 1001;
constexpr int IDC_BTN_STOP       = 1002;
constexpr int IDC_BTN_SAVE_PNG   = 1003;
constexpr int IDC_COMBO_OBJECTS  = 1004;
constexpr int IDC_EDIT_SPP       = 1005;
constexpr int IDC_EDIT_POS_X     = 1006;
constexpr int IDC_EDIT_POS_Y     = 1007;
constexpr int IDC_EDIT_POS_Z     = 1008;
constexpr int IDC_EDIT_RADIUS    = 1009;
constexpr int IDC_BTN_APPLY_POS  = 1010;
constexpr int IDC_PROGRESS_BAR   = 1011;
constexpr int IDC_STATUS_LABEL   = 1012;
constexpr int IDC_COMBO_RES      = 1013;
constexpr int IDC_EDIT_RES_W     = 1014;
constexpr int IDC_EDIT_RES_H     = 1015;
constexpr int IDC_COMBO_TILE     = 1016;
constexpr int IDC_EDIT_DEPTH     = 1017;
constexpr int IDC_COMBO_ALGO     = 1018;
constexpr int IDC_EDIT_PHOTONS   = 1019;
constexpr int IDC_EDIT_FILENAME  = 1020;
constexpr int IDC_BTN_GRAB       = 1021;
constexpr int IDC_BTN_RESET_CAM  = 1022;
constexpr int IDC_COMBO_SCENE    = 1023;
constexpr int IDC_BTN_IMPORT_OBJ = 1024;
constexpr int IDC_COMBO_MAT_TYPE = 1025;
constexpr int IDC_EDIT_MAT_R     = 1026;
constexpr int IDC_EDIT_MAT_G     = 1027;
constexpr int IDC_EDIT_MAT_B     = 1028;
constexpr int IDC_EDIT_MAT_PARAM = 1029;
constexpr int IDC_BTN_APPLY_MAT  = 1030;

int g_current_scene  = 0;
int g_render_width   = 800;
int g_render_height  = 450;
int g_tile_size      = 32;
int g_render_algo    = 1;
int g_photon_count   = 50000;

// Dokładne śledzenie wymiarów bufora w pamięci zapobiegające rozjeżdżaniu skanlinii DIB!
std::atomic<int> g_cur_buf_w(800);
std::atomic<int> g_cur_buf_h(450);

std::vector<uint32_t> g_display_buffer(800 * 450, 0xFF18181B);
std::vector<Vec3>     g_raw_image(800 * 450, Vec3(0, 0, 0));
std::vector<int>      g_sample_counts(800 * 450, 0);

// Struktury dla Metropolis Light Transport (MLT):
std::atomic<size_t> g_mlt_total_mutations(0);
std::atomic<size_t> g_mlt_mutations_done(0);
std::vector<Vec3>   g_mlt_accum;
float               g_mlt_fave = 1.0f;
std::mutex          g_mlt_mutex;

std::atomic<bool>   g_is_rendering(false);
std::atomic<bool>   g_cancel_render(false);
std::atomic<size_t> g_completed_tiles(0);
size_t              g_total_tiles = 0;
std::chrono::high_resolution_clock::time_point g_render_start_time;

std::vector<EditableSceneObject> g_scene_objects = {
    make_crystal(L"1. Krysztal Szklany (Faceted Crystal)", Vec3(-0.5f, 0.70f, 0.1f), 0.65f, 1.40f, 1.55f),
    make_sphere(L"2. Zrodlo Swiatla (Kat boczno-tylny)",   Vec3(3.2f, 2.4f, -1.2f),   0.85f, 3, Vec3(35.0f, 32.0f, 28.0f), 1.0f),
    make_sphere(L"3. Kula Zlota (Prawa)",                 Vec3(1.7f, 0.80f, 0.6f),   0.80f, 1, Vec3(0.95f, 0.78f, 0.35f), 0.02f),
    make_sphere(L"4. Kula Rubinowa (Lewa-Tyl)",           Vec3(-2.0f, 0.65f, -0.6f), 0.65f, 0, Vec3(0.85f, 0.15f, 0.15f), 0.0f),
    make_sphere(L"5. Kula Chromowana (Przod)",            Vec3(0.7f, 0.35f, 1.7f),   0.35f, 1, Vec3(0.92f, 0.95f, 0.98f), 0.05f)
};

// ============================================================================
// 5. INTERAKTYWNA NAWIGACJA 3D (BLENDER ORBIT, PAN, ZOOM & TRANSFORMACJE)
// ============================================================================

enum AxisLock {
    AXIS_NONE = 0,
    AXIS_X,
    AXIS_Y,
    AXIS_Z
};

Vec3  g_cam_target(-0.3f, 0.5f, 0.0f);
float g_cam_distance = 4.6f;
float g_cam_yaw      = 0.05f;
float g_cam_pitch    = 0.45f;
float g_cam_fov      = 38.0f;

int      g_selected_object_idx = 0;
bool     g_is_grabbing         = false;
bool     g_is_scaling          = false;
bool     g_is_orbiting         = false;
bool     g_is_panning          = false;
bool     g_show_floor_grid     = true;

Vec3     g_grab_initial_pos;
float    g_scale_initial_r     = 1.0f;
POINT    g_grab_start_mouse    = {0, 0};
POINT    g_nav_last_mouse      = {0, 0};
Vec3     g_grab_plane_hit0;
Vec3     g_grab_plane_normal;
Vec3     g_grab_plane_point;
AxisLock g_axis_lock           = AXIS_NONE;

HWND g_hWnd          = nullptr;
HWND g_hCanvas       = nullptr;

HWND g_hGroupScene   = nullptr;
HWND g_hLblScene     = nullptr;
HWND g_hComboScene   = nullptr;
HWND g_hBtnImportObj = nullptr;

HWND g_hGroupRender  = nullptr;
HWND g_hLblAlgo      = nullptr;
HWND g_hComboAlgo    = nullptr;
HWND g_hLblPhotons   = nullptr;
HWND g_hEditPhotons  = nullptr;
HWND g_hLblPhotHint  = nullptr;
HWND g_hLblRes       = nullptr;
HWND g_hComboRes     = nullptr;
HWND g_hLblW         = nullptr;
HWND g_hEditResW     = nullptr;
HWND g_hLblH         = nullptr;
HWND g_hEditResH     = nullptr;
HWND g_hLblTile      = nullptr;
HWND g_hComboTile    = nullptr;
HWND g_hLblSPP       = nullptr;
HWND g_hEditSPP      = nullptr;
HWND g_hLblDepth     = nullptr;
HWND g_hEditDepth    = nullptr;
HWND g_hBtnRender    = nullptr;
HWND g_hBtnStop      = nullptr;

HWND g_hGroupObj     = nullptr;
HWND g_hLblObj       = nullptr;
HWND g_hComboObjects = nullptr;
HWND g_hLblPosX      = nullptr;
HWND g_hEditPosX     = nullptr;
HWND g_hLblPosY      = nullptr;
HWND g_hEditPosY     = nullptr;
HWND g_hLblPosZ      = nullptr;
HWND g_hEditPosZ     = nullptr;
HWND g_hLblRadius    = nullptr;
HWND g_hEditRadius   = nullptr;
HWND g_hBtnApplyPos  = nullptr;
HWND g_hBtnGrab      = nullptr;
HWND g_hBtnResetCam  = nullptr;

HWND g_hLblMatType   = nullptr;
HWND g_hComboMatType = nullptr;
HWND g_hLblMatCol    = nullptr;
HWND g_hEditMatR     = nullptr;
HWND g_hEditMatG     = nullptr;
HWND g_hEditMatB     = nullptr;
HWND g_hLblMatParam  = nullptr;
HWND g_hEditMatParam = nullptr;
HWND g_hBtnApplyMat  = nullptr;

HWND g_hGroupSave    = nullptr;
HWND g_hLblFile      = nullptr;
HWND g_hEditFilename = nullptr;
HWND g_hBtnSave      = nullptr;
HWND g_hProgressBar  = nullptr;
HWND g_hStatusLabel  = nullptr;

Camera GetCameraForRender(int width, int height) {
    float aspect_ratio = static_cast<float>(width) / static_cast<float>(std::max(1, height));

    float cy = std::cos(g_cam_yaw);
    float sy = std::sin(g_cam_yaw);
    float cp = std::cos(g_cam_pitch);
    float sp = std::sin(g_cam_pitch);

    Vec3 lookfrom(
        g_cam_target.x + g_cam_distance * cp * sy,
        g_cam_target.y + g_cam_distance * sp,
        g_cam_target.z + g_cam_distance * cp * cy
    );

    Vec3 vup(0.0f, 1.0f, 0.0f);
    return Camera(lookfrom, g_cam_target, vup, g_cam_fov, aspect_ratio);
}

Camera GetCurrentCamera() {
    return GetCameraForRender(g_canvas_view_width, g_canvas_view_height);
}

bool ProjectToScreen(const Vec3& p, const Camera& cam, float viewport_w, float viewport_h,
                     int screen_w, int screen_h, int& out_x, int& out_y) {
    Vec3 v = p - cam.origin;
    Vec3 w_dir = unit_vector(cam.lookat - cam.origin);
    float dz = dot(v, w_dir);
    if (dz <= 0.05f) return false;

    Vec3 u_dir = unit_vector(cam.horizontal);
    Vec3 v_dir = unit_vector(cam.vertical);

    float px = dot(v, u_dir) / dz;
    float py = dot(v, v_dir) / dz;

    float u = 0.5f + px / viewport_w;
    float v_norm = 0.5f + py / viewport_h;

    out_x = static_cast<int>(u * static_cast<float>(screen_w));
    out_y = static_cast<int>((1.0f - v_norm) * static_cast<float>(screen_h));
    return (out_x >= -300 && out_x <= screen_w + 300 && out_y >= -300 && out_y <= screen_h + 300);
}

int RayPickObject(int mx, int my) {
    if (mx < 0 || mx >= g_canvas_view_width || my < 0 || my >= g_canvas_view_height) return -1;
    Camera cam = GetCurrentCamera();

    float u = (static_cast<float>(mx) + 0.5f) / static_cast<float>(g_canvas_view_width);
    float v = (static_cast<float>(g_canvas_view_height - 1 - my) + 0.5f) / static_cast<float>(g_canvas_view_height);
    Ray r = cam.get_ray(u, v);

    int best_idx = -1;
    float closest_t = 1e9f;

    for (int i = 0; i < static_cast<int>(g_scene_objects.size()); ++i) {
        auto h = CreateHittableFromObject(g_scene_objects[i]);
        HitRecord rec;
        if (h->hit(r, 0.001f, closest_t, rec)) {
            closest_t = rec.t;
            best_idx = i;
        }
    }
    return best_idx;
}

void RenderFastPreview() {
    if (g_is_rendering.load()) return;

    Camera cam = GetCurrentCamera();
    int pw = std::max(100, g_canvas_view_width / 2);
    int ph = std::max(60, g_canvas_view_height / 2);

    std::vector<uint32_t> small_buf(pw * ph);

    HittableList world;
    for (int i = 0; i < static_cast<int>(g_scene_objects.size()); ++i) {
        world.add(CreateHittableFromObject(g_scene_objects[i]));
    }
    if (g_current_scene == 0) {
        world.add(std::make_shared<Sphere>(Vec3(0.0f, -1000.0f, 0.0f), 1000.0f,
                                           std::make_shared<Lambertian>(Vec3(0.85f, 0.85f, 0.88f))));
    }

    Vec3 light_pos(3.2f, 2.4f, -1.2f);
    for (const auto& obj : g_scene_objects) {
        if (obj.material && !obj.material->emitted(obj.center).near_zero()) {
            light_pos = obj.center;
            break;
        }
    }

    for (int py = 0; py < ph; ++py) {
        float v = (static_cast<float>(ph - 1 - py) + 0.5f) / static_cast<float>(ph);
        int row_offset = py * pw;

        for (int px = 0; px < pw; ++px) {
            float u = (static_cast<float>(px) + 0.5f) / static_cast<float>(pw);
            Ray r = cam.get_ray(u, v);

            HitRecord rec;
            Vec3 col;
            if (world.hit(r, 0.001f, 1e9f, rec)) {
                Vec3 to_light = light_pos - rec.p;
                float dist_l = to_light.length();
                Vec3 dir_l = to_light / (dist_l > 1e-4f ? dist_l : 1.0f);
                float diff = std::fmax(0.15f, dot(rec.normal, dir_l));

                if (!rec.mat->emitted(rec.p).near_zero()) {
                    col = Vec3(1.0f, 0.95f, 0.85f);
                } else if (rec.mat->is_dielectric()) {
                    float rim = std::pow(1.0f - std::fabs(dot(rec.normal, -unit_vector(r.direction))), 2.5f);
                    col = Vec3(0.12f, 0.18f, 0.28f) + Vec3(0.85f, 0.95f, 1.0f) * (rim * 0.9f);
                } else if (rec.mat->is_specular()) {
                    Vec3 refl = reflect(unit_vector(r.direction), rec.normal);
                    float spec = std::pow(std::fmax(0.0f, dot(refl, dir_l)), 16.0f);
                    col = rec.mat->get_albedo() * diff + Vec3(1.0f, 1.0f, 1.0f) * (spec * 0.6f);
                } else {
                    col = rec.mat->get_albedo() * diff;

                    if (g_show_floor_grid && std::fabs(rec.p.y) < 0.05f) {
                        float fx = std::fabs(rec.p.x - std::round(rec.p.x));
                        float fz = std::fabs(rec.p.z - std::round(rec.p.z));
                        if (fx < 0.035f || fz < 0.035f) {
                            col = col * 0.82f + Vec3(0.40f, 0.45f, 0.55f) * 0.35f;
                        }
                        if (std::fabs(rec.p.z) < 0.045f && std::fabs(rec.p.x) < 8.0f) {
                            col = Vec3(0.85f, 0.25f, 0.25f);
                        } else if (std::fabs(rec.p.x) < 0.045f && std::fabs(rec.p.z) < 8.0f) {
                            col = Vec3(0.25f, 0.45f, 0.90f);
                        }
                    }
                }
            } else {
                Vec3 unit_dir = unit_vector(r.direction);
                float a = 0.5f * (unit_dir.y + 1.0f);
                col = (1.0f - a) * Vec3(0.06f, 0.08f, 0.12f) + a * Vec3(0.18f, 0.22f, 0.32f);
            }

            float cr = std::clamp(std::sqrt(col.x), 0.0f, 0.999f);
            float cg = std::clamp(std::sqrt(col.y), 0.0f, 0.999f);
            float cb = std::clamp(std::sqrt(col.z), 0.0f, 0.999f);

            uint8_t ir = static_cast<uint8_t>(256.0f * cr);
            uint8_t ig = static_cast<uint8_t>(256.0f * cg);
            uint8_t ib = static_cast<uint8_t>(256.0f * cb);

            small_buf[row_offset + px] = (ib) | (ig << 8) | (ir << 16) | (0xFF << 24);
        }
    }

    g_display_buffer.assign(g_canvas_view_width * g_canvas_view_height, 0xFF18181B);
    g_cur_buf_w.store(g_canvas_view_width);
    g_cur_buf_h.store(g_canvas_view_height);

    for (int y = 0; y < g_canvas_view_height; ++y) {
        int sy = y / 2;
        if (sy >= ph) sy = ph - 1;
        int row_dst = y * g_canvas_view_width;
        int row_src = sy * pw;
        for (int x = 0; x < g_canvas_view_width; ++x) {
            int sx = x / 2;
            if (sx >= pw) sx = pw - 1;
            g_display_buffer[row_dst + x] = small_buf[row_src + sx];
        }
    }
}

void UpdateObjectFieldsFromSelection(int index) {
    if (index < 0 || index >= static_cast<int>(g_scene_objects.size())) return;
    const auto& obj = g_scene_objects[index];

    wchar_t buf[64];
    swprintf(buf, 64, L"%.2f", obj.center.x);
    SetWindowTextW(g_hEditPosX, buf);

    swprintf(buf, 64, L"%.2f", obj.center.y);
    SetWindowTextW(g_hEditPosY, buf);

    swprintf(buf, 64, L"%.2f", obj.center.z);
    SetWindowTextW(g_hEditPosZ, buf);

    swprintf(buf, 64, L"%.2f", obj.radius);
    SetWindowTextW(g_hEditRadius, buf);

    if (g_hComboMatType) {
        SendMessage(g_hComboMatType, CB_SETCURSEL, obj.mat_type, 0);
    }
    if (g_hEditMatR) {
        swprintf(buf, 64, L"%.2f", obj.mat_color.x);
        SetWindowTextW(g_hEditMatR, buf);
    }
    if (g_hEditMatG) {
        swprintf(buf, 64, L"%.2f", obj.mat_color.y);
        SetWindowTextW(g_hEditMatG, buf);
    }
    if (g_hEditMatB) {
        swprintf(buf, 64, L"%.2f", obj.mat_color.z);
        SetWindowTextW(g_hEditMatB, buf);
    }
    if (g_hEditMatParam) {
        swprintf(buf, 64, L"%.2f", obj.mat_param);
        SetWindowTextW(g_hEditMatParam, buf);
    }
}

void ApplyMaterialFromFields() {
    int sel = static_cast<int>(SendMessage(g_hComboObjects, CB_GETCURSEL, 0, 0));
    if (sel < 0 || sel >= static_cast<int>(g_scene_objects.size())) return;

    int mat_type = static_cast<int>(SendMessage(g_hComboMatType, CB_GETCURSEL, 0, 0));
    wchar_t bufR[32], bufG[32], bufB[32], bufP[32];
    GetWindowTextW(g_hEditMatR, bufR, 32);
    GetWindowTextW(g_hEditMatG, bufG, 32);
    GetWindowTextW(g_hEditMatB, bufB, 32);
    GetWindowTextW(g_hEditMatParam, bufP, 32);

    float r = wcstof(bufR, nullptr);
    float g = wcstof(bufG, nullptr);
    float b = wcstof(bufB, nullptr);
    float param = wcstof(bufP, nullptr);

    SetObjectMaterial(g_scene_objects[sel], mat_type, Vec3(r, g, b), param);

    const wchar_t* typeNames[] = { L"Lambertian (Matowy)", L"Metal (Zwierciadlany)", L"Dielectric (Szklo)", L"DiffuseLight (Swiatlo)" };
    wchar_t msg[160];
    swprintf(msg, 160, L"Zmieniono material [%s] na: %s (R:%.2f G:%.2f B:%.2f, Param:%.2f)",
             g_scene_objects[sel].name.c_str(), typeNames[std::clamp(mat_type, 0, 3)], r, g, b, param);
    SetWindowTextW(g_hStatusLabel, msg);

    RenderFastPreview();
    if (g_hCanvas) InvalidateRect(g_hCanvas, NULL, FALSE);
}

void ImportOBJModel() {
    if (g_is_rendering.load()) {
        MessageBoxW(g_hWnd, L"Renderowanie jest w toku! Przerwij renderowanie, aby zaimportowac model 3D.", L"Uwaga", MB_OK | MB_ICONWARNING);
        return;
    }

    wchar_t filename[MAX_PATH] = L"";
    OPENFILENAMEW ofn = {};
    ofn.lStructSize = sizeof(OPENFILENAMEW);
    ofn.hwndOwner = g_hWnd;
    ofn.lpstrFilter = L"Modele Wavefront OBJ (*.obj)\0*.obj\0Wszystkie pliki (*.*)\0*.*\0";
    ofn.lpstrFile = filename;
    ofn.nMaxFile = MAX_PATH;
    ofn.Flags = OFN_FILEMUSTEXIST | OFN_PATHMUSTEXIST;

    if (GetOpenFileNameW(&ofn)) {
        std::vector<Triangle> tris;
        Vec3 bmin, bmax;
        if (!LoadOBJGeometry(filename, tris, bmin, bmax)) {
            MessageBoxW(g_hWnd, L"Nie udalo sie wczytac geometrii z pliku .OBJ lub plik nie zawiera trojkatow.", L"Blad importu OBJ", MB_OK | MB_ICONERROR);
            return;
        }

        std::wstring pathStr(filename);
        size_t lastSlash = pathStr.find_last_of(L"\\/");
        std::wstring baseName = (lastSlash != std::wstring::npos) ? pathStr.substr(lastSlash + 1) : pathStr;

        EditableSceneObject mesh_obj;
        mesh_obj.name = L"Model: " + baseName;
        mesh_obj.center = (g_current_scene == 0) ? Vec3(0.0f, 0.7f, 0.0f) : Vec3(0.0f, 0.5f, 0.2f);
        mesh_obj.radius = 0.8f;
        mesh_obj.is_mesh = true;
        mesh_obj.mesh_triangles = std::make_shared<std::vector<Triangle>>(std::move(tris));
        mesh_obj.mesh_bmin = bmin;
        mesh_obj.mesh_bmax = bmax;
        SetObjectMaterial(mesh_obj, 0, Vec3(0.85f, 0.75f, 0.65f), 0.05f);

        g_scene_objects.push_back(mesh_obj);
        int new_idx = static_cast<int>(g_scene_objects.size()) - 1;

        if (g_hComboObjects) {
            SendMessage(g_hComboObjects, CB_ADDSTRING, 0, (LPARAM)mesh_obj.name.c_str());
            SendMessage(g_hComboObjects, CB_SETCURSEL, new_idx, 0);
        }
        g_selected_object_idx = new_idx;
        UpdateObjectFieldsFromSelection(new_idx);

        wchar_t status[240];
        swprintf(status, 240, L"Zaimportowano pomyslnie [%s] (%zu trojkatow). Uzyj 'G', aby przesunac, 'S', aby skalowac, lub zmien material!",
                 baseName.c_str(), mesh_obj.mesh_triangles->size());
        SetWindowTextW(g_hStatusLabel, status);

        RenderFastPreview();
        if (g_hCanvas) InvalidateRect(g_hCanvas, NULL, FALSE);
    }
}

void SwitchScene(int scene_id) {
    g_current_scene = scene_id;

    if (scene_id == 0) {
        // Scena 1: Oryginalna scena z szlifowanym krysztalem i kulami
        g_scene_objects = {
            make_crystal(L"1. Krysztal Szklany (Faceted Crystal)", Vec3(-0.5f, 0.70f, 0.1f), 0.65f, 1.40f, 1.55f),
            make_sphere(L"2. Zrodlo Swiatla (Kat boczno-tylny)",   Vec3(3.2f, 2.4f, -1.2f),   0.85f, 3, Vec3(35.0f, 32.0f, 28.0f), 1.0f),
            make_sphere(L"3. Kula Zlota (Prawa)",                 Vec3(1.7f, 0.80f, 0.6f),   0.80f, 1, Vec3(0.95f, 0.78f, 0.35f), 0.02f),
            make_sphere(L"4. Kula Rubinowa (Lewa-Tyl)",           Vec3(-2.0f, 0.65f, -0.6f), 0.65f, 0, Vec3(0.85f, 0.15f, 0.15f), 0.0f),
            make_sphere(L"5. Kula Chromowana (Przod)",            Vec3(0.7f, 0.35f, 1.7f),   0.35f, 1, Vec3(0.92f, 0.95f, 0.98f), 0.05f)
        };
        g_cam_target   = Vec3(-0.3f, 0.5f, 0.0f);
        g_cam_distance = 4.6f;
        g_cam_yaw      = 0.05f;
        g_cam_pitch    = 0.45f;
        g_cam_fov      = 38.0f;
    } else {
        // Scena 2: Zamkniety pokoj z podluznym oknem, zewnetrznym zrodlem swiatla i zewnetrznym zwierciadlem
        g_scene_objects.clear();

        // 1. Krysztal szklany wewnatrz pokoju
        g_scene_objects.push_back(
            make_crystal(L"1. Krysztal w Pokoju (Refrakcja)", Vec3(0.0f, 0.55f, 0.2f), 0.50f, 1.10f, 1.55f)
        );

        // 2. Kula zlota wewnatrz pokoju
        g_scene_objects.push_back(
            make_sphere(L"2. Kula Zlota w Pokoju", Vec3(0.75f, 0.35f, 0.4f), 0.35f, 1, Vec3(0.95f, 0.78f, 0.35f), 0.03f)
        );

        // 3. Zewnetrzne zrodlo swiatla (za przeslona ochronna)
        g_scene_objects.push_back(
            make_sphere(L"3. Zewnetrzne Zrodlo Swiatla", Vec3(2.2f, 1.15f, 2.1f), 0.40f, 3, Vec3(70.0f, 65.0f, 55.0f), 1.0f)
        );

        // 4. Zewnetrzne lustro odbijajace swiatlo do wnetrza przez okno
        g_scene_objects.push_back(
            make_quad(L"4. Zewnetrzne Lustro (Reflektor)", Vec3(-2.0f, 0.0f, 3.2f), Vec3(4.0f, 0.0f, 0.0f), Vec3(0.0f, 2.4f, 0.0f), 1, Vec3(0.98f, 0.98f, 0.98f), 0.01f)
        );

        // 5. Zewnetrzna przeslona (Baffle uniemozliwiajacy oswietlenie bezposrednie)
        g_scene_objects.push_back(
            make_quad(L"5. Zewnetrzna Przeslona (Baffle)", Vec3(1.35f, 0.0f, 1.4f), Vec3(0.0f, 0.0f, 1.4f), Vec3(0.0f, 2.4f, 0.0f), 0, Vec3(0.2f, 0.2f, 0.25f))
        );

        // 6-9. Sciana z podluznym oknem (Dol, Gora, Lewo, Prawo)
        g_scene_objects.push_back(
            make_quad(L"6. Sciana z Oknem (Panel Dolny)", Vec3(-2.0f, 0.0f, 1.4f), Vec3(4.0f, 0.0f, 0.0f), Vec3(0.0f, 0.95f, 0.0f), 0, Vec3(0.72f, 0.70f, 0.68f))
        );
        g_scene_objects.push_back(
            make_quad(L"7. Sciana z Oknem (Panel Gorny)", Vec3(-2.0f, 1.35f, 1.4f), Vec3(4.0f, 0.0f, 0.0f), Vec3(0.0f, 1.05f, 0.0f), 0, Vec3(0.72f, 0.70f, 0.68f))
        );
        g_scene_objects.push_back(
            make_quad(L"8. Sciana z Oknem (Panel Lewy)", Vec3(-2.0f, 0.95f, 1.4f), Vec3(0.8f, 0.0f, 0.0f), Vec3(0.0f, 0.40f, 0.0f), 0, Vec3(0.72f, 0.70f, 0.68f))
        );
        g_scene_objects.push_back(
            make_quad(L"9. Sciana z Oknem (Panel Prawy)", Vec3(1.2f, 0.95f, 1.4f), Vec3(0.8f, 0.0f, 0.0f), Vec3(0.0f, 0.40f, 0.0f), 0, Vec3(0.72f, 0.70f, 0.68f))
        );

        // 10-14. Sciany zamknietego pokoju (Podloga, Sufit, Lewa, Prawa, Tylna za kamera)
        g_scene_objects.push_back(
            make_quad(L"10. Podloga Pokoju", Vec3(-2.0f, 0.0f, -2.0f), Vec3(4.0f, 0.0f, 0.0f), Vec3(0.0f, 0.0f, 3.4f), 0, Vec3(0.75f, 0.72f, 0.68f))
        );
        g_scene_objects.push_back(
            make_quad(L"11. Sufit Pokoju", Vec3(-2.0f, 2.4f, -2.0f), Vec3(4.0f, 0.0f, 0.0f), Vec3(0.0f, 0.0f, 3.4f), 0, Vec3(0.82f, 0.82f, 0.82f))
        );
        g_scene_objects.push_back(
            make_quad(L"12. Sciana Lewa", Vec3(-2.0f, 0.0f, -2.0f), Vec3(0.0f, 0.0f, 3.4f), Vec3(0.0f, 2.4f, 0.0f), 0, Vec3(0.75f, 0.35f, 0.32f))
        );
        g_scene_objects.push_back(
            make_quad(L"13. Sciana Prawa", Vec3(2.0f, 0.0f, 1.4f), Vec3(0.0f, 0.0f, -3.4f), Vec3(0.0f, 2.4f, 0.0f), 0, Vec3(0.32f, 0.65f, 0.42f))
        );
        g_scene_objects.push_back(
            make_quad(L"14. Sciana Tylna", Vec3(2.0f, 0.0f, -2.0f), Vec3(-4.0f, 0.0f, 0.0f), Vec3(0.0f, 2.4f, 0.0f), 0, Vec3(0.68f, 0.68f, 0.72f))
        );

        // Kamera wewnatrz zamknietego pokoju skierowana na okno
        g_cam_target   = Vec3(0.0f, 0.85f, 0.6f);
        g_cam_distance = 1.9f;
        g_cam_yaw      = 3.14159f;
        g_cam_pitch    = -0.05f;
        g_cam_fov      = 52.0f;
    }

    if (g_hComboObjects) {
        SendMessage(g_hComboObjects, CB_RESETCONTENT, 0, 0);
        for (const auto& obj : g_scene_objects) {
            SendMessage(g_hComboObjects, CB_ADDSTRING, 0, (LPARAM)obj.name.c_str());
        }
        SendMessage(g_hComboObjects, CB_SETCURSEL, 0, 0);
    }
    g_selected_object_idx = 0;
    UpdateObjectFieldsFromSelection(0);

    RenderFastPreview();
    if (g_hCanvas) InvalidateRect(g_hCanvas, NULL, FALSE);
}

void StartGrabMode() {
    if (g_selected_object_idx < 0 || g_selected_object_idx >= static_cast<int>(g_scene_objects.size())) {
        SetWindowTextW(g_hStatusLabel, L"Najpierw kliknij obiekt lewym przyciskiem myszy (LPM), aby go wybrac!");
        return;
    }
    if (g_is_rendering.load()) {
        SetWindowTextW(g_hStatusLabel, L"Renderowanie jest w toku! Kliknij 'Przerwij', aby edytowac pozycje.");
        return;
    }

    g_is_grabbing = true;
    g_is_scaling  = false;
    g_grab_initial_pos = g_scene_objects[g_selected_object_idx].center;
    g_axis_lock = AXIS_NONE;

    Camera cam = GetCurrentCamera();
    g_grab_plane_point = g_grab_initial_pos;
    g_grab_plane_normal = unit_vector(cam.origin - cam.lookat);

    POINT pt;
    GetCursorPos(&pt);
    ScreenToClient(g_hCanvas, &pt);
    g_grab_start_mouse = pt;

    float u0 = (static_cast<float>(pt.x) + 0.5f) / static_cast<float>(g_canvas_view_width);
    float v0 = (static_cast<float>(g_canvas_view_height - 1 - pt.y) + 0.5f) / static_cast<float>(g_canvas_view_height);
    Ray r0 = cam.get_ray(u0, v0);

    float denom0 = dot(g_grab_plane_normal, r0.direction);
    if (std::fabs(denom0) > 1e-6f) {
        float t0 = dot(g_grab_plane_point - r0.origin, g_grab_plane_normal) / denom0;
        g_grab_plane_hit0 = r0.at(t0);
    } else {
        g_grab_plane_hit0 = g_grab_initial_pos;
    }

    SetCapture(g_hCanvas);
    SetFocus(g_hCanvas);

    wchar_t status[200];
    swprintf(status, 200, L"[BLENDER GRAB] %s | Ruch mysza | [X/Y/Z]: Blokada osi | [LPM/Enter]: Zatwierdz | [PPM/Esc]: Anuluj",
             g_scene_objects[g_selected_object_idx].name.c_str());
    SetWindowTextW(g_hStatusLabel, status);

    InvalidateRect(g_hCanvas, NULL, FALSE);
}

void ConfirmGrabMode() {
    if (!g_is_grabbing) return;
    g_is_grabbing = false;
    ReleaseCapture();
    g_axis_lock = AXIS_NONE;

    UpdateObjectFieldsFromSelection(g_selected_object_idx);

    wchar_t status[200];
    const auto& obj = g_scene_objects[g_selected_object_idx];
    swprintf(status, 200, L"Zatwierdzono pozycje: [%s] (X: %.2f, Y: %.2f, Z: %.2f)",
             obj.name.c_str(), obj.center.x, obj.center.y, obj.center.z);
    SetWindowTextW(g_hStatusLabel, status);

    RenderFastPreview();
    InvalidateRect(g_hCanvas, NULL, FALSE);
}

void CancelGrabMode() {
    if (!g_is_grabbing) return;
    g_scene_objects[g_selected_object_idx].center = g_grab_initial_pos;
    g_is_grabbing = false;
    ReleaseCapture();
    g_axis_lock = AXIS_NONE;

    UpdateObjectFieldsFromSelection(g_selected_object_idx);
    SetWindowTextW(g_hStatusLabel, L"Anulowano przesuwanie (przywrocono pozycje wyjsciowa).");

    RenderFastPreview();
    InvalidateRect(g_hCanvas, NULL, FALSE);
}

void ProcessGrabMove(int mx, int my) {
    if (!g_is_grabbing) return;

    Camera cam = GetCurrentCamera();
    float u = (static_cast<float>(mx) + 0.5f) / static_cast<float>(g_canvas_view_width);
    float v = (static_cast<float>(g_canvas_view_height - 1 - my) + 0.5f) / static_cast<float>(g_canvas_view_height);
    Ray r = cam.get_ray(u, v);

    float denom = dot(g_grab_plane_normal, r.direction);
    if (std::fabs(denom) > 1e-6f) {
        float t = dot(g_grab_plane_point - r.origin, g_grab_plane_normal) / denom;
        Vec3 cur_hit = r.at(t);
        Vec3 delta = cur_hit - g_grab_plane_hit0;

        if (g_axis_lock == AXIS_X) {
            delta.y = 0.0f;
            delta.z = 0.0f;
        } else if (g_axis_lock == AXIS_Y) {
            delta.x = 0.0f;
            delta.z = 0.0f;
        } else if (g_axis_lock == AXIS_Z) {
            delta.x = 0.0f;
            delta.y = 0.0f;
        }

        Vec3 new_pos = g_grab_initial_pos + delta;
        g_scene_objects[g_selected_object_idx].center = new_pos;

        UpdateObjectFieldsFromSelection(g_selected_object_idx);
        RenderFastPreview();
        InvalidateRect(g_hCanvas, NULL, FALSE);
    }
}

void StartScaleMode() {
    if (g_selected_object_idx < 0 || g_selected_object_idx >= static_cast<int>(g_scene_objects.size())) return;
    if (g_is_rendering.load()) return;

    g_is_scaling  = true;
    g_is_grabbing = false;
    g_scale_initial_r = g_scene_objects[g_selected_object_idx].radius;

    POINT pt; GetCursorPos(&pt); ScreenToClient(g_hCanvas, &pt);
    g_grab_start_mouse = pt;

    SetCapture(g_hCanvas);
    SetFocus(g_hCanvas);

    wchar_t status[200];
    swprintf(status, 200, L"[BLENDER SCALE] %s | Oddal/przybliz kursor | [LPM/Enter]: Zatwierdz | [PPM/Esc]: Anuluj",
             g_scene_objects[g_selected_object_idx].name.c_str());
    SetWindowTextW(g_hStatusLabel, status);

    InvalidateRect(g_hCanvas, NULL, FALSE);
}

void ConfirmScaleMode() {
    if (!g_is_scaling) return;
    g_is_scaling = false;
    ReleaseCapture();

    UpdateObjectFieldsFromSelection(g_selected_object_idx);
    wchar_t status[200];
    swprintf(status, 200, L"Zatwierdzono nowy promien/skale: R=%.2f", g_scene_objects[g_selected_object_idx].radius);
    SetWindowTextW(g_hStatusLabel, status);

    RenderFastPreview();
    InvalidateRect(g_hCanvas, NULL, FALSE);
}

void CancelScaleMode() {
    if (!g_is_scaling) return;
    g_scene_objects[g_selected_object_idx].radius = g_scale_initial_r;
    g_is_scaling = false;
    ReleaseCapture();

    UpdateObjectFieldsFromSelection(g_selected_object_idx);
    SetWindowTextW(g_hStatusLabel, L"Anulowano skalowanie obiektu.");

    RenderFastPreview();
    InvalidateRect(g_hCanvas, NULL, FALSE);
}

void ProcessScaleMove(int mx, int my) {
    if (!g_is_scaling) return;

    Camera cam = GetCurrentCamera();
    float v_w = 2.0f * std::tan(g_cam_fov * 3.14159f / 360.0f) * (static_cast<float>(g_canvas_view_width) / g_canvas_view_height);
    float v_h = 2.0f * std::tan(g_cam_fov * 3.14159f / 360.0f);

    int sx = 0, sy = 0;
    if (ProjectToScreen(g_scene_objects[g_selected_object_idx].center, cam, v_w, v_h,
                        g_canvas_view_width, g_canvas_view_height, sx, sy)) {
        float d0 = std::hypot(static_cast<float>(g_grab_start_mouse.x - sx), static_cast<float>(g_grab_start_mouse.y - sy));
        float d1 = std::hypot(static_cast<float>(mx - sx), static_cast<float>(my - sy));
        d0 = std::max(20.0f, d0);
        float scale = d1 / d0;

        float new_r = std::clamp(g_scale_initial_r * scale, 0.10f, 6.0f);
        g_scene_objects[g_selected_object_idx].radius = new_r;

        UpdateObjectFieldsFromSelection(g_selected_object_idx);
        RenderFastPreview();
        InvalidateRect(g_hCanvas, NULL, FALSE);
    }
}

void ResetCamera() {
    if (g_current_scene == 0) {
        g_cam_target   = Vec3(-0.3f, 0.5f, 0.0f);
        g_cam_distance = 4.6f;
        g_cam_yaw      = 0.05f;
        g_cam_pitch    = 0.45f;
        g_cam_fov      = 38.0f;
    } else {
        g_cam_target   = Vec3(0.0f, 0.85f, 0.6f);
        g_cam_distance = 1.9f;
        g_cam_yaw      = 3.14159f;
        g_cam_pitch    = -0.05f;
        g_cam_fov      = 52.0f;
    }
    RenderFastPreview();
    InvalidateRect(g_hCanvas, NULL, FALSE);
    SetWindowTextW(g_hStatusLabel, L"Zresetowano widok kamery 3D do kata poczatkowego sceny.");
}

void ApplyPositionFromFields() {
    int sel = static_cast<int>(SendMessage(g_hComboObjects, CB_GETCURSEL, 0, 0));
    if (sel < 0 || sel >= static_cast<int>(g_scene_objects.size())) return;

    wchar_t buf[64];
    GetWindowTextW(g_hEditPosX, buf, 64);
    float x = wcstof(buf, nullptr);

    GetWindowTextW(g_hEditPosY, buf, 64);
    float y = wcstof(buf, nullptr);

    GetWindowTextW(g_hEditPosZ, buf, 64);
    float z = wcstof(buf, nullptr);

    GetWindowTextW(g_hEditRadius, buf, 64);
    float r = wcstof(buf, nullptr);
    if (r <= 0.05f) r = 0.05f;

    g_scene_objects[sel].center = Vec3(x, y, z);
    g_scene_objects[sel].radius = r;
    g_selected_object_idx = sel;

    wchar_t msg[128];
    swprintf(msg, 128, L"Zaktualizowano: [%s] (%.2f, %.2f, %.2f) R=%.2f",
             g_scene_objects[sel].name.c_str(), x, y, z, r);
    SetWindowTextW(g_hStatusLabel, msg);

    RenderFastPreview();
    InvalidateRect(g_hCanvas, NULL, FALSE);
}

void OnResolutionComboChanged() {
    int sel = static_cast<int>(SendMessage(g_hComboRes, CB_GETCURSEL, 0, 0));
    int w = 800, h = 450;
    switch (sel) {
        case 0: w = 640;  h = 360;  break;
        case 1: w = 800;  h = 450;  break;
        case 2: w = 960;  h = 540;  break;
        case 3: w = 1280; h = 720;  break;
        case 4: w = 1920; h = 1080; break;
        default: return;
    }

    wchar_t bufW[32], bufH[32];
    swprintf(bufW, 32, L"%d", w);
    swprintf(bufH, 32, L"%d", h);
    SetWindowTextW(g_hEditResW, bufW);
    SetWindowTextW(g_hEditResH, bufH);
}

// ============================================================================
// 6. STOCHASTYCZNE KAFELKOWANIE ORAZ RENDEROWANIE
// ============================================================================

void StartRendering() {
    if (g_is_rendering.load()) return;
    if (g_is_grabbing) ConfirmGrabMode();
    if (g_is_scaling)  ConfirmScaleMode();

    wchar_t bufW[32], bufH[32];
    GetWindowTextW(g_hEditResW, bufW, 32);
    GetWindowTextW(g_hEditResH, bufH, 32);
    int reqW = wcstol(bufW, nullptr, 10);
    int reqH = wcstol(bufH, nullptr, 10);
    if (reqW < 64)   reqW = 64;
    if (reqW > 3840) reqW = 3840;
    if (reqH < 64)   reqH = 64;
    if (reqH > 2160) reqH = 2160;
    g_render_width  = reqW;
    g_render_height = reqH;

    int tile_sel = static_cast<int>(SendMessage(g_hComboTile, CB_GETCURSEL, 0, 0));
    switch (tile_sel) {
        case 0:  g_tile_size = 16; break;
        case 1:  g_tile_size = 32; break;
        case 2:  g_tile_size = 64; break;
        default: g_tile_size = 32; break;
    }

    g_render_algo = static_cast<int>(SendMessage(g_hComboAlgo, CB_GETCURSEL, 0, 0));

    wchar_t spp_buf[32], depth_buf[32], phot_buf[32];
    GetWindowTextW(g_hEditSPP, spp_buf, 32);
    GetWindowTextW(g_hEditDepth, depth_buf, 32);
    GetWindowTextW(g_hEditPhotons, phot_buf, 32);

    int spp = wcstol(spp_buf, nullptr, 10);
    int max_depth = wcstol(depth_buf, nullptr, 10);
    int photons = wcstol(phot_buf, nullptr, 10);

    if (spp < 1) spp = 1;
    if (spp > 5000) spp = 5000;
    if (max_depth < 1) max_depth = 1;
    if (max_depth > 128) max_depth = 128;
    if (photons < 1000) photons = 1000;
    if (photons > 1000000) photons = 1000000;
    g_photon_count = photons;

    // Ustawienie dokładnych wymiarów bufora w pamięci dla renderera
    g_cur_buf_w.store(g_render_width);
    g_cur_buf_h.store(g_render_height);
    g_display_buffer.assign(g_render_width * g_render_height, 0xFF18181B);
    g_raw_image.assign(g_render_width * g_render_height, Vec3(0, 0, 0));
    g_sample_counts.assign(g_render_width * g_render_height, 0);

    std::vector<Tile> tiles;
    for (int y = 0; y < g_render_height; y += g_tile_size) {
        int y_end = std::min(y + g_tile_size, g_render_height);
        for (int x = 0; x < g_render_width; x += g_tile_size) {
            int x_end = std::min(x + g_tile_size, g_render_width);
            tiles.push_back({x, y, x_end, y_end});
        }
    }
    g_total_tiles = tiles.size();

    float center_x = g_render_width * 0.5f;
    float center_y = g_render_height * 0.5f;
    std::sort(tiles.begin(), tiles.end(), [center_x, center_y](const Tile& a, const Tile& b) {
        float ax = (a.x0 + a.x1) * 0.5f - center_x;
        float ay = (a.y0 + a.y1) * 0.5f - center_y;
        float bx = (b.x0 + b.x1) * 0.5f - center_x;
        float by = (b.y0 + b.y1) * 0.5f - center_y;
        return (ax * ax + ay * ay) < (bx * bx + by * by);
    });

    EnableWindow(g_hBtnRender, FALSE);
    EnableWindow(g_hBtnStop, TRUE);
    EnableWindow(g_hBtnApplyPos, FALSE);
    EnableWindow(g_hBtnGrab, FALSE);

    const wchar_t* algoName = L"Standardowy PT";
    if (g_render_algo == 1) algoName = L"BDPT + Globalna Mapa Fotonow";
    else if (g_render_algo == 2) algoName = L"Metropolis Light Transport (MLT)";
    else if (g_render_algo == 3) algoName = L"Manifold Exploration (MEMLT / MEPT)";

    wchar_t statusInit[160];
    if (g_render_algo == 2 || g_render_algo == 3) {
        size_t total_mutations = static_cast<size_t>(g_render_width) * g_render_height * spp;
        const wchar_t* tag = (g_render_algo == 3) ? L"MEMLT" : L"MLT";
        swprintf(statusInit, 160, L"Rozpoczynanie [%s]: %zu mutacji (%d/px, %d fotonow)...",
                 tag, total_mutations, spp, g_photon_count);
        SendMessage(g_hProgressBar, PBM_SETRANGE, 0, MAKELPARAM(0, 1000));
        SendMessage(g_hProgressBar, PBM_SETPOS, 0, 0);
    } else {
        swprintf(statusInit, 160, L"Rozpoczynanie [%s]: %zu kafelkow %dx%d px (%d fotonow)...",
                 algoName, g_total_tiles, g_tile_size, g_tile_size, g_photon_count);
        SendMessage(g_hProgressBar, PBM_SETRANGE, 0, MAKELPARAM(0, g_total_tiles));
        SendMessage(g_hProgressBar, PBM_SETPOS, 0, 0);
    }
    SetWindowTextW(g_hStatusLabel, statusInit);

    g_is_rendering.store(true);
    g_cancel_render.store(false);
    g_completed_tiles.store(0);
    g_render_start_time = std::chrono::high_resolution_clock::now();

    InvalidateRect(g_hCanvas, NULL, FALSE);

    int algo = g_render_algo;

    std::thread([tiles, spp, max_depth, algo, photons]() {
        HittableList world;
        std::vector<Sphere> light_sources;
        std::shared_ptr<ConvexPolyhedron> glass_crystal = nullptr;
        float crystal_radius = 0.65f;

        if (g_current_scene == 0) {
            world.add(std::make_shared<Sphere>(Vec3(0.0f, -1000.0f, 0.0f), 1000.0f,
                                               std::make_shared<Lambertian>(Vec3(0.85f, 0.85f, 0.88f))));
        }

        for (const auto& obj : g_scene_objects) {
            world.add(CreateHittableFromObject(obj));
            if (obj.is_crystal) {
                float h = obj.secondary_param > 0.1f ? obj.secondary_param : 1.40f;
                crystal_radius = obj.radius;
                glass_crystal = CreateFacetedCrystal(obj.center, obj.radius, h, obj.material);
            }
            if (obj.material && !obj.material->emitted(obj.center).near_zero()) {
                light_sources.push_back(Sphere(obj.center, obj.radius, obj.material));
            }
        }

        // Dokładna kamera dla wybranej rozdzielczości renderowania
        Camera cam = GetCameraForRender(g_render_width, g_render_height);

        PhotonMap photon_map;
        if ((algo == 1 || algo == 2 || algo == 3) && glass_crystal && !light_sources.empty()) {
            photon_map.init(0.06f, -6.0f, 6.0f, -6.0f, 6.0f);

            const auto& light = light_sources[0];
            Vec3 crystal_center = glass_crystal->center;
            float bound_r = crystal_radius * 1.35f;

            Vec3 light_to_cryst = crystal_center - light.center;
            float dist_lc = light_to_cryst.length();
            Vec3 dir_lc = light_to_cryst / dist_lc;

            Vec3 up(0.0f, 1.0f, 0.0f);
            if (std::fabs(dir_lc.y) > 0.9f) up = Vec3(1.0f, 0.0f, 0.0f);
            Vec3 right = unit_vector(cross(dir_lc, up));
            Vec3 v_up = cross(right, dir_lc);

            float solid_angle = (3.14159f * bound_r * bound_r) / (dist_lc * dist_lc + 0.1f);
            Vec3 emit_power = light.mat->emitted(light.center);
            Vec3 photon_flux_base = emit_power * (solid_angle * 4.0f * 3.14159f * light.radius * light.radius / static_cast<float>(photons));

            unsigned int num_threads = std::max(1u, std::thread::hardware_concurrency());
            int photons_per_thread = photons / num_threads;

            std::vector<std::vector<Photon>> thread_photons(num_threads);
            std::vector<std::thread> photon_workers;

            for (unsigned int th = 0; th < num_threads; ++th) {
                photon_workers.emplace_back([&, th]() {
                    std::mt19937 rng(54321 + th * 2039);
                    std::uniform_real_distribution<float> dist01(0.0f, 1.0f);
                    std::uniform_real_distribution<float> d_theta(0.0f, 6.2831853f);

                    auto& local_photons = thread_photons[th];
                    local_photons.reserve(photons_per_thread / 2);

                    for (int i = 0; i < photons_per_thread && !g_cancel_render.load(); ++i) {
                        Vec3 p_light = light.center + light.radius * random_unit_vector(rng);

                        float r_disk = bound_r * std::sqrt(dist01(rng));
                        float th_angle = d_theta(rng);
                        Vec3 target = crystal_center + right * (r_disk * std::cos(th_angle)) + v_up * (r_disk * std::sin(th_angle));
                        Vec3 dir = unit_vector(target - p_light);

                        Ray cur_ray(p_light, dir);
                        Vec3 cur_flux = photon_flux_base;

                        for (int bounce = 0; bounce < 8; ++bounce) {
                            HitRecord hit;
                            if (!glass_crystal->hit(cur_ray, 0.001f, 1e9f, hit)) break;

                            float ref_idx = hit.front_face ? (1.0f / 1.55f) : 1.55f;
                            Vec3 unit_dir = unit_vector(cur_ray.direction);
                            float cos_theta = std::fmin(dot(-unit_dir, hit.normal), 1.0f);
                            float sin_theta = std::sqrt(std::max(0.0f, 1.0f - cos_theta * cos_theta));

                            bool cannot_refract = ref_idx * sin_theta > 1.0f;
                            float refl_prob = schlick_reflectance(cos_theta, ref_idx);

                            Vec3 next_dir;
                            if (cannot_refract || dist01(rng) < refl_prob) {
                                next_dir = reflect(unit_dir, hit.normal);
                            } else {
                                next_dir = refract(unit_dir, hit.normal, ref_idx);
                            }

                            cur_ray = Ray(hit.p + next_dir * 0.0015f, unit_vector(next_dir));
                        }

                        HitRecord floor_hit;
                        if (world.hit(cur_ray, 0.001f, 1e9f, floor_hit)) {
                            if (!floor_hit.mat->is_specular() && floor_hit.mat->emitted(floor_hit.p).near_zero()) {
                                float cos_surf = std::fmax(0.0f, dot(floor_hit.normal, -cur_ray.direction));
                                if (cos_surf > 0.02f) {
                                    local_photons.push_back({floor_hit.p, cur_flux * (cos_surf * 0.35f)});
                                }
                            }
                        }
                    }
                });
            }

            for (auto& pw : photon_workers) {
                if (pw.joinable()) pw.join();
            }

            for (const auto& lph : thread_photons) {
                for (const auto& ph : lph) {
                    photon_map.add_photon(ph.pos, ph.flux);
                }
            }
        }

        if (algo == 2 || algo == 3) {
            // ================================================================
            // MCMC: MLT (Cline & Egbert 2005) / MEMLT (Jakob & Marschner 2012)
            // ================================================================
            size_t total_pixels = static_cast<size_t>(g_render_width) * g_render_height;
            {
                std::lock_guard<std::mutex> lock(g_mlt_mutex);
                g_mlt_accum.assign(total_pixels, Vec3(0, 0, 0));
            }
            size_t total_mutations = total_pixels * spp;
            g_mlt_total_mutations.store(total_mutations);
            g_mlt_mutations_done.store(0);

            unsigned int num_threads = std::max(1u, std::thread::hardware_concurrency());
            size_t bootstrap_count = 25000;
            size_t boot_per_thread = bootstrap_count / num_threads;

            struct SeedPath {
                std::vector<float> u;
                Vec3 col;
                float lum = 0.0f;
            };

            std::vector<std::vector<SeedPath>> thread_seeds(num_threads);
            std::vector<double> thread_lum_sum(num_threads, 0.0);
            std::vector<std::thread> boot_workers;

            // 1. Faza Bootstrap (Sekcja 3.5 z pracy): oszacowanie fave oraz znalezienie jasnych ścieżek
            for (unsigned int th = 0; th < num_threads; ++th) {
                boot_workers.emplace_back([&, th]() {
                    std::mt19937 rng(4321 + th * 7823);
                    std::uniform_real_distribution<float> dist01(0.0f, 1.0f);
                    double local_lum = 0.0;
                    auto& s_list = thread_seeds[th];

                    for (size_t b = 0; b < boot_per_thread && !g_cancel_render.load(); ++b) {
                        std::vector<float> u(MLT_DIM);
                        for (size_t k = 0; k < MLT_DIM; ++k) u[k] = dist01(rng);

                        PrimarySampler sampler{u, 2, &rng};
                        Ray r = cam.get_ray(u[0], 1.0f - u[1]);

                        Vec3 col = evaluate_path_mlt(r, world, light_sources, photon_map, max_depth, sampler);
                        float lum = color_luminance(col);
                        local_lum += lum;

                        if (lum > 1e-4f) {
                            s_list.push_back({u, col, lum});
                        }
                    }
                    thread_lum_sum[th] = local_lum;
                });
            }

            for (auto& bw : boot_workers) {
                if (bw.joinable()) bw.join();
            }

            if (g_cancel_render.load()) {
                g_is_rendering.store(false);
                return;
            }

            double total_lum = 0.0;
            std::vector<SeedPath> all_seeds;
            for (unsigned int th = 0; th < num_threads; ++th) {
                total_lum += thread_lum_sum[th];
                for (const auto& sp : thread_seeds[th]) {
                    all_seeds.push_back(sp);
                }
            }

            float f_ave = static_cast<float>(total_lum / static_cast<double>(bootstrap_count));
            if (f_ave < 1e-6f) f_ave = 1.0f;
            g_mlt_fave = f_ave;

            // 2. Równoległe łańcuchy Markowa MCMC (Jeden łańcuch na wątek CPU)
            size_t mutations_per_thread = total_mutations / num_threads;
            std::vector<std::thread> mlt_workers;

            for (unsigned int th = 0; th < num_threads; ++th) {
                mlt_workers.emplace_back([&, th]() {
                    std::mt19937 rng(99991 + th * 6271);
                    std::uniform_real_distribution<float> dist01(0.0f, 1.0f);

                    SeedPath cur_x;
                    if (!all_seeds.empty()) {
                        cur_x = all_seeds[(th * 17) % all_seeds.size()];
                    } else {
                        cur_x.u.resize(MLT_DIM);
                        for (size_t k = 0; k < MLT_DIM; ++k) cur_x.u[k] = dist01(rng);
                        PrimarySampler sampler{cur_x.u, 2, &rng};
                        Ray r = cam.get_ray(cur_x.u[0], 1.0f - cur_x.u[1]);
                        cur_x.col = evaluate_path_mlt(r, world, light_sources, photon_map, max_depth, sampler);
                        cur_x.lum = color_luminance(cur_x.col);
                    }

                    std::vector<Vec3> local_film(total_pixels, Vec3(0, 0, 0));
                    std::vector<float> u_prop(MLT_DIM);

                    int width = g_render_width;
                    int height = g_render_height;
                    float r1 = 0.1f;
                    float r2 = 0.08f * static_cast<float>(std::min(width, height));
                    float log_r2_r1 = std::log(r2 / r1);
                    float theta1 = 0.0001f;
                    float theta2 = 0.1f;
                    float log_theta = std::log(theta2 / theta1);

                    size_t chunk_size = 20000;
                    size_t steps_in_chunk = 0;

                    for (size_t m = 0; m < mutations_per_thread && !g_cancel_render.load(); ++m) {
                        bool is_large = (dist01(rng) < 0.20f);

                        if (is_large) {
                            // Nowa ścieżka (Sekcja 3.2 z pracy) - gwarantuje ergodyczność
                            for (size_t k = 0; k < MLT_DIM; ++k) u_prop[k] = dist01(rng);
                        } else {
                            bool did_manifold = false;
                            if (algo == 3 && glass_crystal && dist01(rng) < 0.50f) {
                                // Manifold Exploration Perturbation (Jakob & Marschner 2012, Sekcja 5.1 & Rysunek 5 i 6)
                                Ray cur_cam_ray = cam.get_ray(cur_x.u[0], 1.0f - cur_x.u[1]);
                                auto spec_test = TraceThroughCrystal(cur_cam_ray, glass_crystal, 1.55f);
                                if (spec_test.hit) {
                                    Vec3 target_n(0.0f, 1.0f, 0.0f);
                                    float d_plane = dot(spec_test.dir_exit, target_n);
                                    if (std::fabs(d_plane) > 1e-4f) {
                                        float tp = -spec_test.p_exit.y / d_plane;
                                        if (tp > 0.001f) {
                                            Vec3 hit_p = spec_test.p_exit + spec_test.dir_exit * tp;

                                            // Perturbacja celu na płaszczyźnie podłoża wg parametru lambda
                                            std::normal_distribution<float> norm_dist(0.0f, 0.035f);
                                            Vec3 perturbed_target = hit_p + Vec3(norm_dist(rng), 0.0f, norm_dist(rng));

                                            Vec3 walked_dir = cur_cam_ray.direction;
                                            Vec3 final_p;
                                            if (WalkSpecularManifold(cam.origin, walked_dir, glass_crystal, perturbed_target, target_n, final_p, 2e-3f, 12)) {
                                                float new_s = 0.0f, new_t = 0.0f;
                                                if (cam.project_direction(walked_dir, new_s, new_t)) {
                                                    u_prop = cur_x.u;
                                                    u_prop[0] = std::clamp(new_s, 0.0f, 0.999999f);
                                                    u_prop[1] = std::clamp(1.0f - new_t, 0.0f, 0.999999f);
                                                    did_manifold = true;
                                                }
                                            }
                                        }
                                    }
                                }
                            }

                            if (!did_manifold) {
                                u_prop = cur_x.u;
                                // Perturbacja soczewki kamery (Appendix B)
                                float px_f = cur_x.u[0] * static_cast<float>(width);
                                float py_f = cur_x.u[1] * static_cast<float>(height);
                                float phi = dist01(rng) * 6.2831853f;
                                float r = r2 * std::exp(-log_r2_r1 * dist01(rng));
                                float nx = px_f + r * std::cos(phi);
                                float ny = py_f + r * std::sin(phi);
                                nx = std::fmod(std::fmod(nx, static_cast<float>(width)) + static_cast<float>(width), static_cast<float>(width));
                                ny = std::fmod(std::fmod(ny, static_cast<float>(height)) + static_cast<float>(height), static_cast<float>(height));
                                u_prop[0] = nx / static_cast<float>(width);
                                u_prop[1] = ny / static_cast<float>(height);

                                // Perturbacja kątowa odbić promieni (Appendix C)
                                for (size_t k = 2; k < MLT_DIM; ++k) {
                                    float r_ang = theta2 * std::exp(-log_theta * dist01(rng));
                                    float sign = (dist01(rng) < 0.5f) ? -1.0f : 1.0f;
                                    float val = cur_x.u[k] + sign * r_ang * 0.25f;
                                    if (val < 0.0f) val += 1.0f;
                                    if (val >= 1.0f) val -= 1.0f;
                                    u_prop[k] = val;
                                }
                            }
                        }

                        PrimarySampler prop_sampler{u_prop, 2, &rng};
                        Ray prop_ray = cam.get_ray(u_prop[0], 1.0f - u_prop[1]);
                        Vec3 col_prop = evaluate_path_mlt(prop_ray, world, light_sources, photon_map, max_depth, prop_sampler);
                        float lum_prop = color_luminance(col_prop);

                        float A = (cur_x.lum <= 1e-7f) ? 1.0f : std::min(1.0f, lum_prop / cur_x.lum);

                        // Akumulacja wartości oczekiwanej do lokalnego filmu (Sekcja 5.1 & Rysunek 4)
                        if (lum_prop > 1e-7f && A > 1e-7f) {
                            int px_y = std::clamp(static_cast<int>(u_prop[0] * width), 0, width - 1);
                            int py_y = std::clamp(static_cast<int>(u_prop[1] * height), 0, height - 1);
                            int idx_y = py_y * width + px_y;
                            local_film[idx_y] = local_film[idx_y] + (col_prop / lum_prop) * A;
                        }

                        if (cur_x.lum > 1e-7f && (1.0f - A) > 1e-7f) {
                            int px_x = std::clamp(static_cast<int>(cur_x.u[0] * width), 0, width - 1);
                            int py_x = std::clamp(static_cast<int>(cur_x.u[1] * height), 0, height - 1);
                            int idx_x = py_x * width + px_x;
                            local_film[idx_x] = local_film[idx_x] + (cur_x.col / cur_x.lum) * (1.0f - A);
                        }

                        // Warunek akceptacji wg Detailed Balance (Równanie 2)
                        if (dist01(rng) < A) {
                            cur_x.u = u_prop;
                            cur_x.col = col_prop;
                            cur_x.lum = lum_prop;
                        }

                        steps_in_chunk++;
                        if (steps_in_chunk >= chunk_size) {
                            {
                                std::lock_guard<std::mutex> lock(g_mlt_mutex);
                                for (size_t p = 0; p < total_pixels; ++p) {
                                    g_mlt_accum[p] = g_mlt_accum[p] + local_film[p];
                                    local_film[p] = Vec3(0, 0, 0);
                                }
                                g_mlt_mutations_done.fetch_add(steps_in_chunk);
                            }
                            steps_in_chunk = 0;
                        }
                    }

                    if (steps_in_chunk > 0) {
                        std::lock_guard<std::mutex> lock(g_mlt_mutex);
                        for (size_t p = 0; p < total_pixels; ++p) {
                            g_mlt_accum[p] = g_mlt_accum[p] + local_film[p];
                        }
                        g_mlt_mutations_done.fetch_add(steps_in_chunk);
                    }
                });
            }

            for (auto& mw : mlt_workers) {
                if (mw.joinable()) mw.join();
            }

            // Ostateczne skalowanie histogramu metodą Metropolis (Równanie 1: s = fave / have)
            size_t m_done = g_mlt_mutations_done.load();
            if (m_done > 0) {
                float scale = (g_mlt_fave * total_pixels) / static_cast<float>(m_done);
                std::lock_guard<std::mutex> lock(g_mlt_mutex);
                for (size_t i = 0; i < total_pixels; ++i) {
                    Vec3 rad = g_mlt_accum[i] * scale;
                    g_raw_image[i] = rad;
                    g_sample_counts[i] = 1;

                    float cr = std::clamp(std::sqrt(rad.x), 0.0f, 0.999f);
                    float cg = std::clamp(std::sqrt(rad.y), 0.0f, 0.999f);
                    float cb = std::clamp(std::sqrt(rad.z), 0.0f, 0.999f);

                    uint8_t ir = static_cast<uint8_t>(256.0f * cr);
                    uint8_t ig = static_cast<uint8_t>(256.0f * cg);
                    uint8_t ib = static_cast<uint8_t>(256.0f * cb);

                    g_display_buffer[i] = (ib) | (ig << 8) | (ir << 16) | (0xFF << 24);
                }
            }
            g_is_rendering.store(false);
        } else {
            // ================================================================
            // KAFELKOWY STANDARDOWY PT LUB BDPT
            // ================================================================
            unsigned int num_threads = std::max(1u, std::thread::hardware_concurrency());
            std::atomic<size_t> tile_index(0);
            std::vector<std::thread> workers;

            for (unsigned int t = 0; t < num_threads; ++t) {
                workers.emplace_back([&, t]() {
                    std::mt19937 rng(1337 + t * 997);
                    std::uniform_real_distribution<float> dist01(0.0f, 1.0f);

                    while (!g_cancel_render.load()) {
                        size_t idx = tile_index.fetch_add(1);
                        if (idx >= tiles.size()) break;

                        const Tile& tile = tiles[idx];
                        int tw = tile.x1 - tile.x0;
                        int th = tile.y1 - tile.y0;

                        std::vector<std::pair<int, int>> offsets;
                        offsets.reserve(tw * th);
                        for (int dy = 0; dy < th; ++dy) {
                            for (int dx = 0; dx < tw; ++dx) {
                                offsets.emplace_back(dx, dy);
                            }
                        }

                        for (int s = 0; s < spp && !g_cancel_render.load(); ++s) {
                            std::shuffle(offsets.begin(), offsets.end(), rng);

                            for (const auto& off : offsets) {
                                if (g_cancel_render.load()) break;

                                int px = tile.x0 + off.first;
                                int py = tile.y0 + off.second;

                                // Precyzyjny TOP-DOWN raytracing zgodny z buforem
                                float u = (static_cast<float>(px) + dist01(rng)) / static_cast<float>(g_render_width);
                                float v = (static_cast<float>(g_render_height - 1 - py) + dist01(rng)) / static_cast<float>(g_render_height);

                                Ray r = cam.get_ray(u, v);
                                Vec3 col;

                                if (algo == 1) {
                                    col = ray_color_bdpt(r, world, light_sources, photon_map, max_depth, rng);
                                } else {
                                    col = ray_color_standard(r, world, max_depth, rng);
                                }

                                int p_idx = py * g_render_width + px;
                                g_raw_image[p_idx] = g_raw_image[p_idx] + col;
                                g_sample_counts[p_idx] += 1;

                                int count = g_sample_counts[p_idx];
                                Vec3 avg = g_raw_image[p_idx] / static_cast<float>(count);

                                float cr = std::clamp(std::sqrt(avg.x), 0.0f, 0.999f);
                                float cg = std::clamp(std::sqrt(avg.y), 0.0f, 0.999f);
                                float cb = std::clamp(std::sqrt(avg.z), 0.0f, 0.999f);

                                uint8_t ir = static_cast<uint8_t>(256.0f * cr);
                                uint8_t ig = static_cast<uint8_t>(256.0f * cg);
                                uint8_t ib = static_cast<uint8_t>(256.0f * cb);

                                g_display_buffer[p_idx] = (ib) | (ig << 8) | (ir << 16) | (0xFF << 24);
                            }
                        }

                        g_completed_tiles.fetch_add(1);
                    }
                });
            }

            for (auto& worker : workers) {
                if (worker.joinable()) worker.join();
            }

            g_is_rendering.store(false);
        }
    }).detach();
}

void StopRendering() {
    if (g_is_rendering.load()) {
        g_cancel_render.store(true);
        if (g_render_algo == 3) {
            SetWindowTextW(g_hStatusLabel, L"Zatrzymywanie eksploracji rozmaitosci (MEMLT)...");
        } else if (g_render_algo == 2) {
            SetWindowTextW(g_hStatusLabel, L"Zatrzymywanie lancuchow Markowa (MLT)...");
        } else {
            SetWindowTextW(g_hStatusLabel, L"Zatrzymywanie watkow kafelkowania...");
        }
    }
}

void SaveToPNG() {
    wchar_t filenameW[MAX_PATH] = L"render_kaustyka.png";
    if (g_hEditFilename) {
        wchar_t buf[MAX_PATH];
        GetWindowTextW(g_hEditFilename, buf, MAX_PATH);
        std::wstring ws(buf);
        while (!ws.empty() && iswspace(ws.front())) ws.erase(ws.begin());
        while (!ws.empty() && iswspace(ws.back())) ws.pop_back();
        if (!ws.empty()) {
            if (ws.length() < 4 || (ws.substr(ws.length() - 4) != L".png" && ws.substr(ws.length() - 4) != L".PNG")) {
                ws += L".png";
            }
            wcsncpy(filenameW, ws.c_str(), MAX_PATH - 1);
            filenameW[MAX_PATH - 1] = L'\0';
        }
    }

    char filenameA[MAX_PATH];
    WideCharToMultiByte(CP_UTF8, 0, filenameW, -1, filenameA, MAX_PATH, nullptr, nullptr);

    std::vector<uint8_t> pixels(g_render_width * g_render_height * 3);

    for (int j = 0; j < g_render_height; ++j) {
        for (int i = 0; i < g_render_width; ++i) {
            int p_idx = j * g_render_width + i;
            int count = std::max(1, g_sample_counts[p_idx]);
            Vec3 avg = g_raw_image[p_idx] / static_cast<float>(count);

            float r = std::clamp(std::sqrt(avg.x), 0.0f, 0.999f);
            float g = std::clamp(std::sqrt(avg.y), 0.0f, 0.999f);
            float b = std::clamp(std::sqrt(avg.z), 0.0f, 0.999f);

            int dst_idx = (j * g_render_width + i) * 3;
            pixels[dst_idx + 0] = static_cast<uint8_t>(256.0f * r);
            pixels[dst_idx + 1] = static_cast<uint8_t>(256.0f * g);
            pixels[dst_idx + 2] = static_cast<uint8_t>(256.0f * b);
        }
    }

    if (stbi_write_png(filenameA, g_render_width, g_render_height, 3, pixels.data(), g_render_width * 3) != 0) {
        wchar_t msg[300];
        swprintf(msg, 300, L"Zapisano pomyslnie obraz w rozdzielczosci %dx%d px!\nPlik docelowy: %s",
                 g_render_width, g_render_height, filenameW);
        MessageBoxW(g_hWnd, msg, L"Sukces zapisu PNG", MB_OK | MB_ICONINFORMATION);
    } else {
        wchar_t msg[300];
        swprintf(msg, 300, L"Blad podczas zapisu do pliku:\n%s", filenameW);
        MessageBoxW(g_hWnd, msg, L"Blad zapisu", MB_OK | MB_ICONERROR);
    }
}

// ============================================================================
// 7. PROCEDURA OBSŁUGI OKNA KANWY (PODGLĄD 3D & BLENDER GRAB/SCALE/ORBIT)
// ============================================================================

LRESULT CALLBACK CanvasWndProc(HWND hWnd, UINT msg, WPARAM wParam, LPARAM lParam) {
    switch (msg) {
        case WM_LBUTTONDOWN: {
            int mx = LOWORD(lParam);
            int my = HIWORD(lParam);
            SetFocus(hWnd);

            if (GetKeyState(VK_MENU) < 0) {
                if (GetKeyState(VK_SHIFT) < 0) {
                    g_is_panning = true;
                } else {
                    g_is_orbiting = true;
                }
                g_nav_last_mouse = {mx, my};
                SetCapture(hWnd);
                return 0;
            }

            if (g_is_grabbing) {
                ConfirmGrabMode();
            } else if (g_is_scaling) {
                ConfirmScaleMode();
            } else {
                int hit_idx = RayPickObject(mx, my);
                if (hit_idx >= 0) {
                    g_selected_object_idx = hit_idx;
                    SendMessage(g_hComboObjects, CB_SETCURSEL, hit_idx, 0);
                    UpdateObjectFieldsFromSelection(hit_idx);

                    wchar_t status[180];
                    swprintf(status, 180, L"Wybrano: [%s]. Wcisnij 'G', aby przemieszczac, lub 'S', aby skalowac.",
                             g_scene_objects[hit_idx].name.c_str());
                    SetWindowTextW(g_hStatusLabel, status);

                    RenderFastPreview();
                    InvalidateRect(hWnd, NULL, FALSE);
                }
            }
            return 0;
        }

        case WM_LBUTTONUP: {
            if (g_is_orbiting || g_is_panning) {
                g_is_orbiting = false;
                g_is_panning  = false;
                ReleaseCapture();
            }
            return 0;
        }

        case WM_MBUTTONDOWN: {
            int mx = LOWORD(lParam);
            int my = HIWORD(lParam);
            SetFocus(hWnd);

            if (GetKeyState(VK_SHIFT) < 0) {
                g_is_panning = true;
            } else {
                g_is_orbiting = true;
            }
            g_nav_last_mouse = {mx, my};
            SetCapture(hWnd);
            return 0;
        }

        case WM_MBUTTONUP: {
            g_is_orbiting = false;
            g_is_panning  = false;
            ReleaseCapture();
            return 0;
        }

        case WM_MOUSEWHEEL: {
            short zDelta = GET_WHEEL_DELTA_WPARAM(wParam);
            float factor = (zDelta > 0) ? 0.90f : 1.10f;
            g_cam_distance = std::clamp(g_cam_distance * factor, 0.8f, 35.0f);
            RenderFastPreview();
            InvalidateRect(hWnd, NULL, FALSE);
            return 0;
        }

        case WM_RBUTTONDOWN: {
            if (g_is_grabbing) {
                CancelGrabMode();
            } else if (g_is_scaling) {
                CancelScaleMode();
            }
            return 0;
        }

        case WM_MOUSEMOVE: {
            int mx = LOWORD(lParam);
            int my = HIWORD(lParam);

            if (g_is_orbiting) {
                int dx = mx - g_nav_last_mouse.x;
                int dy = my - g_nav_last_mouse.y;
                g_cam_yaw   += dx * 0.007f;
                g_cam_pitch = std::clamp(g_cam_pitch + dy * 0.007f, -1.45f, 1.45f);
                g_nav_last_mouse = {mx, my};
                RenderFastPreview();
                InvalidateRect(hWnd, NULL, FALSE);
                return 0;
            }
            else if (g_is_panning) {
                int dx = mx - g_nav_last_mouse.x;
                int dy = my - g_nav_last_mouse.y;
                Camera cam = GetCurrentCamera();
                Vec3 w = unit_vector(cam.origin - cam.lookat);
                Vec3 u = unit_vector(cross(cam.vup, w));
                Vec3 v = cross(w, u);
                g_cam_target = g_cam_target - u * (dx * 0.0025f * g_cam_distance) + v * (dy * 0.0025f * g_cam_distance);
                g_nav_last_mouse = {mx, my};
                RenderFastPreview();
                InvalidateRect(hWnd, NULL, FALSE);
                return 0;
            }
            else if (g_is_grabbing) {
                ProcessGrabMove(mx, my);
            }
            else if (g_is_scaling) {
                ProcessScaleMove(mx, my);
            }
            else {
                int hit_idx = RayPickObject(mx, my);
                if (hit_idx >= 0) {
                    SetCursor(LoadCursor(NULL, IDC_HAND));
                } else {
                    SetCursor(LoadCursor(NULL, IDC_ARROW));
                }
            }
            return 0;
        }

        case WM_SETCURSOR: {
            if (g_is_grabbing) {
                SetCursor(LoadCursor(NULL, IDC_SIZEALL));
                return TRUE;
            } else if (g_is_scaling) {
                SetCursor(LoadCursor(NULL, IDC_SIZENWSE));
                return TRUE;
            } else if (g_is_orbiting || g_is_panning) {
                SetCursor(LoadCursor(NULL, IDC_SIZEALL));
                return TRUE;
            }
            break;
        }

        case WM_KEYDOWN: {
            if (wParam == 'G' || wParam == 'g') {
                if (!g_is_grabbing) StartGrabMode();
                else ConfirmGrabMode();
                return 0;
            }
            else if (wParam == 'S' || wParam == 's') {
                if (!g_is_scaling) StartScaleMode();
                else ConfirmScaleMode();
                return 0;
            }
            else if (wParam == 'F' || wParam == 'f') {
                g_show_floor_grid = !g_show_floor_grid;
                RenderFastPreview();
                InvalidateRect(hWnd, NULL, FALSE);
                return 0;
            }
            else if (g_is_grabbing) {
                if (wParam == 'X' || wParam == 'x') {
                    g_axis_lock = (g_axis_lock == AXIS_X) ? AXIS_NONE : AXIS_X;
                    POINT pt; GetCursorPos(&pt); ScreenToClient(hWnd, &pt);
                    ProcessGrabMove(pt.x, pt.y);
                    return 0;
                }
                else if (wParam == 'Y' || wParam == 'y') {
                    g_axis_lock = (g_axis_lock == AXIS_Y) ? AXIS_NONE : AXIS_Y;
                    POINT pt; GetCursorPos(&pt); ScreenToClient(hWnd, &pt);
                    ProcessGrabMove(pt.x, pt.y);
                    return 0;
                }
                else if (wParam == 'Z' || wParam == 'z') {
                    g_axis_lock = (g_axis_lock == AXIS_Z) ? AXIS_NONE : AXIS_Z;
                    POINT pt; GetCursorPos(&pt); ScreenToClient(hWnd, &pt);
                    ProcessGrabMove(pt.x, pt.y);
                    return 0;
                }
                else if (wParam == VK_RETURN) {
                    ConfirmGrabMode();
                    return 0;
                }
                else if (wParam == VK_ESCAPE) {
                    CancelGrabMode();
                    return 0;
                }
            }
            else if (g_is_scaling) {
                if (wParam == VK_RETURN) {
                    ConfirmScaleMode();
                    return 0;
                }
                else if (wParam == VK_ESCAPE) {
                    CancelScaleMode();
                    return 0;
                }
            }
            break;
        }

        case WM_PAINT: {
            PAINTSTRUCT ps;
            HDC hdc = BeginPaint(hWnd, &ps);

            int bufW = g_cur_buf_w.load();
            int bufH = g_cur_buf_h.load();

            if (bufW > 0 && bufH > 0 && g_display_buffer.size() >= static_cast<size_t>(bufW * bufH)) {
                int destX = 0, destY = 0;
                int destW = g_canvas_view_width;
                int destH = g_canvas_view_height;

                // Centrowanie i zachowanie proporcji kadru dla aktywnego renderu Path Tracer
                if (g_is_rendering.load() || bufW != g_canvas_view_width || bufH != g_canvas_view_height) {
                    float canvasAspect = static_cast<float>(g_canvas_view_width) / static_cast<float>(std::max(1, g_canvas_view_height));
                    float bufAspect = static_cast<float>(bufW) / static_cast<float>(bufH);

                    if (canvasAspect > bufAspect) {
                        destH = g_canvas_view_height;
                        destW = static_cast<int>(destH * bufAspect);
                        destX = (g_canvas_view_width - destW) / 2;
                        destY = 0;
                    } else {
                        destW = g_canvas_view_width;
                        destH = static_cast<int>(destW / bufAspect);
                        destX = 0;
                        destY = (g_canvas_view_height - destH) / 2;
                    }

                    if (destX > 0 || destY > 0) {
                        HBRUSH hBg = CreateSolidBrush(RGB(18, 18, 22));
                        if (destX > 0) {
                            RECT rLeft = {0, 0, destX, g_canvas_view_height};
                            RECT rRight = {destX + destW, 0, g_canvas_view_width, g_canvas_view_height};
                            FillRect(hdc, &rLeft, hBg);
                            FillRect(hdc, &rRight, hBg);
                        }
                        if (destY > 0) {
                            RECT rTop = {0, 0, g_canvas_view_width, destY};
                            RECT rBottom = {0, destY + destH, g_canvas_view_width, g_canvas_view_height};
                            FillRect(hdc, &rTop, hBg);
                            FillRect(hdc, &rBottom, hBg);
                        }
                        DeleteObject(hBg);
                    }
                }

                BITMAPINFO bmi = {};
                bmi.bmiHeader.biSize = sizeof(BITMAPINFOHEADER);
                bmi.bmiHeader.biWidth = bufW;
                bmi.bmiHeader.biHeight = -bufH; // Top-Down DIB zgodny ze skanliniami bufora
                bmi.bmiHeader.biPlanes = 1;
                bmi.bmiHeader.biBitCount = 32;
                bmi.bmiHeader.biCompression = BI_RGB;

                SetStretchBltMode(hdc, HALFTONE);
                SetBrushOrgEx(hdc, 0, 0, NULL);

                StretchDIBits(
                    hdc,
                    destX, destY, destW, destH,
                    0, 0, bufW, bufH,
                    g_display_buffer.data(),
                    &bmi,
                    DIB_RGB_COLORS,
                    SRCCOPY
                );
            }

            // 2. Nakładki 3D (Wskaźnik zaznaczenia, osie X/Y/Z i kropkowane linie)
            if (!g_is_rendering.load() && g_selected_object_idx >= 0 && g_selected_object_idx < static_cast<int>(g_scene_objects.size())) {
                Camera cam = GetCurrentCamera();
                const auto& sel_obj = g_scene_objects[g_selected_object_idx];

                int sx = 0, sy = 0;
                float v_w = 2.0f * std::tan(g_cam_fov * 3.14159f / 360.0f) * (static_cast<float>(g_canvas_view_width) / g_canvas_view_height);
                float v_h = 2.0f * std::tan(g_cam_fov * 3.14159f / 360.0f);

                if (ProjectToScreen(sel_obj.center, cam, v_w, v_h, g_canvas_view_width, g_canvas_view_height, sx, sy)) {
                    if (g_is_grabbing) {
                        HPEN hPenAxis = nullptr;
                        Vec3 pA, pB;
                        if (g_axis_lock == AXIS_X) {
                            hPenAxis = CreatePen(PS_SOLID, 2, RGB(255, 60, 60));
                            pA = sel_obj.center + Vec3(-20.0f, 0.0f, 0.0f);
                            pB = sel_obj.center + Vec3( 20.0f, 0.0f, 0.0f);
                        } else if (g_axis_lock == AXIS_Y) {
                            hPenAxis = CreatePen(PS_SOLID, 2, RGB(60, 255, 60));
                            pA = sel_obj.center + Vec3(0.0f, -20.0f, 0.0f);
                            pB = sel_obj.center + Vec3(0.0f,  20.0f, 0.0f);
                        } else if (g_axis_lock == AXIS_Z) {
                            hPenAxis = CreatePen(PS_SOLID, 2, RGB(60, 140, 255));
                            pA = sel_obj.center + Vec3(0.0f, 0.0f, -20.0f);
                            pB = sel_obj.center + Vec3(0.0f, 0.0f,  20.0f);
                        }

                        if (hPenAxis) {
                            HPEN hOldPen = (HPEN)SelectObject(hdc, hPenAxis);
                            int ax1 = 0, ay1 = 0, ax2 = 0, ay2 = 0;
                            if (ProjectToScreen(pA, cam, v_w, v_h, g_canvas_view_width, g_canvas_view_height, ax1, ay1) &&
                                ProjectToScreen(pB, cam, v_w, v_h, g_canvas_view_width, g_canvas_view_height, ax2, ay2)) {
                                MoveToEx(hdc, ax1, ay1, NULL);
                                LineTo(hdc, ax2, ay2);
                            }
                            SelectObject(hdc, hOldPen);
                            DeleteObject(hPenAxis);
                        }

                        int orig_sx = 0, orig_sy = 0;
                        if (ProjectToScreen(g_grab_initial_pos, cam, v_w, v_h, g_canvas_view_width, g_canvas_view_height, orig_sx, orig_sy)) {
                            HPEN hPenDash = CreatePen(PS_DOT, 1, RGB(230, 230, 230));
                            HPEN hOldPen = (HPEN)SelectObject(hdc, hPenDash);
                            MoveToEx(hdc, orig_sx, orig_sy, NULL);
                            LineTo(hdc, sx, sy);
                            SelectObject(hdc, hOldPen);
                            DeleteObject(hPenDash);
                        }
                    }

                    COLORREF ringColor = RGB(255, 140, 0);
                    if (g_is_grabbing) ringColor = RGB(255, 230, 20);
                    else if (g_is_scaling) ringColor = RGB(80, 220, 255);

                    HPEN hPenOrange = CreatePen(PS_SOLID, 2, ringColor);
                    HBRUSH hNullBrush = (HBRUSH)GetStockObject(NULL_BRUSH);
                    HPEN hOldPen = (HPEN)SelectObject(hdc, hPenOrange);
                    HBRUSH hOldBrush = (HBRUSH)SelectObject(hdc, hNullBrush);

                    int ring_r = 18;
                    Ellipse(hdc, sx - ring_r, sy - ring_r, sx + ring_r, sy + ring_r);
                    MoveToEx(hdc, sx - 6, sy, NULL); LineTo(hdc, sx + 7, sy);
                    MoveToEx(hdc, sx, sy - 6, NULL); LineTo(hdc, sx, sy + 7);

                    SelectObject(hdc, hOldPen);
                    SelectObject(hdc, hOldBrush);
                    DeleteObject(hPenOrange);
                }
            }

            // 3. Pasek pomocniczy HUD na dole kanwy
            if (!g_is_rendering.load()) {
                SetBkMode(hdc, TRANSPARENT);
                HFONT hFont = (HFONT)GetStockObject(DEFAULT_GUI_FONT);
                HFONT hOldFont = (HFONT)SelectObject(hdc, hFont);

                RECT hudRect = { 10, g_canvas_view_height - 30, g_canvas_view_width - 10, g_canvas_view_height - 6 };
                HBRUSH hBgBrush = CreateSolidBrush(RGB(18, 22, 28));
                FillRect(hdc, &hudRect, hBgBrush);
                DeleteObject(hBgBrush);

                wchar_t hudText[240];
                if (g_is_grabbing) {
                    const wchar_t* axisName = L"Swobodny";
                    if (g_axis_lock == AXIS_X) axisName = L"Os X (Czerwona)";
                    else if (g_axis_lock == AXIS_Y) axisName = L"Os Y (Zielona)";
                    else if (g_axis_lock == AXIS_Z) axisName = L"Os Z (Niebieska)";

                    swprintf(hudText, 240, L" ✋ [BLENDER GRAB] %s | Os: %s | [X/Y/Z]: Zmien os | [LPM/Enter]: Zatwierdz | [PPM/Esc]: Anuluj",
                             g_scene_objects[g_selected_object_idx].name.c_str(), axisName);
                    SetTextColor(hdc, RGB(255, 215, 0));
                } else if (g_is_scaling) {
                    swprintf(hudText, 240, L" 📐 [BLENDER SCALE] %s | R=%.2f | Ruch mysza | [LPM/Enter]: Zatwierdz | [PPM/Esc]: Anuluj",
                             g_scene_objects[g_selected_object_idx].name.c_str(), g_scene_objects[g_selected_object_idx].radius);
                    SetTextColor(hdc, RGB(80, 220, 255));
                } else {
                    swprintf(hudText, 240, L" 🎮 [BLENDER 3D] LPM: Wybierz | G: Przesun | S: Skaluj | SPM/Alt+LPM: Obrot | Scroll: Zoom | Shift+SPM: Pan | F: Siatka",
                             (g_selected_object_idx >= 0 ? g_scene_objects[g_selected_object_idx].name.c_str() : L"Brak"));
                    SetTextColor(hdc, RGB(210, 225, 245));
                }

                DrawTextW(hdc, hudText, -1, &hudRect, DT_SINGLELINE | DT_VCENTER | DT_LEFT);

                RECT topRect = { 10, 8, 380, 28 };
                HBRUSH hTopBg = CreateSolidBrush(RGB(20, 24, 32));
                FillRect(hdc, &topRect, hTopBg);
                DeleteObject(hTopBg);
                SetTextColor(hdc, RGB(180, 200, 230));
                wchar_t topText[120];
                swprintf(topText, 120, L"  Widok 3D: Plynny cieniowany | Siatka podloza: [%s]  ", g_show_floor_grid ? L"ON" : L"OFF");
                DrawTextW(hdc, topText, -1, &topRect, DT_SINGLELINE | DT_VCENTER | DT_LEFT);

                SelectObject(hdc, hOldFont);
            }

            EndPaint(hWnd, &ps);
            return 0;
        }

        case WM_ERASEBKGND:
            return 1;
    }
    return DefWindowProc(hWnd, msg, wParam, lParam);
}

// ============================================================================
// 8. RESPONSYWNE SKALOWANIE OKNA (PEŁNE WSPARCIE MAKSYMALIZACJI)
// ============================================================================

void ResizeLayout(int clientW, int clientH) {
    if (clientW < 400 || clientH < 300) return;

    int panelW = 390;
    int sideX = clientW - panelW - 15;
    int canvasW = std::max(200, sideX - 25);
    int canvasH = std::max(150, clientH - 75);

    g_canvas_view_width  = canvasW;
    g_canvas_view_height = canvasH;

    if (g_hCanvas) {
        MoveWindow(g_hCanvas, 15, 15, canvasW, canvasH, TRUE);
    }
    if (g_hProgressBar) {
        MoveWindow(g_hProgressBar, 15, canvasH + 20, canvasW, 20, TRUE);
    }
    if (g_hStatusLabel) {
        MoveWindow(g_hStatusLabel, 15, canvasH + 44, clientW - 30, 24, TRUE);
    }

    int curY = 15;
    // 1. Grupa: Scena 3D i Import OBJ
    if (g_hGroupScene) MoveWindow(g_hGroupScene, sideX, curY, panelW, 82, TRUE);
    curY += 20;
    if (g_hLblScene) MoveWindow(g_hLblScene, sideX + 15, curY, 80, 20, TRUE);
    if (g_hComboScene) MoveWindow(g_hComboScene, sideX + 100, curY - 2, 275, 150, TRUE);
    curY += 26;
    if (g_hBtnImportObj) MoveWindow(g_hBtnImportObj, sideX + 15, curY, 360, 28, TRUE);

    curY += 44;
    // 2. Grupa: Parametry renderowania
    if (g_hGroupRender) MoveWindow(g_hGroupRender, sideX, curY, panelW, 222, TRUE);
    curY += 20;
    if (g_hLblAlgo) MoveWindow(g_hLblAlgo, sideX + 15, curY, 95, 20, TRUE);
    if (g_hComboAlgo) MoveWindow(g_hComboAlgo, sideX + 115, curY - 2, 260, 150, TRUE);

    curY += 26;
    if (g_hLblPhotons) MoveWindow(g_hLblPhotons, sideX + 15, curY, 100, 20, TRUE);
    if (g_hEditPhotons) MoveWindow(g_hEditPhotons, sideX + 115, curY - 2, 85, 22, TRUE);
    if (g_hLblPhotHint) MoveWindow(g_hLblPhotHint, sideX + 210, curY, 165, 20, TRUE);

    curY += 26;
    if (g_hLblRes) MoveWindow(g_hLblRes, sideX + 15, curY, 95, 20, TRUE);
    if (g_hComboRes) MoveWindow(g_hComboRes, sideX + 115, curY - 2, 260, 200, TRUE);

    curY += 26;
    if (g_hLblW) MoveWindow(g_hLblW, sideX + 15, curY, 80, 20, TRUE);
    if (g_hEditResW) MoveWindow(g_hEditResW, sideX + 100, curY - 2, 65, 22, TRUE);
    if (g_hLblH) MoveWindow(g_hLblH, sideX + 190, curY, 75, 20, TRUE);
    if (g_hEditResH) MoveWindow(g_hEditResH, sideX + 270, curY - 2, 65, 22, TRUE);

    curY += 26;
    if (g_hLblTile) MoveWindow(g_hLblTile, sideX + 15, curY, 95, 20, TRUE);
    if (g_hComboTile) MoveWindow(g_hComboTile, sideX + 115, curY - 2, 260, 150, TRUE);

    curY += 26;
    if (g_hLblSPP) MoveWindow(g_hLblSPP, sideX + 15, curY, 80, 20, TRUE);
    if (g_hEditSPP) MoveWindow(g_hEditSPP, sideX + 100, curY - 2, 65, 22, TRUE);
    if (g_hLblDepth) MoveWindow(g_hLblDepth, sideX + 190, curY, 75, 20, TRUE);
    if (g_hEditDepth) MoveWindow(g_hEditDepth, sideX + 270, curY - 2, 65, 22, TRUE);

    curY += 30;
    if (g_hBtnRender) MoveWindow(g_hBtnRender, sideX + 15, curY, 200, 32, TRUE);
    if (g_hBtnStop) MoveWindow(g_hBtnStop, sideX + 225, curY, 150, 32, TRUE);

    curY += 46;
    // 3. Grupa: Obiekt, Pozycja & Material
    if (g_hGroupObj) MoveWindow(g_hGroupObj, sideX, curY, panelW, 252, TRUE);
    curY += 18;
    if (g_hLblObj) MoveWindow(g_hLblObj, sideX + 15, curY, 90, 20, TRUE);
    if (g_hComboObjects) MoveWindow(g_hComboObjects, sideX + 110, curY - 2, 265, 200, TRUE);

    curY += 25;
    if (g_hLblPosX) MoveWindow(g_hLblPosX, sideX + 15, curY, 75, 20, TRUE);
    if (g_hEditPosX) MoveWindow(g_hEditPosX, sideX + 90, curY - 2, 75, 22, TRUE);
    if (g_hLblRadius) MoveWindow(g_hLblRadius, sideX + 185, curY, 70, 20, TRUE);
    if (g_hEditRadius) MoveWindow(g_hEditRadius, sideX + 260, curY - 2, 80, 22, TRUE);

    curY += 24;
    if (g_hLblPosY) MoveWindow(g_hLblPosY, sideX + 15, curY, 75, 20, TRUE);
    if (g_hEditPosY) MoveWindow(g_hEditPosY, sideX + 90, curY - 2, 75, 22, TRUE);
    if (g_hBtnApplyPos) MoveWindow(g_hBtnApplyPos, sideX + 185, curY - 2, 185, 24, TRUE);

    curY += 24;
    if (g_hLblPosZ) MoveWindow(g_hLblPosZ, sideX + 15, curY, 75, 20, TRUE);
    if (g_hEditPosZ) MoveWindow(g_hEditPosZ, sideX + 90, curY - 2, 75, 22, TRUE);
    if (g_hBtnGrab) MoveWindow(g_hBtnGrab, sideX + 185, curY - 2, 185, 24, TRUE);

    curY += 25;
    if (g_hBtnResetCam) MoveWindow(g_hBtnResetCam, sideX + 185, curY - 2, 185, 24, TRUE);

    // Sekcja Materialu wewnatrz grupy obiektow:
    curY += 28;
    if (g_hLblMatType) MoveWindow(g_hLblMatType, sideX + 15, curY, 90, 20, TRUE);
    if (g_hComboMatType) MoveWindow(g_hComboMatType, sideX + 110, curY - 2, 265, 150, TRUE);

    curY += 25;
    if (g_hLblMatCol) MoveWindow(g_hLblMatCol, sideX + 15, curY, 75, 20, TRUE);
    if (g_hEditMatR) MoveWindow(g_hEditMatR, sideX + 90, curY - 2, 50, 22, TRUE);
    if (g_hEditMatG) MoveWindow(g_hEditMatG, sideX + 145, curY - 2, 50, 22, TRUE);
    if (g_hEditMatB) MoveWindow(g_hEditMatB, sideX + 200, curY - 2, 50, 22, TRUE);
    if (g_hEditMatParam) MoveWindow(g_hEditMatParam, sideX + 260, curY - 2, 80, 22, TRUE);

    curY += 26;
    if (g_hBtnApplyMat) MoveWindow(g_hBtnApplyMat, sideX + 15, curY, 360, 26, TRUE);

    curY += 36;
    // 4. Grupa: Zapis PNG
    if (g_hGroupSave) MoveWindow(g_hGroupSave, sideX, curY, panelW, 78, TRUE);
    curY += 18;
    if (g_hLblFile) MoveWindow(g_hLblFile, sideX + 15, curY, 80, 20, TRUE);
    if (g_hEditFilename) MoveWindow(g_hEditFilename, sideX + 100, curY - 2, 275, 22, TRUE);

    curY += 24;
    if (g_hBtnSave) MoveWindow(g_hBtnSave, sideX + 15, curY, 360, 26, TRUE);

    // Renderujemy szybki podgląd tylko jeśli renderowanie PT nie trwa w tle!
    if (!g_is_rendering.load()) {
        RenderFastPreview();
    }
    if (g_hCanvas) InvalidateRect(g_hCanvas, NULL, FALSE);
}

// ============================================================================
// 9. GŁÓWNA PROCEDURA OKNA
// ============================================================================

LRESULT CALLBACK MainWndProc(HWND hWnd, UINT msg, WPARAM wParam, LPARAM lParam) {
    switch (msg) {
        case WM_CREATE: {
            WNDCLASSW wcCanvas = {};
            wcCanvas.lpfnWndProc = CanvasWndProc;
            wcCanvas.hInstance = GetModuleHandle(NULL);
            wcCanvas.lpszClassName = L"PathTracerCanvas";
            wcCanvas.hCursor = LoadCursor(NULL, IDC_ARROW);
            RegisterClassW(&wcCanvas);

            g_hCanvas = CreateWindowExW(
                WS_EX_CLIENTEDGE, L"PathTracerCanvas", nullptr,
                WS_CHILD | WS_VISIBLE,
                15, 15, g_canvas_view_width, g_canvas_view_height,
                hWnd, nullptr, GetModuleHandle(NULL), nullptr
            );

            HFONT hFont = (HFONT)GetStockObject(DEFAULT_GUI_FONT);
            int sideX = 835;
            int curY = 15;

            // 1. Grupa: Scena 3D & Import OBJ
            g_hGroupScene = CreateWindowW(L"BUTTON", L" Wybor Sceny 3D & Import Geometrii ",
                WS_CHILD | WS_VISIBLE | BS_GROUPBOX,
                sideX, curY, 390, 82, hWnd, nullptr, nullptr, nullptr);
            SendMessage(g_hGroupScene, WM_SETFONT, (WPARAM)hFont, TRUE);

            curY += 20;
            g_hLblScene = CreateWindowW(L"STATIC", L"Scena 3D:", WS_CHILD | WS_VISIBLE, sideX + 15, curY, 80, 20, hWnd, nullptr, nullptr, nullptr);
            SendMessage(g_hLblScene, WM_SETFONT, (WPARAM)hFont, TRUE);

            g_hComboScene = CreateWindowW(L"COMBOBOX", nullptr, WS_CHILD | WS_VISIBLE | CBS_DROPDOWNLIST,
                sideX + 100, curY - 2, 275, 150, hWnd, (HMENU)IDC_COMBO_SCENE, nullptr, nullptr);
            SendMessage(g_hComboScene, WM_SETFONT, (WPARAM)hFont, TRUE);
            SendMessage(g_hComboScene, CB_ADDSTRING, 0, (LPARAM)L"1. Krysztal Szklany & Kule (Kaustyka)");
            SendMessage(g_hComboScene, CB_ADDSTRING, 0, (LPARAM)L"2. Zamkniety Pokoj ze Szczelina (Manifold)");
            SendMessage(g_hComboScene, CB_SETCURSEL, 0, 0);

            curY += 26;
            g_hBtnImportObj = CreateWindowW(L"BUTTON", L"📂 Importuj model 3D Wavefront (.OBJ)", WS_CHILD | WS_VISIBLE | BS_PUSHBUTTON,
                sideX + 15, curY, 360, 28, hWnd, (HMENU)IDC_BTN_IMPORT_OBJ, nullptr, nullptr);
            SendMessage(g_hBtnImportObj, WM_SETFONT, (WPARAM)hFont, TRUE);

            curY += 44;
            // 2. Grupa: Parametry renderowania & Algorytm
            g_hGroupRender = CreateWindowW(L"BUTTON", L" Parametry renderowania & Algorytm ",
                WS_CHILD | WS_VISIBLE | BS_GROUPBOX,
                sideX, curY, 390, 222, hWnd, nullptr, nullptr, nullptr);
            SendMessage(g_hGroupRender, WM_SETFONT, (WPARAM)hFont, TRUE);

            curY += 20;
            g_hLblAlgo = CreateWindowW(L"STATIC", L"Algorytm PT:", WS_CHILD | WS_VISIBLE, sideX + 15, curY, 95, 20, hWnd, nullptr, nullptr, nullptr);
            SendMessage(g_hLblAlgo, WM_SETFONT, (WPARAM)hFont, TRUE);

            g_hComboAlgo = CreateWindowW(L"COMBOBOX", nullptr, WS_CHILD | WS_VISIBLE | CBS_DROPDOWNLIST,
                sideX + 115, curY - 2, 260, 150, hWnd, (HMENU)IDC_COMBO_ALGO, nullptr, nullptr);
            SendMessage(g_hComboAlgo, WM_SETFONT, (WPARAM)hFont, TRUE);
            SendMessage(g_hComboAlgo, CB_ADDSTRING, 0, (LPARAM)L"Standardowy Path Tracing (PT)");
            SendMessage(g_hComboAlgo, CB_ADDSTRING, 0, (LPARAM)L"BDPT (Globalna Mapa Fotonow - Krysztal)");
            SendMessage(g_hComboAlgo, CB_ADDSTRING, 0, (LPARAM)L"Metropolis Light Transport (MLT)");
            SendMessage(g_hComboAlgo, CB_ADDSTRING, 0, (LPARAM)L"Manifold Exploration (MEMLT - Rozmaitosci)");
            SendMessage(g_hComboAlgo, CB_SETCURSEL, 1, 0);

            curY += 26;
            g_hLblPhotons = CreateWindowW(L"STATIC", L"Fotony kaustyki:", WS_CHILD | WS_VISIBLE, sideX + 15, curY, 100, 20, hWnd, nullptr, nullptr, nullptr);
            SendMessage(g_hLblPhotons, WM_SETFONT, (WPARAM)hFont, TRUE);

            g_hEditPhotons = CreateWindowExW(WS_EX_CLIENTEDGE, L"EDIT", L"50000", WS_CHILD | WS_VISIBLE | ES_NUMBER,
                sideX + 115, curY - 2, 85, 22, hWnd, (HMENU)IDC_EDIT_PHOTONS, nullptr, nullptr);
            SendMessage(g_hEditPhotons, WM_SETFONT, (WPARAM)hFont, TRUE);

            g_hLblPhotHint = CreateWindowW(L"STATIC", L"(np. 20000 - 150000)", WS_CHILD | WS_VISIBLE, sideX + 210, curY, 165, 20, hWnd, nullptr, nullptr, nullptr);
            SendMessage(g_hLblPhotHint, WM_SETFONT, (WPARAM)hFont, TRUE);

            curY += 26;
            g_hLblRes = CreateWindowW(L"STATIC", L"Rozdzielczosc:", WS_CHILD | WS_VISIBLE, sideX + 15, curY, 95, 20, hWnd, nullptr, nullptr, nullptr);
            SendMessage(g_hLblRes, WM_SETFONT, (WPARAM)hFont, TRUE);

            g_hComboRes = CreateWindowW(L"COMBOBOX", nullptr, WS_CHILD | WS_VISIBLE | CBS_DROPDOWNLIST | WS_VSCROLL,
                sideX + 115, curY - 2, 260, 200, hWnd, (HMENU)IDC_COMBO_RES, nullptr, nullptr);
            SendMessage(g_hComboRes, WM_SETFONT, (WPARAM)hFont, TRUE);
            SendMessage(g_hComboRes, CB_ADDSTRING, 0, (LPARAM)L"640 x 360  (Bardzo szybki podglad)");
            SendMessage(g_hComboRes, CB_ADDSTRING, 0, (LPARAM)L"800 x 450  (Standardowy podglad)");
            SendMessage(g_hComboRes, CB_ADDSTRING, 0, (LPARAM)L"960 x 540  (qHD)");
            SendMessage(g_hComboRes, CB_ADDSTRING, 0, (LPARAM)L"1280 x 720 (HD 720p)");
            SendMessage(g_hComboRes, CB_ADDSTRING, 0, (LPARAM)L"1920 x 1080 (Full HD 1080p)");
            SendMessage(g_hComboRes, CB_ADDSTRING, 0, (LPARAM)L"Niestandardowa (wpisz ponizej)");
            SendMessage(g_hComboRes, CB_SETCURSEL, 1, 0);

            curY += 26;
            g_hLblW = CreateWindowW(L"STATIC", L"Szerokosc W:", WS_CHILD | WS_VISIBLE, sideX + 15, curY, 80, 20, hWnd, nullptr, nullptr, nullptr);
            SendMessage(g_hLblW, WM_SETFONT, (WPARAM)hFont, TRUE);
            g_hEditResW = CreateWindowExW(WS_EX_CLIENTEDGE, L"EDIT", L"800", WS_CHILD | WS_VISIBLE | ES_NUMBER, sideX + 100, curY - 2, 65, 22, hWnd, (HMENU)IDC_EDIT_RES_W, nullptr, nullptr);
            SendMessage(g_hEditResW, WM_SETFONT, (WPARAM)hFont, TRUE);

            g_hLblH = CreateWindowW(L"STATIC", L"Wysokosc H:", WS_CHILD | WS_VISIBLE, sideX + 190, curY, 75, 20, hWnd, nullptr, nullptr, nullptr);
            SendMessage(g_hLblH, WM_SETFONT, (WPARAM)hFont, TRUE);
            g_hEditResH = CreateWindowExW(WS_EX_CLIENTEDGE, L"EDIT", L"450", WS_CHILD | WS_VISIBLE | ES_NUMBER, sideX + 270, curY - 2, 65, 22, hWnd, (HMENU)IDC_EDIT_RES_H, nullptr, nullptr);
            SendMessage(g_hEditResH, WM_SETFONT, (WPARAM)hFont, TRUE);

            curY += 26;
            g_hLblTile = CreateWindowW(L"STATIC", L"Kafelek (Tiling):", WS_CHILD | WS_VISIBLE, sideX + 15, curY, 95, 20, hWnd, nullptr, nullptr, nullptr);
            SendMessage(g_hLblTile, WM_SETFONT, (WPARAM)hFont, TRUE);

            g_hComboTile = CreateWindowW(L"COMBOBOX", nullptr, WS_CHILD | WS_VISIBLE | CBS_DROPDOWNLIST,
                sideX + 115, curY - 2, 260, 150, hWnd, (HMENU)IDC_COMBO_TILE, nullptr, nullptr);
            SendMessage(g_hComboTile, WM_SETFONT, (WPARAM)hFont, TRUE);
            SendMessage(g_hComboTile, CB_ADDSTRING, 0, (LPARAM)L"16 x 16 px (Drobny - bez szwow)");
            SendMessage(g_hComboTile, CB_ADDSTRING, 0, (LPARAM)L"32 x 32 px (Standardowy - optymalny)");
            SendMessage(g_hComboTile, CB_ADDSTRING, 0, (LPARAM)L"64 x 64 px (Duze kafelki)");
            SendMessage(g_hComboTile, CB_SETCURSEL, 0, 0);

            curY += 26;
            g_hLblSPP = CreateWindowW(L"STATIC", L"Probki (SPP):", WS_CHILD | WS_VISIBLE, sideX + 15, curY, 80, 20, hWnd, nullptr, nullptr, nullptr);
            SendMessage(g_hLblSPP, WM_SETFONT, (WPARAM)hFont, TRUE);
            g_hEditSPP = CreateWindowExW(WS_EX_CLIENTEDGE, L"EDIT", L"64", WS_CHILD | WS_VISIBLE | ES_NUMBER, sideX + 100, curY - 2, 65, 22, hWnd, (HMENU)IDC_EDIT_SPP, nullptr, nullptr);
            SendMessage(g_hEditSPP, WM_SETFONT, (WPARAM)hFont, TRUE);

            g_hLblDepth = CreateWindowW(L"STATIC", L"Maks. odbic:", WS_CHILD | WS_VISIBLE, sideX + 190, curY, 75, 20, hWnd, nullptr, nullptr, nullptr);
            SendMessage(g_hLblDepth, WM_SETFONT, (WPARAM)hFont, TRUE);
            g_hEditDepth = CreateWindowExW(WS_EX_CLIENTEDGE, L"EDIT", L"16", WS_CHILD | WS_VISIBLE | ES_NUMBER, sideX + 270, curY - 2, 65, 22, hWnd, (HMENU)IDC_EDIT_DEPTH, nullptr, nullptr);
            SendMessage(g_hEditDepth, WM_SETFONT, (WPARAM)hFont, TRUE);

            curY += 30;
            g_hBtnRender = CreateWindowW(L"BUTTON", L"▶ Rozpocznij renderowanie", WS_CHILD | WS_VISIBLE | BS_PUSHBUTTON,
                sideX + 15, curY, 200, 32, hWnd, (HMENU)IDC_BTN_RENDER, nullptr, nullptr);
            SendMessage(g_hBtnRender, WM_SETFONT, (WPARAM)hFont, TRUE);

            g_hBtnStop = CreateWindowW(L"BUTTON", L"⏹ Przerwij", WS_CHILD | WS_VISIBLE | BS_PUSHBUTTON | WS_DISABLED,
                sideX + 225, curY, 150, 32, hWnd, (HMENU)IDC_BTN_STOP, nullptr, nullptr);
            SendMessage(g_hBtnStop, WM_SETFONT, (WPARAM)hFont, TRUE);

            curY += 46;

            // 3. Grupa: Obiekt, Pozycja & Material
            g_hGroupObj = CreateWindowW(L"BUTTON", L" Edycja Obiektu, Pozycji (Blender 3D) & Materialu ",
                WS_CHILD | WS_VISIBLE | BS_GROUPBOX,
                sideX, curY, 390, 252, hWnd, nullptr, nullptr, nullptr);
            SendMessage(g_hGroupObj, WM_SETFONT, (WPARAM)hFont, TRUE);

            curY += 18;
            g_hLblObj = CreateWindowW(L"STATIC", L"Wybierz obiekt:", WS_CHILD | WS_VISIBLE, sideX + 15, curY, 90, 20, hWnd, nullptr, nullptr, nullptr);
            SendMessage(g_hLblObj, WM_SETFONT, (WPARAM)hFont, TRUE);

            g_hComboObjects = CreateWindowW(L"COMBOBOX", nullptr, WS_CHILD | WS_VISIBLE | CBS_DROPDOWNLIST | WS_VSCROLL,
                sideX + 110, curY - 2, 265, 200, hWnd, (HMENU)IDC_COMBO_OBJECTS, nullptr, nullptr);
            SendMessage(g_hComboObjects, WM_SETFONT, (WPARAM)hFont, TRUE);

            curY += 25;
            g_hLblPosX = CreateWindowW(L"STATIC", L"Pozycja X:", WS_CHILD | WS_VISIBLE, sideX + 15, curY, 75, 20, hWnd, nullptr, nullptr, nullptr);
            SendMessage(g_hLblPosX, WM_SETFONT, (WPARAM)hFont, TRUE);
            g_hEditPosX = CreateWindowExW(WS_EX_CLIENTEDGE, L"EDIT", L"0.00", WS_CHILD | WS_VISIBLE, sideX + 90, curY - 2, 75, 22, hWnd, nullptr, nullptr, nullptr);
            SendMessage(g_hEditPosX, WM_SETFONT, (WPARAM)hFont, TRUE);

            g_hLblRadius = CreateWindowW(L"STATIC", L"Promien R:", WS_CHILD | WS_VISIBLE, sideX + 185, curY, 70, 20, hWnd, nullptr, nullptr, nullptr);
            SendMessage(g_hLblRadius, WM_SETFONT, (WPARAM)hFont, TRUE);
            g_hEditRadius = CreateWindowExW(WS_EX_CLIENTEDGE, L"EDIT", L"0.65", WS_CHILD | WS_VISIBLE, sideX + 260, curY - 2, 80, 22, hWnd, nullptr, nullptr, nullptr);
            SendMessage(g_hEditRadius, WM_SETFONT, (WPARAM)hFont, TRUE);

            curY += 24;
            g_hLblPosY = CreateWindowW(L"STATIC", L"Pozycja Y:", WS_CHILD | WS_VISIBLE, sideX + 15, curY, 75, 20, hWnd, nullptr, nullptr, nullptr);
            SendMessage(g_hLblPosY, WM_SETFONT, (WPARAM)hFont, TRUE);
            g_hEditPosY = CreateWindowExW(WS_EX_CLIENTEDGE, L"EDIT", L"0.00", WS_CHILD | WS_VISIBLE, sideX + 90, curY - 2, 75, 22, hWnd, nullptr, nullptr, nullptr);
            SendMessage(g_hEditPosY, WM_SETFONT, (WPARAM)hFont, TRUE);

            g_hBtnApplyPos = CreateWindowW(L"BUTTON", L"Zastosuj pozycje", WS_CHILD | WS_VISIBLE | BS_PUSHBUTTON,
                sideX + 185, curY - 2, 185, 24, hWnd, (HMENU)IDC_BTN_APPLY_POS, nullptr, nullptr);
            SendMessage(g_hBtnApplyPos, WM_SETFONT, (WPARAM)hFont, TRUE);

            curY += 24;
            g_hLblPosZ = CreateWindowW(L"STATIC", L"Pozycja Z:", WS_CHILD | WS_VISIBLE, sideX + 15, curY, 75, 20, hWnd, nullptr, nullptr, nullptr);
            SendMessage(g_hLblPosZ, WM_SETFONT, (WPARAM)hFont, TRUE);
            g_hEditPosZ = CreateWindowExW(WS_EX_CLIENTEDGE, L"EDIT", L"0.00", WS_CHILD | WS_VISIBLE, sideX + 90, curY - 2, 75, 22, hWnd, nullptr, nullptr, nullptr);
            SendMessage(g_hEditPosZ, WM_SETFONT, (WPARAM)hFont, TRUE);

            g_hBtnGrab = CreateWindowW(L"BUTTON", L"✋ Przesun w 3D (G)", WS_CHILD | WS_VISIBLE | BS_PUSHBUTTON,
                sideX + 185, curY - 2, 185, 24, hWnd, (HMENU)IDC_BTN_GRAB, nullptr, nullptr);
            SendMessage(g_hBtnGrab, WM_SETFONT, (WPARAM)hFont, TRUE);

            curY += 25;
            g_hBtnResetCam = CreateWindowW(L"BUTTON", L"🎥 Reset kamery 3D", WS_CHILD | WS_VISIBLE | BS_PUSHBUTTON,
                sideX + 185, curY - 2, 185, 24, hWnd, (HMENU)IDC_BTN_RESET_CAM, nullptr, nullptr);
            SendMessage(g_hBtnResetCam, WM_SETFONT, (WPARAM)hFont, TRUE);

            // Material Editor wewnatrz grupy obiektow:
            curY += 28;
            g_hLblMatType = CreateWindowW(L"STATIC", L"Material:", WS_CHILD | WS_VISIBLE, sideX + 15, curY, 90, 20, hWnd, nullptr, nullptr, nullptr);
            SendMessage(g_hLblMatType, WM_SETFONT, (WPARAM)hFont, TRUE);

            g_hComboMatType = CreateWindowW(L"COMBOBOX", nullptr, WS_CHILD | WS_VISIBLE | CBS_DROPDOWNLIST,
                sideX + 110, curY - 2, 265, 150, hWnd, (HMENU)IDC_COMBO_MAT_TYPE, nullptr, nullptr);
            SendMessage(g_hComboMatType, WM_SETFONT, (WPARAM)hFont, TRUE);
            SendMessage(g_hComboMatType, CB_ADDSTRING, 0, (LPARAM)L"Lambertian (Matowy / Rozproszony)");
            SendMessage(g_hComboMatType, CB_ADDSTRING, 0, (LPARAM)L"Metal (Zwierciadlany / Lustro)");
            SendMessage(g_hComboMatType, CB_ADDSTRING, 0, (LPARAM)L"Dielectric (Szklo / Krysztal)");
            SendMessage(g_hComboMatType, CB_ADDSTRING, 0, (LPARAM)L"DiffuseLight (Emisja Swiatla)");
            SendMessage(g_hComboMatType, CB_SETCURSEL, 0, 0);

            curY += 25;
            g_hLblMatCol = CreateWindowW(L"STATIC", L"RGB / Param:", WS_CHILD | WS_VISIBLE, sideX + 15, curY, 75, 20, hWnd, nullptr, nullptr, nullptr);
            SendMessage(g_hLblMatCol, WM_SETFONT, (WPARAM)hFont, TRUE);

            g_hEditMatR = CreateWindowExW(WS_EX_CLIENTEDGE, L"EDIT", L"0.85", WS_CHILD | WS_VISIBLE, sideX + 90, curY - 2, 50, 22, hWnd, (HMENU)IDC_EDIT_MAT_R, nullptr, nullptr);
            SendMessage(g_hEditMatR, WM_SETFONT, (WPARAM)hFont, TRUE);

            g_hEditMatG = CreateWindowExW(WS_EX_CLIENTEDGE, L"EDIT", L"0.85", WS_CHILD | WS_VISIBLE, sideX + 145, curY - 2, 50, 22, hWnd, (HMENU)IDC_EDIT_MAT_G, nullptr, nullptr);
            SendMessage(g_hEditMatG, WM_SETFONT, (WPARAM)hFont, TRUE);

            g_hEditMatB = CreateWindowExW(WS_EX_CLIENTEDGE, L"EDIT", L"0.85", WS_CHILD | WS_VISIBLE, sideX + 200, curY - 2, 50, 22, hWnd, (HMENU)IDC_EDIT_MAT_B, nullptr, nullptr);
            SendMessage(g_hEditMatB, WM_SETFONT, (WPARAM)hFont, TRUE);

            g_hEditMatParam = CreateWindowExW(WS_EX_CLIENTEDGE, L"EDIT", L"0.05", WS_CHILD | WS_VISIBLE, sideX + 260, curY - 2, 80, 22, hWnd, (HMENU)IDC_EDIT_MAT_PARAM, nullptr, nullptr);
            SendMessage(g_hEditMatParam, WM_SETFONT, (WPARAM)hFont, TRUE);

            curY += 26;
            g_hBtnApplyMat = CreateWindowW(L"BUTTON", L"🎨 Zastosuj Material do Wybranego Obiektu", WS_CHILD | WS_VISIBLE | BS_PUSHBUTTON,
                sideX + 15, curY, 360, 26, hWnd, (HMENU)IDC_BTN_APPLY_MAT, nullptr, nullptr);
            SendMessage(g_hBtnApplyMat, WM_SETFONT, (WPARAM)hFont, TRUE);

            curY += 36;

            // 4. Grupa: Zapis PNG
            g_hGroupSave = CreateWindowW(L"BUTTON", L" Zapis obrazu do pliku PNG ",
                WS_CHILD | WS_VISIBLE | BS_GROUPBOX,
                sideX, curY, 390, 78, hWnd, nullptr, nullptr, nullptr);
            SendMessage(g_hGroupSave, WM_SETFONT, (WPARAM)hFont, TRUE);

            curY += 18;
            g_hLblFile = CreateWindowW(L"STATIC", L"Nazwa pliku:", WS_CHILD | WS_VISIBLE, sideX + 15, curY, 80, 20, hWnd, nullptr, nullptr, nullptr);
            SendMessage(g_hLblFile, WM_SETFONT, (WPARAM)hFont, TRUE);

            g_hEditFilename = CreateWindowExW(WS_EX_CLIENTEDGE, L"EDIT", L"render_kaustyka.png", WS_CHILD | WS_VISIBLE,
                sideX + 100, curY - 2, 275, 22, hWnd, (HMENU)IDC_EDIT_FILENAME, nullptr, nullptr);
            SendMessage(g_hEditFilename, WM_SETFONT, (WPARAM)hFont, TRUE);

            curY += 24;
            g_hBtnSave = CreateWindowW(L"BUTTON", L"💾 Zapisz obraz PNG",
                WS_CHILD | WS_VISIBLE | BS_PUSHBUTTON,
                sideX + 15, curY, 360, 26, hWnd, (HMENU)IDC_BTN_SAVE_PNG, nullptr, nullptr);
            SendMessage(g_hBtnSave, WM_SETFONT, (WPARAM)hFont, TRUE);

            g_hProgressBar = CreateWindowExW(0, PROGRESS_CLASSW, nullptr,
                WS_CHILD | WS_VISIBLE | PBS_SMOOTH,
                15, 480, g_canvas_view_width, 22, hWnd, (HMENU)IDC_PROGRESS_BAR, nullptr, nullptr);

            g_hStatusLabel = CreateWindowW(L"STATIC", L"Gotowy. Pelna kontrola 3D: LPM=Wybierz, G=Przesun, S=Skaluj, SPM/Alt+LPM=Obrot, Scroll=Zoom.",
                WS_CHILD | WS_VISIBLE,
                15, 510, g_canvas_view_width + 390, 24, hWnd, (HMENU)IDC_STATUS_LABEL, nullptr, nullptr);
            SendMessage(g_hStatusLabel, WM_SETFONT, (WPARAM)hFont, TRUE);

            SwitchScene(0);

            SetTimer(hWnd, TIMER_UPDATE_ID, 33, nullptr);
            break;
        }

        case WM_SIZE: {
            if (wParam != SIZE_MINIMIZED) {
                int clientW = LOWORD(lParam);
                int clientH = HIWORD(lParam);
                ResizeLayout(clientW, clientH);
            }
            break;
        }

        case WM_TIMER: {
            if (wParam == TIMER_UPDATE_ID) {
                if (g_is_rendering.load()) {
                    auto now = std::chrono::high_resolution_clock::now();
                    std::chrono::duration<double> elapsed = now - g_render_start_time;

                    if (g_render_algo == 2 || g_render_algo == 3) {
                        size_t done = g_mlt_mutations_done.load();
                        size_t total = g_mlt_total_mutations.load();
                        int pos = (total > 0) ? static_cast<int>(done * 1000 / total) : 0;
                        int percent = (total > 0) ? static_cast<int>(done * 100 / total) : 0;
                        SendMessage(g_hProgressBar, PBM_SETPOS, pos, 0);

                        size_t total_pixels = static_cast<size_t>(g_render_width) * g_render_height;
                        if (done > 0 && g_mlt_accum.size() >= total_pixels) {
                            float scale = (g_mlt_fave * total_pixels) / static_cast<float>(done);
                            std::lock_guard<std::mutex> lock(g_mlt_mutex);
                            for (size_t i = 0; i < total_pixels; ++i) {
                                Vec3 rad = g_mlt_accum[i] * scale;
                                g_raw_image[i] = rad;
                                g_sample_counts[i] = 1;

                                float cr = std::clamp(std::sqrt(rad.x), 0.0f, 0.999f);
                                float cg = std::clamp(std::sqrt(rad.y), 0.0f, 0.999f);
                                float cb = std::clamp(std::sqrt(rad.z), 0.0f, 0.999f);

                                uint8_t ir = static_cast<uint8_t>(256.0f * cr);
                                uint8_t ig = static_cast<uint8_t>(256.0f * cg);
                                uint8_t ib = static_cast<uint8_t>(256.0f * cb);

                                g_display_buffer[i] = (ib) | (ig << 8) | (ir << 16) | (0xFF << 24);
                            }
                        }

                        const wchar_t* algoLabel = (g_render_algo == 3) ? L"Manifold Exploration MEMLT" : L"Metropolis Light Transport";
                        wchar_t status[200];
                        swprintf(status, 200, L"Renderowanie [%s]... %d%% (%zu/%zu mutacji) [Czas: %.1f s]",
                                 algoLabel, percent, done, total, elapsed.count());
                        SetWindowTextW(g_hStatusLabel, status);
                    } else {
                        size_t done = g_completed_tiles.load();
                        SendMessage(g_hProgressBar, PBM_SETPOS, done, 0);

                        int percent = (g_total_tiles > 0) ? static_cast<int>(done * 100 / g_total_tiles) : 0;
                        const wchar_t* modeTag = (g_render_algo == 1) ? L"BDPT + Mapa Fotonow" : L"PT";

                        wchar_t status[180];
                        swprintf(status, 180, L"Renderowanie [%s]... %d%% (%zu/%zu kafelkow) [Czas: %.1f s, Fotony: %d]",
                                 modeTag, percent, done, g_total_tiles, elapsed.count(), g_photon_count);
                        SetWindowTextW(g_hStatusLabel, status);
                    }

                    InvalidateRect(g_hCanvas, NULL, FALSE);
                } else {
                    if (!IsWindowEnabled(g_hBtnRender)) {
                        EnableWindow(g_hBtnRender, TRUE);
                        EnableWindow(g_hBtnStop, FALSE);
                        EnableWindow(g_hBtnApplyPos, TRUE);
                        EnableWindow(g_hBtnGrab, TRUE);

                        auto now = std::chrono::high_resolution_clock::now();
                        std::chrono::duration<double> elapsed = now - g_render_start_time;

                        const wchar_t* modeTag = (g_render_algo == 3) ? L"MEMLT" : ((g_render_algo == 2) ? L"MLT" : ((g_render_algo == 1) ? L"BDPT" : L"PT"));
                        wchar_t status[180];

                        if (g_render_algo == 2 || g_render_algo == 3) {
                            size_t done = g_mlt_mutations_done.load();
                            size_t total = g_mlt_total_mutations.load();
                            SendMessage(g_hProgressBar, PBM_SETPOS, 1000, 0);
                            if (g_cancel_render.load()) {
                                swprintf(status, 180, L"[%s] Renderowanie przerwane na %zu/%zu mutacjach.", modeTag, done, total);
                            } else {
                                swprintf(status, 180, L"[%s] Renderowanie ukonczone w %.2f s! (%zu mutacji, %dx%d px).",
                                         modeTag, elapsed.count(), done, g_render_width, g_render_height);
                            }
                        } else {
                            size_t done = g_completed_tiles.load();
                            SendMessage(g_hProgressBar, PBM_SETPOS, done, 0);
                            if (g_cancel_render.load()) {
                                swprintf(status, 180, L"[%s] Renderowanie przerwane na kafelku %zu/%zu.", modeTag, done, g_total_tiles);
                            } else {
                                swprintf(status, 180, L"[%s] Renderowanie ukonczone w %.2f s! Rozdzielczosc: %dx%d px.",
                                         modeTag, elapsed.count(), g_render_width, g_render_height);
                            }
                        }

                        SetWindowTextW(g_hStatusLabel, status);
                        InvalidateRect(g_hCanvas, NULL, FALSE);
                    }
                }
            }
            break;
        }

        case WM_COMMAND: {
            int wmId = LOWORD(wParam);
            int wmEvent = HIWORD(wParam);

            if (wmId == IDC_COMBO_SCENE && wmEvent == CBN_SELCHANGE) {
                int sel = static_cast<int>(SendMessage(g_hComboScene, CB_GETCURSEL, 0, 0));
                SwitchScene(sel);
            }
            else if (wmId == IDC_BTN_IMPORT_OBJ) {
                ImportOBJModel();
            }
            else if (wmId == IDC_BTN_APPLY_MAT) {
                ApplyMaterialFromFields();
            }
            else if (wmId == IDC_COMBO_ALGO && wmEvent == CBN_SELCHANGE) {
                int sel = static_cast<int>(SendMessage(g_hComboAlgo, CB_GETCURSEL, 0, 0));
                g_render_algo = sel;
                if (sel == 2 || sel == 3) {
                    SetWindowTextW(g_hLblSPP, L"Mutacje/px:");
                    EnableWindow(g_hComboTile, FALSE);
                } else {
                    SetWindowTextW(g_hLblSPP, L"Probki (SPP):");
                    EnableWindow(g_hComboTile, TRUE);
                }
            }
            else if (wmId == IDC_COMBO_RES && wmEvent == CBN_SELCHANGE) {
                OnResolutionComboChanged();
            }
            else if (wmId == IDC_COMBO_OBJECTS && wmEvent == CBN_SELCHANGE) {
                int sel = static_cast<int>(SendMessage(g_hComboObjects, CB_GETCURSEL, 0, 0));
                g_selected_object_idx = sel;
                UpdateObjectFieldsFromSelection(sel);
                RenderFastPreview();
                InvalidateRect(g_hCanvas, NULL, FALSE);
            }
            else if (wmId == IDC_BTN_APPLY_POS) {
                ApplyPositionFromFields();
            }
            else if (wmId == IDC_BTN_GRAB) {
                if (!g_is_grabbing) StartGrabMode();
                else ConfirmGrabMode();
            }
            else if (wmId == IDC_BTN_RESET_CAM) {
                ResetCamera();
            }
            else if (wmId == IDC_BTN_RENDER) {
                StartRendering();
            }
            else if (wmId == IDC_BTN_STOP) {
                StopRendering();
                RenderFastPreview();
                InvalidateRect(g_hCanvas, NULL, FALSE);
            }
            else if (wmId == IDC_BTN_SAVE_PNG) {
                SaveToPNG();
            }
            break;
        }

        case WM_DESTROY:
            KillTimer(hWnd, TIMER_UPDATE_ID);
            StopRendering();
            PostQuitMessage(0);
            break;

        default:
            return DefWindowProc(hWnd, msg, wParam, lParam);
    }
    return 0;
}

// ============================================================================
// 10. PUNKT WEJŚCIA PROGRAMU (WinMain)
// ============================================================================

int WINAPI WinMain(HINSTANCE hInstance, HINSTANCE, LPSTR, int nCmdShow) {
    INITCOMMONCONTROLSEX icex;
    icex.dwSize = sizeof(INITCOMMONCONTROLSEX);
    icex.dwICC = ICC_STANDARD_CLASSES | ICC_PROGRESS_CLASS;
    InitCommonControlsEx(&icex);

    WNDCLASSW wc = {};
    wc.lpfnWndProc = MainWndProc;
    wc.hInstance = hInstance;
    wc.lpszClassName = L"PathTracerMainWindow";
    wc.hCursor = LoadCursor(NULL, IDC_ARROW);
    wc.hbrBackground = (HBRUSH)(COLOR_BTNFACE + 1);

    RegisterClassW(&wc);

    int winWidth  = 1320;
    int winHeight = 720;

    g_hWnd = CreateWindowW(
        L"PathTracerMainWindow",
        L"Path Tracer 3D - Blender Viewport, Manifold Exploration Room & OBJ Importer",
        WS_OVERLAPPEDWINDOW,
        CW_USEDEFAULT, CW_USEDEFAULT,
        winWidth, winHeight,
        nullptr, nullptr, hInstance, nullptr
    );

    if (!g_hWnd) return 0;

    ShowWindow(g_hWnd, nCmdShow);
    UpdateWindow(g_hWnd);

    MSG msg;
    while (GetMessage(&msg, nullptr, 0, 0)) {
        if (msg.message == WM_KEYDOWN) {
            if (msg.wParam == 'G' || msg.wParam == 'g' ||
                msg.wParam == 'S' || msg.wParam == 's' ||
                msg.wParam == 'F' || msg.wParam == 'f' ||
                (g_is_grabbing && (msg.wParam == 'X' || msg.wParam == 'x' ||
                                   msg.wParam == 'Y' || msg.wParam == 'y' ||
                                   msg.wParam == 'Z' || msg.wParam == 'z' ||
                                   msg.wParam == VK_ESCAPE || msg.wParam == VK_RETURN)) ||
                (g_is_scaling  && (msg.wParam == VK_ESCAPE || msg.wParam == VK_RETURN))) {
                HWND hFocus = GetFocus();
                bool isEditing = (hFocus == g_hEditPosX || hFocus == g_hEditPosY || hFocus == g_hEditPosZ ||
                                  hFocus == g_hEditRadius || hFocus == g_hEditFilename || hFocus == g_hEditPhotons ||
                                  hFocus == g_hEditSPP || hFocus == g_hEditDepth || hFocus == g_hEditResW || hFocus == g_hEditResH ||
                                  hFocus == g_hEditMatR || hFocus == g_hEditMatG || hFocus == g_hEditMatB || hFocus == g_hEditMatParam);
                if (!isEditing || g_is_grabbing || g_is_scaling) {
                    SendMessage(g_hCanvas, WM_KEYDOWN, msg.wParam, msg.lParam);
                    continue;
                }
            }
        }
        TranslateMessage(&msg);
        DispatchMessage(&msg);
    }

    return (int)msg.wParam;
}
