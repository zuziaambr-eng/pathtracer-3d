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
#include <chrono>
#include <cmath>
#include <random>
#include <algorithm>
#include <cstdint>

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"

#pragma comment(lib, "comctl32.lib")

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

// Płaszczyzna ograniczająca półprzestrzeń dla wielościanu wypukłego: n . (p - center) <= d
struct PolyPlane {
    Vec3 n;
    float d;
};

// Szlifowany Kryształ / Wielościan Wypukły (Slab Method - analityczne obcinanie półprzestrzeni)
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
                    return false; // Promień jest równoległy i na zewnątrz płaszczyzny
                }
            } else if (denom > 0.0f) {
                // Promień wychodzi z półprzestrzeni
                float t = numer / denom;
                if (t < t_exit) {
                    t_exit = t;
                    n_exit = plane.n;
                }
            } else {
                // denom < 0.0f: Promień wchodzi do półprzestrzeni
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

// Tworzenie szlifowanego kryształu heksagonalnego z fasetami ostrosłupowymi na szczycie i ścięciami
std::shared_ptr<ConvexPolyhedron> CreateFacetedCrystal(const Vec3& center, float radius, float height, std::shared_ptr<Material> mat) {
    auto crystal = std::make_shared<ConvexPolyhedron>(center, mat);
    float half_h = height * 0.5f;

    // 1. Podstawa i szczyt
    crystal->add_plane(Vec3(0.0f, -1.0f, 0.0f), half_h);
    crystal->add_plane(Vec3(0.0f, 1.0f, 0.0f), half_h);

    // 2. Sześć ścian pionowych pryzmatu heksagonalnego
    for (int i = 0; i < 6; ++i) {
        float angle = i * (3.14159265f / 3.0f);
        crystal->add_plane(Vec3(std::cos(angle), 0.0f, std::sin(angle)), radius);
    }

    // 3. Sześć ukośnych faset wierzchołkowych (szlif diamentowy / ostrosłup kwarcu)
    for (int i = 0; i < 6; ++i) {
        float angle = (i + 0.5f) * (3.14159265f / 3.0f);
        Vec3 slanted_norm = unit_vector(Vec3(std::cos(angle) * 0.85f, 1.15f, std::sin(angle) * 0.85f));
        crystal->add_plane(slanted_norm, radius * 1.08f);
    }

    // 4. Sześć dolnych faset ściętych (fazowanie krawędzi podstawy)
    for (int i = 0; i < 6; ++i) {
        float angle = i * (3.14159265f / 3.0f);
        Vec3 slanted_norm = unit_vector(Vec3(std::cos(angle) * 0.80f, -1.25f, std::sin(angle) * 0.80f));
        crystal->add_plane(slanted_norm, radius * 1.18f);
    }

    return crystal;
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

// Standardowy Path Tracer (Forward PT)
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

// Dwukierunkowy Path Tracer (BDPT + Globalna Siatka Fotonów Kaustyki)
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

        // 1. Direct Next Event Estimation
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

        // 2. Kaustyka z Globalnej Mapy Fotonów
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
// 4. STRUKTURY DANYCH I ZMIENNE GLOBALNE
// ============================================================================

struct Tile {
    int x0, y0;
    int x1, y1;
};

struct EditableSceneObject {
    std::wstring name;
    Vec3 center;
    float radius;          // Promień lub skala kryształu
    float secondary_param; // Wysokość kryształu
    bool is_crystal = false;
    std::shared_ptr<Material> material;
};

constexpr int CANVAS_VIEW_WIDTH  = 800;
constexpr int CANVAS_VIEW_HEIGHT = 450;
constexpr int TIMER_UPDATE_ID    = 101;

// Identyfikatory kontrolek
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

// Aktualne parametry renderowania
int g_render_width   = 800;
int g_render_height  = 450;
int g_tile_size      = 32;
int g_render_algo    = 1;
int g_photon_count   = 50000;

// Bufory obrazu
std::vector<uint32_t> g_display_buffer(CANVAS_VIEW_WIDTH * CANVAS_VIEW_HEIGHT, 0xFF18181B);
std::vector<Vec3>     g_raw_image(g_render_width * g_render_height, Vec3(0, 0, 0));
std::vector<int>      g_sample_counts(g_render_width * g_render_height, 0);

// Flagi renderowania
std::atomic<bool>   g_is_rendering(false);
std::atomic<bool>   g_cancel_render(false);
std::atomic<size_t> g_completed_tiles(0);
size_t              g_total_tiles = 0;
std::chrono::high_resolution_clock::time_point g_render_start_time;

// Obiekty sceny ze SZLIFOWANYM KRYSZTAŁEM i dramatycznym kątem światła
std::vector<EditableSceneObject> g_scene_objects = {
    { L"1. Krysztal Szklany (Faceted Crystal)", Vec3(-0.5f, 0.70f, 0.1f), 0.65f, 1.40f, true,  std::make_shared<Dielectric>(1.55f) },
    { L"2. Zrodlo Swiatla (Kat boczno-tylny)",   Vec3(3.2f, 2.4f, -1.2f),   0.85f, 0.0f,  false, std::make_shared<DiffuseLight>(Vec3(35.0f, 32.0f, 28.0f)) },
    { L"3. Kula Zlota (Prawa)",                 Vec3(1.7f, 0.80f, 0.6f),   0.80f, 0.0f,  false, std::make_shared<Metal>(Vec3(0.95f, 0.78f, 0.35f), 0.02f) },
    { L"4. Kula Rubinowa (Lewa-Tyl)",           Vec3(-2.0f, 0.65f, -0.6f), 0.65f, 0.0f,  false, std::make_shared<Lambertian>(Vec3(0.85f, 0.15f, 0.15f)) },
    { L"5. Kula Chromowana (Przod)",            Vec3(0.7f, 0.35f, 1.7f),   0.35f, 0.0f,  false, std::make_shared<Metal>(Vec3(0.92f, 0.95f, 0.98f), 0.05f) }
};

// ============================================================================
// 5. BLENDER-STYLE INTERACTIVE 3D VIEWPORT & GRAB (G) SYSTEM
// ============================================================================

enum AxisLock {
    AXIS_NONE = 0,
    AXIS_X,
    AXIS_Y,
    AXIS_Z
};

int      g_selected_object_idx = 0; // Domyślnie wybrany obiekt 0 (kryształ)
bool     g_is_grabbing         = false;
Vec3     g_grab_initial_pos;
POINT    g_grab_start_mouse    = {0, 0};
Vec3     g_grab_plane_hit0;
Vec3     g_grab_plane_normal;
Vec3     g_grab_plane_point;
AxisLock g_axis_lock           = AXIS_NONE;

// Uchwyty okna
HWND g_hWnd          = nullptr;
HWND g_hCanvas       = nullptr;
HWND g_hBtnRender    = nullptr;
HWND g_hBtnStop      = nullptr;
HWND g_hBtnSave      = nullptr;
HWND g_hComboObjects = nullptr;
HWND g_hComboRes     = nullptr;
HWND g_hEditResW     = nullptr;
HWND g_hEditResH     = nullptr;
HWND g_hComboTile    = nullptr;
HWND g_hComboAlgo    = nullptr;
HWND g_hEditPhotons  = nullptr;
HWND g_hEditSPP      = nullptr;
HWND g_hEditDepth    = nullptr;
HWND g_hEditPosX     = nullptr;
HWND g_hEditPosY     = nullptr;
HWND g_hEditPosZ     = nullptr;
HWND g_hEditRadius   = nullptr;
HWND g_hBtnApplyPos  = nullptr;
HWND g_hBtnGrab      = nullptr;
HWND g_hEditFilename = nullptr;
HWND g_hProgressBar  = nullptr;
HWND g_hStatusLabel  = nullptr;

Camera GetCurrentCamera() {
    const float aspect_ratio = static_cast<float>(CANVAS_VIEW_WIDTH) / static_cast<float>(CANVAS_VIEW_HEIGHT);
    Vec3 lookfrom(0.2f, 2.5f, 4.3f);
    Vec3 lookat(-0.3f, 0.5f, 0.0f);
    Vec3 vup(0.0f, 1.0f, 0.0f);
    float vfov = 38.0f;
    return Camera(lookfrom, lookat, vup, vfov, aspect_ratio);
}

// Rzutowanie współrzędnych 3D na ekran 2D kanwy
bool ProjectToScreen(const Vec3& p, const Camera& cam, float viewport_w, float viewport_h,
                     int screen_w, int screen_h, int& out_x, int& out_y) {
    Vec3 v = p - cam.origin;
    Vec3 w_dir = unit_vector(cam.lower_left_corner + cam.horizontal * 0.5f + cam.vertical * 0.5f - cam.origin);
    float dz = dot(v, w_dir);
    if (dz <= 0.1f) return false;

    Vec3 u_dir = unit_vector(cam.horizontal);
    Vec3 v_dir = unit_vector(cam.vertical);

    float px = dot(v, u_dir) / dz;
    float py = dot(v, v_dir) / dz;

    float u = 0.5f + px / viewport_w;
    float v_norm = 0.5f + py / viewport_h;

    out_x = static_cast<int>(u * static_cast<float>(screen_w));
    out_y = static_cast<int>((1.0f - v_norm) * static_cast<float>(screen_h));
    return (out_x >= -200 && out_x <= screen_w + 200 && out_y >= -200 && out_y <= screen_h + 200);
}

// Ray Picking: Wykrywanie klikniętego obiektu z pozycji kursora myszki
int RayPickObject(int mx, int my) {
    if (mx < 0 || mx >= CANVAS_VIEW_WIDTH || my < 0 || my >= CANVAS_VIEW_HEIGHT) return -1;
    Camera cam = GetCurrentCamera();
    float u = (static_cast<float>(mx) + 0.5f) / static_cast<float>(CANVAS_VIEW_WIDTH);
    float v = (static_cast<float>(CANVAS_VIEW_HEIGHT - 1 - my) + 0.5f) / static_cast<float>(CANVAS_VIEW_HEIGHT);
    Ray r = cam.get_ray(u, v);

    int best_idx = -1;
    float closest_t = 1e9f;

    for (int i = 0; i < static_cast<int>(g_scene_objects.size()); ++i) {
        const auto& obj = g_scene_objects[i];
        HitRecord rec;
        if (obj.is_crystal) {
            float h = obj.secondary_param > 0.1f ? obj.secondary_param : 1.40f;
            auto crystal = CreateFacetedCrystal(obj.center, obj.radius, h, obj.material);
            if (crystal->hit(r, 0.001f, closest_t, rec)) {
                closest_t = rec.t;
                best_idx = i;
            }
        } else {
            Sphere s(obj.center, obj.radius, obj.material);
            if (s.hit(r, 0.001f, closest_t, rec)) {
                closest_t = rec.t;
                best_idx = i;
            }
        }
    }
    return best_idx;
}

// Błyskawiczny podgląd 3D sceny w czasie rzeczywistym (60+ FPS)
void RenderFastPreview() {
    if (g_is_rendering.load()) return;

    Camera cam = GetCurrentCamera();
    constexpr int PW = 400;
    constexpr int PH = 225;

    std::vector<uint32_t> small_buf(PW * PH);

    HittableList world;
    for (int i = 0; i < static_cast<int>(g_scene_objects.size()); ++i) {
        const auto& obj = g_scene_objects[i];
        if (obj.is_crystal) {
            float h = obj.secondary_param > 0.1f ? obj.secondary_param : 1.40f;
            world.add(CreateFacetedCrystal(obj.center, obj.radius, h, obj.material));
        } else {
            world.add(std::make_shared<Sphere>(obj.center, obj.radius, obj.material));
        }
    }
    // Podłoże
    world.add(std::make_shared<Sphere>(Vec3(0.0f, -1000.0f, 0.0f), 1000.0f,
                                       std::make_shared<Lambertian>(Vec3(0.85f, 0.85f, 0.88f))));

    Vec3 light_pos(3.2f, 2.4f, -1.2f);
    if (g_scene_objects.size() > 1) {
        light_pos = g_scene_objects[1].center;
    }

    for (int py = 0; py < PH; ++py) {
        for (int px = 0; px < PW; ++px) {
            float u = (static_cast<float>(px) + 0.5f) / static_cast<float>(PW);
            float v = (static_cast<float>(PH - 1 - py) + 0.5f) / static_cast<float>(PH);
            Ray r = cam.get_ray(u, v);

            HitRecord rec;
            Vec3 col;
            if (world.hit(r, 0.001f, 1e9f, rec)) {
                Vec3 to_light = light_pos - rec.p;
                float dist_l = to_light.length();
                Vec3 dir_l = to_light / dist_l;
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

            small_buf[py * PW + px] = (ib) | (ig << 8) | (ir << 16) | (0xFF << 24);
        }
    }

    if (g_display_buffer.size() != CANVAS_VIEW_WIDTH * CANVAS_VIEW_HEIGHT) {
        g_display_buffer.resize(CANVAS_VIEW_WIDTH * CANVAS_VIEW_HEIGHT);
    }
    for (int y = 0; y < CANVAS_VIEW_HEIGHT; ++y) {
        int sy = y / 2;
        if (sy >= PH) sy = PH - 1;
        int row_dst = y * CANVAS_VIEW_WIDTH;
        int row_src = sy * PW;
        for (int x = 0; x < CANVAS_VIEW_WIDTH; ++x) {
            int sx = x / 2;
            if (sx >= PW) sx = PW - 1;
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
    g_grab_initial_pos = g_scene_objects[g_selected_object_idx].center;
    g_axis_lock = AXIS_NONE;

    Camera cam = GetCurrentCamera();
    g_grab_plane_point = g_grab_initial_pos;
    g_grab_plane_normal = unit_vector(cam.origin - cam.lookat);

    POINT pt;
    GetCursorPos(&pt);
    ScreenToClient(g_hCanvas, &pt);
    g_grab_start_mouse = pt;

    float u0 = (static_cast<float>(pt.x) + 0.5f) / static_cast<float>(CANVAS_VIEW_WIDTH);
    float v0 = (static_cast<float>(CANVAS_VIEW_HEIGHT - 1 - pt.y) + 0.5f) / static_cast<float>(CANVAS_VIEW_HEIGHT);
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
    swprintf(status, 200, L"[BLENDER GRAB] Przesuwanie: %s | Ruch mysza | [X/Y/Z]: Blokada osi | [LPM/Enter]: Zatwierdz | [PPM/Esc]: Anuluj",
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
    SetWindowTextW(g_hStatusLabel, L"Anulowano przesuwanie (przywrocono poczatkowa pozycje).");

    RenderFastPreview();
    InvalidateRect(g_hCanvas, NULL, FALSE);
}

void ProcessGrabMove(int mx, int my) {
    if (!g_is_grabbing) return;

    Camera cam = GetCurrentCamera();
    float u = (static_cast<float>(mx) + 0.5f) / static_cast<float>(CANVAS_VIEW_WIDTH);
    float v = (static_cast<float>(CANVAS_VIEW_HEIGHT - 1 - my) + 0.5f) / static_cast<float>(CANVAS_VIEW_HEIGHT);
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
    SendMessage(g_hProgressBar, PBM_SETRANGE, 0, MAKELPARAM(0, g_total_tiles));
    SendMessage(g_hProgressBar, PBM_SETPOS, 0, 0);

    const wchar_t* algoName = (g_render_algo == 1) ? L"BDPT + Globalna Mapa Fotonow" : L"Standardowy PT";
    wchar_t statusInit[160];
    swprintf(statusInit, 160, L"Rozpoczynanie [%s]: %zu kafelkow %dx%d px (%d fotonow)...",
             algoName, g_total_tiles, g_tile_size, g_tile_size, g_photon_count);
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

        world.add(std::make_shared<Sphere>(Vec3(0.0f, -1000.0f, 0.0f), 1000.0f,
                                           std::make_shared<Lambertian>(Vec3(0.85f, 0.85f, 0.88f))));

        for (const auto& obj : g_scene_objects) {
            if (obj.is_crystal) {
                float h = obj.secondary_param > 0.1f ? obj.secondary_param : 1.40f;
                crystal_radius = obj.radius;
                glass_crystal = CreateFacetedCrystal(obj.center, obj.radius, h, obj.material);
                world.add(glass_crystal);
            } else {
                auto sphere_ptr = std::make_shared<Sphere>(obj.center, obj.radius, obj.material);
                world.add(sphere_ptr);
                if (!obj.material->emitted(obj.center).near_zero()) {
                    light_sources.push_back(*sphere_ptr);
                }
            }
        }

        const float aspect_ratio = static_cast<float>(g_render_width) / static_cast<float>(g_render_height);
        Vec3 lookfrom(0.2f, 2.5f, 4.3f);
        Vec3 lookat(-0.3f, 0.5f, 0.0f);
        Vec3 vup(0.0f, 1.0f, 0.0f);
        float vfov = 38.0f;
        Camera cam(lookfrom, lookat, vup, vfov, aspect_ratio);

        PhotonMap photon_map;
        if (algo == 1 && glass_crystal && !light_sources.empty()) {
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

                            float u = (static_cast<float>(px) + dist01(rng)) / static_cast<float>(g_render_width - 1);
                            float v = (static_cast<float>(py) + dist01(rng)) / static_cast<float>(g_render_height - 1);

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
    }).detach();
}

void StopRendering() {
    if (g_is_rendering.load()) {
        g_cancel_render.store(true);
        SetWindowTextW(g_hStatusLabel, L"Zatrzymywanie watkow kafelkowania...");
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
        int src_row = g_render_height - 1 - j;
        for (int i = 0; i < g_render_width; ++i) {
            int p_idx = src_row * g_render_width + i;
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
// 7. PROCEDURA OBSŁUGI OKNA KANWY (PODGLĄD 3D & BLENDER GRAB)
// ============================================================================

LRESULT CALLBACK CanvasWndProc(HWND hWnd, UINT msg, WPARAM wParam, LPARAM lParam) {
    switch (msg) {
        case WM_LBUTTONDOWN: {
            int mx = LOWORD(lParam);
            int my = HIWORD(lParam);
            SetFocus(hWnd);

            if (g_is_grabbing) {
                ConfirmGrabMode();
            } else {
                int hit_idx = RayPickObject(mx, my);
                if (hit_idx >= 0) {
                    g_selected_object_idx = hit_idx;
                    SendMessage(g_hComboObjects, CB_SETCURSEL, hit_idx, 0);
                    UpdateObjectFieldsFromSelection(hit_idx);

                    wchar_t status[180];
                    swprintf(status, 180, L"Wybrano: [%s]. Wcisnij 'G', aby przemieszczac myszka w 3D.",
                             g_scene_objects[hit_idx].name.c_str());
                    SetWindowTextW(g_hStatusLabel, status);

                    RenderFastPreview();
                    InvalidateRect(hWnd, NULL, FALSE);
                }
            }
            return 0;
        }

        case WM_RBUTTONDOWN: {
            if (g_is_grabbing) {
                CancelGrabMode();
            }
            return 0;
        }

        case WM_MOUSEMOVE: {
            int mx = LOWORD(lParam);
            int my = HIWORD(lParam);

            if (g_is_grabbing) {
                ProcessGrabMove(mx, my);
            } else {
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
            }
            break;
        }

        case WM_KEYDOWN: {
            if (wParam == 'G' || wParam == 'g') {
                if (!g_is_grabbing) {
                    StartGrabMode();
                } else {
                    ConfirmGrabMode();
                }
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
            break;
        }

        case WM_PAINT: {
            PAINTSTRUCT ps;
            HDC hdc = BeginPaint(hWnd, &ps);

            // 1. Podgląd bufora 3D
            if (!g_display_buffer.empty()) {
                BITMAPINFO bmi = {};
                bmi.bmiHeader.biSize = sizeof(BITMAPINFOHEADER);
                bmi.bmiHeader.biWidth = g_render_width;
                bmi.bmiHeader.biHeight = g_render_height;
                bmi.bmiHeader.biPlanes = 1;
                bmi.bmiHeader.biBitCount = 32;
                bmi.bmiHeader.biCompression = BI_RGB;

                SetStretchBltMode(hdc, HALFTONE);
                SetBrushOrgEx(hdc, 0, 0, NULL);

                StretchDIBits(
                    hdc,
                    0, 0, CANVAS_VIEW_WIDTH, CANVAS_VIEW_HEIGHT,
                    0, 0, g_render_width, g_render_height,
                    g_display_buffer.data(),
                    &bmi,
                    DIB_RGB_COLORS,
                    SRCCOPY
                );
            }

            // 2. Nakładki 3D (Wskaźnik zaznaczenia obiektu, osie X/Y/Z i linie pomocnicze)
            if (!g_is_rendering.load() && g_selected_object_idx >= 0 && g_selected_object_idx < static_cast<int>(g_scene_objects.size())) {
                Camera cam = GetCurrentCamera();
                const auto& sel_obj = g_scene_objects[g_selected_object_idx];

                int sx = 0, sy = 0;
                float v_w = 2.0f * std::tan(38.0f * 3.14159f / 360.0f) * (static_cast<float>(CANVAS_VIEW_WIDTH) / CANVAS_VIEW_HEIGHT);
                float v_h = 2.0f * std::tan(38.0f * 3.14159f / 360.0f);

                if (ProjectToScreen(sel_obj.center, cam, v_w, v_h, CANVAS_VIEW_WIDTH, CANVAS_VIEW_HEIGHT, sx, sy)) {
                    if (g_is_grabbing) {
                        // Rysowanie zablokowanej osi w 3D (Czerwona X, Zielona Y, Niebieska Z)
                        HPEN hPenAxis = nullptr;
                        Vec3 pA, pB;
                        if (g_axis_lock == AXIS_X) {
                            hPenAxis = CreatePen(PS_SOLID, 2, RGB(255, 60, 60));
                            pA = sel_obj.center + Vec3(-15.0f, 0.0f, 0.0f);
                            pB = sel_obj.center + Vec3( 15.0f, 0.0f, 0.0f);
                        } else if (g_axis_lock == AXIS_Y) {
                            hPenAxis = CreatePen(PS_SOLID, 2, RGB(60, 255, 60));
                            pA = sel_obj.center + Vec3(0.0f, -15.0f, 0.0f);
                            pB = sel_obj.center + Vec3(0.0f,  15.0f, 0.0f);
                        } else if (g_axis_lock == AXIS_Z) {
                            hPenAxis = CreatePen(PS_SOLID, 2, RGB(60, 140, 255));
                            pA = sel_obj.center + Vec3(0.0f, 0.0f, -15.0f);
                            pB = sel_obj.center + Vec3(0.0f, 0.0f,  15.0f);
                        }

                        if (hPenAxis) {
                            HPEN hOldPen = (HPEN)SelectObject(hdc, hPenAxis);
                            int ax1 = 0, ay1 = 0, ax2 = 0, ay2 = 0;
                            if (ProjectToScreen(pA, cam, v_w, v_h, CANVAS_VIEW_WIDTH, CANVAS_VIEW_HEIGHT, ax1, ay1) &&
                                ProjectToScreen(pB, cam, v_w, v_h, CANVAS_VIEW_WIDTH, CANVAS_VIEW_HEIGHT, ax2, ay2)) {
                                MoveToEx(hdc, ax1, ay1, NULL);
                                LineTo(hdc, ax2, ay2);
                            }
                            SelectObject(hdc, hOldPen);
                            DeleteObject(hPenAxis);
                        }

                        // Kropkowana linia łącząca punkt startowy z aktualną pozycją
                        int orig_sx = 0, orig_sy = 0;
                        if (ProjectToScreen(g_grab_initial_pos, cam, v_w, v_h, CANVAS_VIEW_WIDTH, CANVAS_VIEW_HEIGHT, orig_sx, orig_sy)) {
                            HPEN hPenDash = CreatePen(PS_DOT, 1, RGB(220, 220, 220));
                            HPEN hOldPen = (HPEN)SelectObject(hdc, hPenDash);
                            MoveToEx(hdc, orig_sx, orig_sy, NULL);
                            LineTo(hdc, sx, sy);
                            SelectObject(hdc, hOldPen);
                            DeleteObject(hPenDash);
                        }
                    }

                    // Pierścień zaznaczenia obiektu (pomarańczowy w stylu Blendera)
                    HPEN hPenOrange = CreatePen(PS_SOLID, 2, g_is_grabbing ? RGB(255, 230, 20) : RGB(255, 140, 0));
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

                RECT hudRect = { 10, CANVAS_VIEW_HEIGHT - 30, CANVAS_VIEW_WIDTH - 10, CANVAS_VIEW_HEIGHT - 6 };
                HBRUSH hBgBrush = CreateSolidBrush(RGB(18, 22, 28));
                FillRect(hdc, &hudRect, hBgBrush);
                DeleteObject(hBgBrush);

                wchar_t hudText[220];
                if (g_is_grabbing) {
                    const wchar_t* axisName = L"Swobodny";
                    if (g_axis_lock == AXIS_X) axisName = L"Os X (Czerwona)";
                    else if (g_axis_lock == AXIS_Y) axisName = L"Os Y (Zielona)";
                    else if (g_axis_lock == AXIS_Z) axisName = L"Os Z (Niebieska)";

                    swprintf(hudText, 220, L" ✋ [BLENDER GRAB] %s | Os: %s | [X/Y/Z]: Zmien os | [LPM/Enter]: Zatwierdz | [PPM/Esc]: Anuluj",
                             g_scene_objects[g_selected_object_idx].name.c_str(), axisName);
                    SetTextColor(hdc, RGB(255, 215, 0));
                } else {
                    swprintf(hudText, 220, L" 🎮 [PODGLAD 3D] Kliknij obiekt LPM, aby wybrac | Klawisz [G]: Przesuwaj (Blender) | Wybrany: %s",
                             (g_selected_object_idx >= 0 ? g_scene_objects[g_selected_object_idx].name.c_str() : L"Brak"));
                    SetTextColor(hdc, RGB(210, 225, 245));
                }

                DrawTextW(hdc, hudText, -1, &hudRect, DT_SINGLELINE | DT_VCENTER | DT_LEFT);
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
// 8. GŁÓWNA PROCEDURA OKNA
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
                15, 15, CANVAS_VIEW_WIDTH, CANVAS_VIEW_HEIGHT,
                hWnd, nullptr, GetModuleHandle(NULL), nullptr
            );

            HFONT hFont = (HFONT)GetStockObject(DEFAULT_GUI_FONT);
            int sideX = 835;
            int curY = 15;

            // --- GRUPA 1: PARAMETRY RENDEROWANIA & KAUSTYKA ---
            HWND hGroupRender = CreateWindowW(L"BUTTON", L" Parametry renderowania & Kaustyka ",
                WS_CHILD | WS_VISIBLE | BS_GROUPBOX,
                sideX, curY, 380, 240, hWnd, nullptr, nullptr, nullptr);
            SendMessage(hGroupRender, WM_SETFONT, (WPARAM)hFont, TRUE);

            curY += 22;

            HWND hLblAlgo = CreateWindowW(L"STATIC", L"Algorytm PT:", WS_CHILD | WS_VISIBLE, sideX + 15, curY, 95, 20, hWnd, nullptr, nullptr, nullptr);
            SendMessage(hLblAlgo, WM_SETFONT, (WPARAM)hFont, TRUE);

            g_hComboAlgo = CreateWindowW(L"COMBOBOX", nullptr, WS_CHILD | WS_VISIBLE | CBS_DROPDOWNLIST,
                sideX + 115, curY - 2, 250, 150, hWnd, (HMENU)IDC_COMBO_ALGO, nullptr, nullptr);
            SendMessage(g_hComboAlgo, WM_SETFONT, (WPARAM)hFont, TRUE);
            SendMessage(g_hComboAlgo, CB_ADDSTRING, 0, (LPARAM)L"Standardowy Path Tracing (PT)");
            SendMessage(g_hComboAlgo, CB_ADDSTRING, 0, (LPARAM)L"BDPT (Globalna Mapa Fotonow - Krysztal)");
            SendMessage(g_hComboAlgo, CB_SETCURSEL, 1, 0);

            curY += 27;

            HWND hLblPhotons = CreateWindowW(L"STATIC", L"Fotony kaustyki:", WS_CHILD | WS_VISIBLE, sideX + 15, curY, 100, 20, hWnd, nullptr, nullptr, nullptr);
            SendMessage(hLblPhotons, WM_SETFONT, (WPARAM)hFont, TRUE);

            g_hEditPhotons = CreateWindowExW(WS_EX_CLIENTEDGE, L"EDIT", L"50000", WS_CHILD | WS_VISIBLE | ES_NUMBER,
                sideX + 115, curY - 2, 85, 22, hWnd, (HMENU)IDC_EDIT_PHOTONS, nullptr, nullptr);
            SendMessage(g_hEditPhotons, WM_SETFONT, (WPARAM)hFont, TRUE);

            HWND hLblPhotonsHint = CreateWindowW(L"STATIC", L"(np. 20000 - 150000)", WS_CHILD | WS_VISIBLE, sideX + 210, curY, 155, 20, hWnd, nullptr, nullptr, nullptr);
            SendMessage(hLblPhotonsHint, WM_SETFONT, (WPARAM)hFont, TRUE);

            curY += 27;

            HWND hLblRes = CreateWindowW(L"STATIC", L"Rozdzielczosc:", WS_CHILD | WS_VISIBLE, sideX + 15, curY, 95, 20, hWnd, nullptr, nullptr, nullptr);
            SendMessage(hLblRes, WM_SETFONT, (WPARAM)hFont, TRUE);

            g_hComboRes = CreateWindowW(L"COMBOBOX", nullptr, WS_CHILD | WS_VISIBLE | CBS_DROPDOWNLIST | WS_VSCROLL,
                sideX + 115, curY - 2, 250, 200, hWnd, (HMENU)IDC_COMBO_RES, nullptr, nullptr);
            SendMessage(g_hComboRes, WM_SETFONT, (WPARAM)hFont, TRUE);
            SendMessage(g_hComboRes, CB_ADDSTRING, 0, (LPARAM)L"640 x 360  (Bardzo szybki podglad)");
            SendMessage(g_hComboRes, CB_ADDSTRING, 0, (LPARAM)L"800 x 450  (Standardowy podglad)");
            SendMessage(g_hComboRes, CB_ADDSTRING, 0, (LPARAM)L"960 x 540  (qHD)");
            SendMessage(g_hComboRes, CB_ADDSTRING, 0, (LPARAM)L"1280 x 720 (HD 720p)");
            SendMessage(g_hComboRes, CB_ADDSTRING, 0, (LPARAM)L"1920 x 1080 (Full HD 1080p)");
            SendMessage(g_hComboRes, CB_ADDSTRING, 0, (LPARAM)L"Niestandardowa (wpisz ponizej)");
            SendMessage(g_hComboRes, CB_SETCURSEL, 1, 0);

            curY += 27;

            HWND hLblW = CreateWindowW(L"STATIC", L"Szerokosc W:", WS_CHILD | WS_VISIBLE, sideX + 15, curY, 80, 20, hWnd, nullptr, nullptr, nullptr);
            SendMessage(hLblW, WM_SETFONT, (WPARAM)hFont, TRUE);
            g_hEditResW = CreateWindowExW(WS_EX_CLIENTEDGE, L"EDIT", L"800", WS_CHILD | WS_VISIBLE | ES_NUMBER, sideX + 100, curY - 2, 65, 22, hWnd, (HMENU)IDC_EDIT_RES_W, nullptr, nullptr);
            SendMessage(g_hEditResW, WM_SETFONT, (WPARAM)hFont, TRUE);

            HWND hLblH = CreateWindowW(L"STATIC", L"Wysokosc H:", WS_CHILD | WS_VISIBLE, sideX + 190, curY, 75, 20, hWnd, nullptr, nullptr, nullptr);
            SendMessage(hLblH, WM_SETFONT, (WPARAM)hFont, TRUE);
            g_hEditResH = CreateWindowExW(WS_EX_CLIENTEDGE, L"EDIT", L"450", WS_CHILD | WS_VISIBLE | ES_NUMBER, sideX + 270, curY - 2, 65, 22, hWnd, (HMENU)IDC_EDIT_RES_H, nullptr, nullptr);
            SendMessage(g_hEditResH, WM_SETFONT, (WPARAM)hFont, TRUE);

            curY += 27;

            HWND hLblTile = CreateWindowW(L"STATIC", L"Kafelek (Tiling):", WS_CHILD | WS_VISIBLE, sideX + 15, curY, 95, 20, hWnd, nullptr, nullptr, nullptr);
            SendMessage(hLblTile, WM_SETFONT, (WPARAM)hFont, TRUE);

            g_hComboTile = CreateWindowW(L"COMBOBOX", nullptr, WS_CHILD | WS_VISIBLE | CBS_DROPDOWNLIST,
                sideX + 115, curY - 2, 250, 150, hWnd, (HMENU)IDC_COMBO_TILE, nullptr, nullptr);
            SendMessage(g_hComboTile, WM_SETFONT, (WPARAM)hFont, TRUE);
            SendMessage(g_hComboTile, CB_ADDSTRING, 0, (LPARAM)L"16 x 16 px (Drobny - bez szwow)");
            SendMessage(g_hComboTile, CB_ADDSTRING, 0, (LPARAM)L"32 x 32 px (Standardowy - optymalny)");
            SendMessage(g_hComboTile, CB_ADDSTRING, 0, (LPARAM)L"64 x 64 px (Duze kafelki)");
            SendMessage(g_hComboTile, CB_SETCURSEL, 0, 0);

            curY += 27;

            HWND hLblSPP = CreateWindowW(L"STATIC", L"Probki (SPP):", WS_CHILD | WS_VISIBLE, sideX + 15, curY, 80, 20, hWnd, nullptr, nullptr, nullptr);
            SendMessage(hLblSPP, WM_SETFONT, (WPARAM)hFont, TRUE);
            g_hEditSPP = CreateWindowExW(WS_EX_CLIENTEDGE, L"EDIT", L"64", WS_CHILD | WS_VISIBLE | ES_NUMBER, sideX + 100, curY - 2, 65, 22, hWnd, (HMENU)IDC_EDIT_SPP, nullptr, nullptr);
            SendMessage(g_hEditSPP, WM_SETFONT, (WPARAM)hFont, TRUE);

            HWND hLblDepth = CreateWindowW(L"STATIC", L"Maks. odbic:", WS_CHILD | WS_VISIBLE, sideX + 190, curY, 75, 20, hWnd, nullptr, nullptr, nullptr);
            SendMessage(hLblDepth, WM_SETFONT, (WPARAM)hFont, TRUE);
            g_hEditDepth = CreateWindowExW(WS_EX_CLIENTEDGE, L"EDIT", L"16", WS_CHILD | WS_VISIBLE | ES_NUMBER, sideX + 270, curY - 2, 65, 22, hWnd, (HMENU)IDC_EDIT_DEPTH, nullptr, nullptr);
            SendMessage(g_hEditDepth, WM_SETFONT, (WPARAM)hFont, TRUE);

            curY += 32;

            g_hBtnRender = CreateWindowW(L"BUTTON", L"▶ Rozpocznij renderowanie", WS_CHILD | WS_VISIBLE | BS_PUSHBUTTON,
                sideX + 15, curY, 195, 34, hWnd, (HMENU)IDC_BTN_RENDER, nullptr, nullptr);
            SendMessage(g_hBtnRender, WM_SETFONT, (WPARAM)hFont, TRUE);

            g_hBtnStop = CreateWindowW(L"BUTTON", L"⏹ Przerwij", WS_CHILD | WS_VISIBLE | BS_PUSHBUTTON | WS_DISABLED,
                sideX + 220, curY, 145, 34, hWnd, (HMENU)IDC_BTN_STOP, nullptr, nullptr);
            SendMessage(g_hBtnStop, WM_SETFONT, (WPARAM)hFont, TRUE);

            curY += 50;

            // --- GRUPA 2: PRZESUWANIE PRZEDMIOTÓW (BLENDER INTERFACE) ---
            HWND hGroupObj = CreateWindowW(L"BUTTON", L" Przesuwanie przedmiotow (Blender 3D) ",
                WS_CHILD | WS_VISIBLE | BS_GROUPBOX,
                sideX, curY, 380, 195, hWnd, nullptr, nullptr, nullptr);
            SendMessage(hGroupObj, WM_SETFONT, (WPARAM)hFont, TRUE);

            curY += 22;
            HWND hLblObj = CreateWindowW(L"STATIC", L"Wybierz obiekt:", WS_CHILD | WS_VISIBLE, sideX + 15, curY, 110, 20, hWnd, nullptr, nullptr, nullptr);
            SendMessage(hLblObj, WM_SETFONT, (WPARAM)hFont, TRUE);

            g_hComboObjects = CreateWindowW(L"COMBOBOX", nullptr, WS_CHILD | WS_VISIBLE | CBS_DROPDOWNLIST | WS_VSCROLL,
                sideX + 15, curY + 20, 350, 200, hWnd, (HMENU)IDC_COMBO_OBJECTS, nullptr, nullptr);
            SendMessage(g_hComboObjects, WM_SETFONT, (WPARAM)hFont, TRUE);

            for (const auto& obj : g_scene_objects) {
                SendMessage(g_hComboObjects, CB_ADDSTRING, 0, (LPARAM)obj.name.c_str());
            }
            SendMessage(g_hComboObjects, CB_SETCURSEL, 0, 0);

            curY += 48;

            auto CreateCoordinateRow = [&](const wchar_t* labelText, HWND& outEdit, int yPos) {
                HWND hLbl = CreateWindowW(L"STATIC", labelText, WS_CHILD | WS_VISIBLE, sideX + 15, yPos, 75, 20, hWnd, nullptr, nullptr, nullptr);
                SendMessage(hLbl, WM_SETFONT, (WPARAM)hFont, TRUE);
                outEdit = CreateWindowExW(WS_EX_CLIENTEDGE, L"EDIT", L"0.00", WS_CHILD | WS_VISIBLE, sideX + 90, yPos - 2, 75, 22, hWnd, nullptr, nullptr, nullptr);
                SendMessage(outEdit, WM_SETFONT, (WPARAM)hFont, TRUE);
            };

            CreateCoordinateRow(L"Pozycja X:", g_hEditPosX, curY);
            CreateCoordinateRow(L"Pozycja Y:", g_hEditPosY, curY + 25);
            CreateCoordinateRow(L"Pozycja Z:", g_hEditPosZ, curY + 50);

            HWND hLblR = CreateWindowW(L"STATIC", L"Promien R:", WS_CHILD | WS_VISIBLE, sideX + 185, curY, 70, 20, hWnd, nullptr, nullptr, nullptr);
            SendMessage(hLblR, WM_SETFONT, (WPARAM)hFont, TRUE);
            g_hEditRadius = CreateWindowExW(WS_EX_CLIENTEDGE, L"EDIT", L"0.65", WS_CHILD | WS_VISIBLE, sideX + 260, curY - 2, 80, 22, hWnd, nullptr, nullptr, nullptr);
            SendMessage(g_hEditRadius, WM_SETFONT, (WPARAM)hFont, TRUE);

            g_hBtnApplyPos = CreateWindowW(L"BUTTON", L"Zastosuj recznie", WS_CHILD | WS_VISIBLE | BS_PUSHBUTTON,
                sideX + 185, curY + 24, 155, 28, hWnd, (HMENU)IDC_BTN_APPLY_POS, nullptr, nullptr);
            SendMessage(g_hBtnApplyPos, WM_SETFONT, (WPARAM)hFont, TRUE);

            g_hBtnGrab = CreateWindowW(L"BUTTON", L"✋ Przesun w 3D (Klawisz G)", WS_CHILD | WS_VISIBLE | BS_PUSHBUTTON,
                sideX + 185, curY + 54, 155, 30, hWnd, (HMENU)IDC_BTN_GRAB, nullptr, nullptr);
            SendMessage(g_hBtnGrab, WM_SETFONT, (WPARAM)hFont, TRUE);

            UpdateObjectFieldsFromSelection(0);

            curY += 92;

            // --- GRUPA 3: ZAPIS DO PLIKU PNG ---
            HWND hGroupSave = CreateWindowW(L"BUTTON", L" Zapis obrazu do pliku PNG ",
                WS_CHILD | WS_VISIBLE | BS_GROUPBOX,
                sideX, curY, 380, 85, hWnd, nullptr, nullptr, nullptr);
            SendMessage(hGroupSave, WM_SETFONT, (WPARAM)hFont, TRUE);

            curY += 20;

            HWND hLblFile = CreateWindowW(L"STATIC", L"Nazwa pliku:", WS_CHILD | WS_VISIBLE, sideX + 15, curY, 80, 20, hWnd, nullptr, nullptr, nullptr);
            SendMessage(hLblFile, WM_SETFONT, (WPARAM)hFont, TRUE);

            g_hEditFilename = CreateWindowExW(WS_EX_CLIENTEDGE, L"EDIT", L"render_kaustyka.png", WS_CHILD | WS_VISIBLE,
                sideX + 100, curY - 2, 260, 22, hWnd, (HMENU)IDC_EDIT_FILENAME, nullptr, nullptr);
            SendMessage(g_hEditFilename, WM_SETFONT, (WPARAM)hFont, TRUE);

            curY += 26;

            g_hBtnSave = CreateWindowW(L"BUTTON", L"💾 Zapisz obraz PNG",
                WS_CHILD | WS_VISIBLE | BS_PUSHBUTTON,
                sideX + 15, curY, 350, 30, hWnd, (HMENU)IDC_BTN_SAVE_PNG, nullptr, nullptr);
            SendMessage(g_hBtnSave, WM_SETFONT, (WPARAM)hFont, TRUE);

            g_hProgressBar = CreateWindowExW(0, PROGRESS_CLASSW, nullptr,
                WS_CHILD | WS_VISIBLE | PBS_SMOOTH,
                15, 480, CANVAS_VIEW_WIDTH, 22, hWnd, (HMENU)IDC_PROGRESS_BAR, nullptr, nullptr);

            g_hStatusLabel = CreateWindowW(L"STATIC", L"Gotowy. Kliknij obiekt w kadrze LPM i wcisnij 'G', aby plynnie przesuwac w 3D (styl Blender).",
                WS_CHILD | WS_VISIBLE,
                15, 510, CANVAS_VIEW_WIDTH + 390, 30, hWnd, (HMENU)IDC_STATUS_LABEL, nullptr, nullptr);
            SendMessage(g_hStatusLabel, WM_SETFONT, (WPARAM)hFont, TRUE);

            RenderFastPreview();
            SetTimer(hWnd, TIMER_UPDATE_ID, 33, nullptr);
            break;
        }

        case WM_TIMER: {
            if (wParam == TIMER_UPDATE_ID) {
                if (g_is_rendering.load()) {
                    size_t done = g_completed_tiles.load();
                    SendMessage(g_hProgressBar, PBM_SETPOS, done, 0);

                    auto now = std::chrono::high_resolution_clock::now();
                    std::chrono::duration<double> elapsed = now - g_render_start_time;

                    int percent = (g_total_tiles > 0) ? static_cast<int>(done * 100 / g_total_tiles) : 0;
                    const wchar_t* modeTag = (g_render_algo == 1) ? L"BDPT + Mapa Fotonow" : L"PT";

                    wchar_t status[180];
                    swprintf(status, 180, L"Renderowanie [%s]... %d%% (%zu/%zu kafelkow) [Czas: %.1f s, Fotony: %d]",
                             modeTag, percent, done, g_total_tiles, elapsed.count(), g_photon_count);
                    SetWindowTextW(g_hStatusLabel, status);

                    InvalidateRect(g_hCanvas, NULL, FALSE);
                } else {
                    if (!IsWindowEnabled(g_hBtnRender)) {
                        EnableWindow(g_hBtnRender, TRUE);
                        EnableWindow(g_hBtnStop, FALSE);
                        EnableWindow(g_hBtnApplyPos, TRUE);
                        EnableWindow(g_hBtnGrab, TRUE);

                        auto now = std::chrono::high_resolution_clock::now();
                        std::chrono::duration<double> elapsed = now - g_render_start_time;

                        size_t done = g_completed_tiles.load();
                        SendMessage(g_hProgressBar, PBM_SETPOS, done, 0);

                        const wchar_t* modeTag = (g_render_algo == 1) ? L"BDPT" : L"PT";
                        wchar_t status[180];
                        if (g_cancel_render.load()) {
                            swprintf(status, 180, L"[%s] Renderowanie przerwane na kafelku %zu/%zu.", modeTag, done, g_total_tiles);
                        } else {
                            swprintf(status, 180, L"[%s] Renderowanie ukonczone w %.2f s! Rozdzielczosc: %dx%d px.",
                                     modeTag, elapsed.count(), g_render_width, g_render_height);
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

            if (wmId == IDC_COMBO_RES && wmEvent == CBN_SELCHANGE) {
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
                if (!g_is_grabbing) {
                    StartGrabMode();
                } else {
                    ConfirmGrabMode();
                }
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
// 9. PUNKT WEJŚCIA PROGRAMU (WinMain)
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

    int winWidth  = 1255;
    int winHeight = 595;

    g_hWnd = CreateWindowW(
        L"PathTracerMainWindow",
        L"Path Tracer 3D - Interaktywny Podglad 3D & Przesuwanie Obiektow (Blender G)",
        WS_OVERLAPPED | WS_CAPTION | WS_SYSMENU | WS_MINIMIZEBOX,
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
                (g_is_grabbing && (msg.wParam == 'X' || msg.wParam == 'x' ||
                                   msg.wParam == 'Y' || msg.wParam == 'y' ||
                                   msg.wParam == 'Z' || msg.wParam == 'z' ||
                                   msg.wParam == VK_ESCAPE || msg.wParam == VK_RETURN))) {
                HWND hFocus = GetFocus();
                bool isEditing = (hFocus == g_hEditPosX || hFocus == g_hEditPosY || hFocus == g_hEditPosZ ||
                                  hFocus == g_hEditRadius || hFocus == g_hEditFilename || hFocus == g_hEditPhotons ||
                                  hFocus == g_hEditSPP || hFocus == g_hEditDepth || hFocus == g_hEditResW || hFocus == g_hEditResH);
                if (!isEditing || g_is_grabbing) {
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
