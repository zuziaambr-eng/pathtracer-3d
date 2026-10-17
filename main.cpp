#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>
#include <memory>
#include <random>
#include <thread>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <algorithm>
#include <string>

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"

// ============================================================================
// 1. WEKTORY I MATEMATYKA (Vec3)
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
inline Vec3 unit_vector(const Vec3& v) { return v / v.length(); }

// Losowanie wektora jednostkowego na sferze
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

// Odbicie wektora v względem normalnej n
inline Vec3 reflect(const Vec3& v, const Vec3& n) {
    return v - 2.0f * dot(v, n) * n;
}

// Załamanie wektora (Prawo Snella)
inline Vec3 refract(const Vec3& uv, const Vec3& n, float etai_over_etat) {
    float cos_theta = std::fmin(dot(-uv, n), 1.0f);
    Vec3 r_out_perp = etai_over_etat * (uv + cos_theta * n);
    Vec3 r_out_parallel = -std::sqrt(std::fabs(1.0f - r_out_perp.length_squared())) * n;
    return r_out_perp + r_out_parallel;
}

// Przybliżenie Schlicka dla odbicia Fresnela
inline float schlick_reflectance(float cosine, float ref_idx) {
    float r0 = (1.0f - ref_idx) / (1.0f + ref_idx);
    r0 = r0 * r0;
    return r0 + (1.0f - r0) * std::pow(1.0f - cosine, 5.0f);
}

// ============================================================================
// 2. PROMIEŃ (Ray) ORAZ PUNKT PRZECIĘCIA (HitRecord)
// ============================================================================

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

// ============================================================================
// 3. MATERIAŁY (Lambertian, Metal, Szkło, Światło)
// ============================================================================

class Material {
public:
    virtual ~Material() = default;

    // Rozproszenie promienia
    virtual bool scatter(const Ray& r_in, const HitRecord& rec, Vec3& attenuation, Ray& scattered, std::mt19937& rng) const {
        return false;
    }

    // Emisja światła (dla źródeł światła)
    virtual Vec3 emitted(const Vec3& p) const {
        return Vec3(0, 0, 0);
    }
};

// Materiał matowy (dyfuzyjny - prawo Lamberta)
class Lambertian : public Material {
public:
    Vec3 albedo;
    explicit Lambertian(const Vec3& a) : albedo(a) {}

    bool scatter(const Ray& r_in, const HitRecord& rec, Vec3& attenuation, Ray& scattered, std::mt19937& rng) const override {
        Vec3 scatter_direction = rec.normal + random_unit_vector(rng);
        if (scatter_direction.near_zero()) {
            scatter_direction = rec.normal;
        }
        scattered = Ray(rec.p, unit_vector(scatter_direction));
        attenuation = albedo;
        return true;
    }
};

// Materiał metaliczny (odbicie lustrzane z chropowatością)
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
};

// Materiał przezroczysty / dielektryk (szkło, woda)
class Dielectric : public Material {
public:
    float ir; // Współczynnik załamania światła (np. 1.5 dla szkła)
    explicit Dielectric(float index_of_refraction) : ir(index_of_refraction) {}

    bool scatter(const Ray& r_in, const HitRecord& rec, Vec3& attenuation, Ray& scattered, std::mt19937& rng) const override {
        attenuation = Vec3(1.0f, 1.0f, 1.0f);
        float refraction_ratio = rec.front_face ? (1.0f / ir) : ir;

        Vec3 unit_direction = unit_vector(r_in.direction);
        float cos_theta = std::fmin(dot(-unit_direction, rec.normal), 1.0f);
        float sin_theta = std::sqrt(1.0f - cos_theta * cos_theta);

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
};

// Emiter światła (np. jarząca się kula)
class DiffuseLight : public Material {
public:
    Vec3 emit_color;
    explicit DiffuseLight(const Vec3& c) : emit_color(c) {}

    Vec3 emitted(const Vec3& p) const override {
        return emit_color;
    }
};

// ============================================================================
// 4. OBIEKTY GEOMETRYCZNE I SCENA (Sphere, HittableList)
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

        // Znajdź najbliższy pierwiastek w dopuszczalnym zakresie [t_min, t_max]
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

class HittableList : public Hittable {
public:
    std::vector<std::shared_ptr<Hittable>> objects;

    void add(std::shared_ptr<Hittable> object) {
        objects.push_back(std::move(object));
    }

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

// ============================================================================
// 5. KAMERA (Camera)
// ============================================================================

class Camera {
public:
    Vec3 origin;
    Vec3 lower_left_corner;
    Vec3 horizontal;
    Vec3 vertical;

    Camera(Vec3 lookfrom, Vec3 lookat, Vec3 vup, float vfov_deg, float aspect_ratio) {
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
// 6. GŁÓWNA PĘTLA PATH TRACINGU (ray_color)
// ============================================================================

Vec3 ray_color(const Ray& r, const Hittable& world, int depth, std::mt19937& rng) {
    if (depth <= 0) {
        return Vec3(0, 0, 0); // Limit odbić
    }

    HitRecord rec;
    // 0.001f zapobiega "shadow acne" (samoprzecięciu z powodu precyzji zmiennoprzecinkowej)
    if (!world.hit(r, 0.001f, 1e9f, rec)) {
        // Tło sceny: delikatny gradient nieba
        Vec3 unit_direction = unit_vector(r.direction);
        float a = 0.5f * (unit_direction.y + 1.0f);
        return (1.0f - a) * Vec3(0.04f, 0.04f, 0.06f) + a * Vec3(0.12f, 0.18f, 0.28f);
    }

    Vec3 emitted = rec.mat->emitted(rec.p);
    Ray scattered;
    Vec3 attenuation;

    // Jeżeli materiał nie rozprasza (np. jest tylko źródłem światła)
    if (!rec.mat->scatter(r, rec, attenuation, scattered, rng)) {
        return emitted;
    }

    // Równanie renderowania (Monte Carlo Path Tracing)
    return emitted + attenuation * ray_color(scattered, world, depth - 1, rng);
}

// ============================================================================
// 7. ZAPIS DO PLIKÓW (BMP oraz PPM)
// ============================================================================

void write_u16(std::ofstream& f, uint16_t val) {
    f.put(static_cast<char>(val & 0xFF));
    f.put(static_cast<char>((val >> 8) & 0xFF));
}

void write_u32(std::ofstream& f, uint32_t val) {
    f.put(static_cast<char>(val & 0xFF));
    f.put(static_cast<char>((val >> 8) & 0xFF));
    f.put(static_cast<char>((val >> 16) & 0xFF));
    f.put(static_cast<char>((val >> 24) & 0xFF));
}

// Zapis do formatu BMP (można otworzyć od razu w systemie Windows / Paint)
bool save_bmp(const std::string& filename, int width, int height, const std::vector<Vec3>& buffer) {
    std::ofstream file(filename, std::ios::binary);
    if (!file) return false;

    int row_padding = (4 - (width * 3) % 4) % 4;
    int row_stride = width * 3 + row_padding;
    uint32_t file_size = 54 + static_cast<uint32_t>(row_stride * height);

    // BITMAPFILEHEADER (14 bajtów)
    file.put('B'); file.put('M');
    write_u32(file, file_size);
    write_u16(file, 0); // reserved
    write_u16(file, 0); // reserved
    write_u32(file, 54); // offset

    // BITMAPINFOHEADER (40 bajtów)
    write_u32(file, 40); // biSize
    write_u32(file, static_cast<uint32_t>(width));
    write_u32(file, static_cast<uint32_t>(height));
    write_u16(file, 1);  // biPlanes
    write_u16(file, 24); // biBitCount (24-bit RGB)
    write_u32(file, 0);  // biCompression (BI_RGB)
    write_u32(file, static_cast<uint32_t>(row_stride * height));
    write_u32(file, 2835); // 72 DPI
    write_u32(file, 2835);
    write_u32(file, 0);
    write_u32(file, 0);

    std::vector<uint8_t> padding(row_padding, 0);

    for (int j = 0; j < height; ++j) {
        for (int i = 0; i < width; ++i) {
            const Vec3& col = buffer[j * width + i];

            // Korekcja gamma 2.0 (sqrt)
            float r = std::clamp(std::sqrt(col.x), 0.0f, 0.999f);
            float g = std::clamp(std::sqrt(col.y), 0.0f, 0.999f);
            float b = std::clamp(std::sqrt(col.z), 0.0f, 0.999f);

            uint8_t ir = static_cast<uint8_t>(256.0f * r);
            uint8_t ig = static_cast<uint8_t>(256.0f * g);
            uint8_t ib = static_cast<uint8_t>(256.0f * b);

            // W formacie BMP kolory zapisujemy jako BGR
            file.put(ib);
            file.put(ig);
            file.put(ir);
        }
        if (row_padding > 0) {
            file.write(reinterpret_cast<const char*>(padding.data()), row_padding);
        }
    }
    return true;
}

// Zapis do formatu PPM (prosty format tekstowy Netpbm)
bool save_ppm(const std::string& filename, int width, int height, const std::vector<Vec3>& buffer) {
    std::ofstream file(filename);
    if (!file) return false;

    file << "P3\n" << width << " " << height << "\n255\n";
    for (int j = height - 1; j >= 0; --j) {
        for (int i = 0; i < width; ++i) {
            const Vec3& col = buffer[j * width + i];
            float r = std::clamp(std::sqrt(col.x), 0.0f, 0.999f);
            float g = std::clamp(std::sqrt(col.y), 0.0f, 0.999f);
            float b = std::clamp(std::sqrt(col.z), 0.0f, 0.999f);

            int ir = static_cast<int>(256.0f * r);
            int ig = static_cast<int>(256.0f * g);
            int ib = static_cast<int>(256.0f * b);

            file << ir << " " << ig << " " << ib << "\n";
        }
    }
    return true;
}

// Zapis do formatu PNG przy użyciu biblioteki stb_image_write
bool save_png(const std::string& filename, int width, int height, const std::vector<Vec3>& buffer) {
    std::vector<uint8_t> pixels(width * height * 3);

    for (int j = 0; j < height; ++j) {
        // PNG zapisuje wiersze od góry do dołu
        int src_row = height - 1 - j;
        for (int i = 0; i < width; ++i) {
            const Vec3& col = buffer[src_row * width + i];

            float r = std::clamp(std::sqrt(col.x), 0.0f, 0.999f);
            float g = std::clamp(std::sqrt(col.y), 0.0f, 0.999f);
            float b = std::clamp(std::sqrt(col.z), 0.0f, 0.999f);

            int dst_idx = (j * width + i) * 3;
            pixels[dst_idx + 0] = static_cast<uint8_t>(256.0f * r);
            pixels[dst_idx + 1] = static_cast<uint8_t>(256.0f * g);
            pixels[dst_idx + 2] = static_cast<uint8_t>(256.0f * b);
        }
    }

    return stbi_write_png(filename.c_str(), width, height, 3, pixels.data(), width * 3) != 0;
}

// ============================================================================
// 8. GŁÓWNA FUNKCJA (MAIN)
// ============================================================================

int main() {
    // Parametry obrazu
    const float aspect_ratio = 16.0f / 9.0f;
    const int width = 800;
    const int height = static_cast<int>(width / aspect_ratio);
    const int samples_per_pixel = 100; // Liczba próbek na piksel (antyaliasing i redukcja szumu)
    const int max_depth = 32;          // Maksymalna liczba odbić promienia

    // Definicja sceny
    HittableList world;

    // 1. Podłoże (szara matowa kula o ogromnym promieniu)
    world.add(std::make_shared<Sphere>(
        Vec3(0.0f, -1000.0f, 0.0f), 1000.0f,
        std::make_shared<Lambertian>(Vec3(0.65f, 0.65f, 0.65f))
    ));

    // 2. Centralna kula (czerwona, matowa)
    world.add(std::make_shared<Sphere>(
        Vec3(0.0f, 1.0f, 0.0f), 1.0f,
        std::make_shared<Lambertian>(Vec3(0.85f, 0.2f, 0.15f))
    ));

    // 3. Lewa kula (szkło - załamanie światła i odbicie)
    world.add(std::make_shared<Sphere>(
        Vec3(-2.2f, 1.0f, 0.0f), 1.0f,
        std::make_shared<Dielectric>(1.5f)
    ));

    // 4. Prawa kula (polerowane złoto - metal)
    world.add(std::make_shared<Sphere>(
        Vec3(2.2f, 1.0f, 0.0f), 1.0f,
        std::make_shared<Metal>(Vec3(0.85f, 0.75f, 0.35f), 0.02f)
    ));

    // 5. Mniejsza kula z przodu (satynowe chłodne aluminium)
    world.add(std::make_shared<Sphere>(
        Vec3(0.7f, 0.4f, 1.3f), 0.4f,
        std::make_shared<Metal>(Vec3(0.7f, 0.8f, 0.9f), 0.25f)
    ));

    // 6. Druga mała kula z przodu (zielona matowa)
    world.add(std::make_shared<Sphere>(
        Vec3(-0.8f, 0.35f, 1.2f), 0.35f,
        std::make_shared<Lambertian>(Vec3(0.2f, 0.75f, 0.3f))
    ));

    // 7. Źródło światła (jasno świecąca ciepła kula wisząca u góry)
    world.add(std::make_shared<Sphere>(
        Vec3(0.0f, 6.0f, 1.5f), 2.0f,
        std::make_shared<DiffuseLight>(Vec3(12.0f, 10.5f, 9.0f))
    ));

    // Ustawienia kamery
    Vec3 lookfrom(0.0f, 2.0f, 5.0f);
    Vec3 lookat(0.0f, 0.8f, 0.0f);
    Vec3 vup(0.0f, 1.0f, 0.0f);
    float vfov = 40.0f;
    Camera cam(lookfrom, lookat, vup, vfov, aspect_ratio);

    // Bufor obrazu
    std::vector<Vec3> image(width * height);

    std::cout << "==========================================\n";
    std::cout << "          C++ PATH TRACER                 \n";
    std::cout << "==========================================\n";
    std::cout << "Rozdzielczosc: " << width << "x" << height << "\n";
    std::cout << "Probki / piksel (SPP): " << samples_per_pixel << "\n";
    std::cout << "Maksymalna glebokosc odbic: " << max_depth << "\n";

    unsigned int num_threads = std::max(1u, std::thread::hardware_concurrency());
    std::cout << "Wielowatkowosc: uruchamianie na " << num_threads << " watkach CPU...\n";

    auto start_time = std::chrono::high_resolution_clock::now();

    // Pula wątków renderujących wiersz po wierszu
    std::atomic<int> next_row(0);
    std::atomic<int> completed_rows(0);
    std::vector<std::thread> workers;

    for (unsigned int t = 0; t < num_threads; ++t) {
        workers.emplace_back([&, t]() {
            // Niezależny generator liczb losowych dla każdego wątku
            std::mt19937 rng(1337 + t * 733);
            std::uniform_real_distribution<float> dist01(0.0f, 1.0f);

            while (true) {
                int j = next_row.fetch_add(1);
                if (j >= height) break;

                for (int i = 0; i < width; ++i) {
                    Vec3 pixel_color(0.0f, 0.0f, 0.0f);

                    // Monte Carlo supersampling wewnątrz piksela (Antialiasing)
                    for (int s = 0; s < samples_per_pixel; ++s) {
                        float u = (static_cast<float>(i) + dist01(rng)) / static_cast<float>(width - 1);
                        float v = (static_cast<float>(j) + dist01(rng)) / static_cast<float>(height - 1);

                        Ray r = cam.get_ray(u, v);
                        pixel_color = pixel_color + ray_color(r, world, max_depth, rng);
                    }

                    image[j * width + i] = pixel_color / static_cast<float>(samples_per_pixel);
                }

                completed_rows.fetch_add(1);
            }
        });
    }

    // Pasek postępu w głównym wątku
    while (completed_rows.load() < height) {
        std::this_thread::sleep_for(std::chrono::milliseconds(150));
        int done = completed_rows.load();
        int percent = done * 100 / height;
        std::cout << "\rPostep: [" << percent << "%] "
                  << "(" << done << "/" << height << " wierszy)" << std::flush;
    }

    for (auto& worker : workers) {
        worker.join();
    }

    auto end_time = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = end_time - start_time;

    std::cout << "\nRenderowanie ukonczone w " << elapsed.count() << " s!\n";

    // Zapis do plików
    std::cout << "Zapisywanie 'output.png'...";
    if (save_png("output.png", width, height, image)) {
        std::cout << " OK!\n";
    } else {
        std::cout << " BLAD zapisu PNG!\n";
    }

    std::cout << "Zapisywanie 'output.bmp'...";
    if (save_bmp("output.bmp", width, height, image)) {
        std::cout << " OK!\n";
    } else {
        std::cout << " BLAD zapisu BMP!\n";
    }

    std::cout << "Zapisywanie 'output.ppm'...";
    if (save_ppm("output.ppm", width, height, image)) {
        std::cout << " OK!\n";
    } else {
        std::cout << " BLAD zapisu PPM!\n";
    }

    std::cout << "Mozesz otworzyc plik 'output.bmp' bezposrednio w przegladarce zdjec lub Paintcie.\n";
    return 0;
}

