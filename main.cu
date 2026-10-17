#include <iostream>
#include <vector>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <algorithm>
#include <cuda_runtime.h>
#include <curand_kernel.h>

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"

// Makro do sprawdzania błędów CUDA
#define CHECK_CUDA(call)                                                      \
    do {                                                                      \
        cudaError_t err = call;                                               \
        if (err != cudaSuccess) {                                             \
            std::cerr << "CUDA Error w " << __FILE__ << ":" << __LINE__        \
                      << " -> " << cudaGetErrorString(err) << std::endl;      \
            return 1;                                                         \
        }                                                                     \
    } while (0)

// ============================================================================
// 1. STRUKTURA WEKTORA NA GPU / HOST (Vec3)
// ============================================================================

struct Vec3 {
    float x, y, z;

    __host__ __device__ Vec3() : x(0.0f), y(0.0f), z(0.0f) {}
    __host__ __device__ Vec3(float x_, float y_, float z_) : x(x_), y(y_), z(z_) {}

    __host__ __device__ Vec3 operator-() const { return Vec3(-x, -y, -z); }
    __host__ __device__ Vec3 operator+(const Vec3& v) const { return Vec3(x + v.x, y + v.y, z + v.z); }
    __host__ __device__ Vec3 operator-(const Vec3& v) const { return Vec3(x - v.x, y - v.y, z - v.z); }
    __host__ __device__ Vec3 operator*(const Vec3& v) const { return Vec3(x * v.x, y * v.y, z * v.z); }
    __host__ __device__ Vec3 operator*(float t) const { return Vec3(x * t, y * t, z * t); }
    __host__ __device__ Vec3 operator/(float t) const { return Vec3(x / t, y / t, z / t); }

    __host__ __device__ float length_squared() const { return x * x + y * y + z * z; }
    __host__ __device__ float length() const { return sqrtf(length_squared()); }
};

__host__ __device__ inline Vec3 operator*(float t, const Vec3& v) { return v * t; }
__host__ __device__ inline float dot(const Vec3& u, const Vec3& v) { return u.x * v.x + u.y * v.y + u.z * v.z; }
__host__ __device__ inline Vec3 cross(const Vec3& u, const Vec3& v) {
    return Vec3(u.y * v.z - u.z * v.y,
                u.z * v.x - u.x * v.z,
                u.x * v.y - u.y * v.x);
}
__host__ __device__ inline Vec3 unit_vector(const Vec3& v) {
    float l = v.length();
    return l > 1e-8f ? v / l : Vec3(0.0f, 0.0f, 0.0f);
}

// Odbicie wektora względem normalnej n
__device__ inline Vec3 reflect(const Vec3& v, const Vec3& n) {
    return v - 2.0f * dot(v, n) * n;
}

// Załamanie światła (Prawo Snella)
__device__ inline Vec3 refract(const Vec3& uv, const Vec3& n, float etai_over_etat) {
    float cos_theta = fminf(dot(-uv, n), 1.0f);
    Vec3 r_out_perp = etai_over_etat * (uv + cos_theta * n);
    Vec3 r_out_parallel = -sqrtf(fabsf(1.0f - r_out_perp.length_squared())) * n;
    return r_out_perp + r_out_parallel;
}

// Aproksymacja Schlicka dla odbicia Fresnela
__device__ inline float schlick_reflectance(float cosine, float ref_idx) {
    float r0 = (1.0f - ref_idx) / (1.0f + ref_idx);
    r0 = r0 * r0;
    return r0 + (1.0f - r0) * powf(1.0f - cosine, 5.0f);
}

// Stałoczasowe generowanie losowego wektora jednostkowego na GPU (bez rozbieżności wątków w warpie)
__device__ inline Vec3 random_unit_vector(curandState* local_rand_state) {
    float z = curand_uniform(local_rand_state) * 2.0f - 1.0f; // [-1, 1]
    float a = curand_uniform(local_rand_state) * 6.28318530718f; // [0, 2pi]
    float r = sqrtf(fmaxf(0.0f, 1.0f - z * z));
    return Vec3(r * cosf(a), r * sinf(a), z);
}

// ============================================================================
// 2. PROMIEŃ (Ray) ORAZ MATERIAŁY (Material)
// ============================================================================

struct Ray {
    Vec3 origin;
    Vec3 direction;

    __host__ __device__ Ray() {}
    __host__ __device__ Ray(const Vec3& o, const Vec3& d) : origin(o), direction(d) {}
    __host__ __device__ Vec3 at(float t) const { return origin + t * direction; }
};

enum MaterialType {
    MAT_LAMBERTIAN,
    MAT_METAL,
    MAT_DIELECTRIC,
    MAT_LIGHT
};

struct Material {
    MaterialType type;
    Vec3 albedo_or_emit;
    float roughness_or_ior;

    __host__ __device__ static Material make_lambertian(const Vec3& albedo) {
        Material m;
        m.type = MAT_LAMBERTIAN;
        m.albedo_or_emit = albedo;
        m.roughness_or_ior = 0.0f;
        return m;
    }

    __host__ __device__ static Material make_metal(const Vec3& albedo, float roughness) {
        Material m;
        m.type = MAT_METAL;
        m.albedo_or_emit = albedo;
        m.roughness_or_ior = roughness < 1.0f ? roughness : 1.0f;
        return m;
    }

    __host__ __device__ static Material make_dielectric(float ior) {
        Material m;
        m.type = MAT_DIELECTRIC;
        m.albedo_or_emit = Vec3(1.0f, 1.0f, 1.0f);
        m.roughness_or_ior = ior;
        return m;
    }

    __host__ __device__ static Material make_light(const Vec3& emit_color) {
        Material m;
        m.type = MAT_LIGHT;
        m.albedo_or_emit = emit_color;
        m.roughness_or_ior = 0.0f;
        return m;
    }
};

struct HitRecord {
    Vec3 p;
    Vec3 normal;
    Material mat;
    float t;
    bool front_face;

    __device__ void set_face_normal(const Ray& r, const Vec3& outward_normal) {
        front_face = dot(r.direction, outward_normal) < 0.0f;
        normal = front_face ? outward_normal : -outward_normal;
    }
};

// ============================================================================
// 3. GEOMETRIA SCENY (Sphere)
// ============================================================================

struct Sphere {
    Vec3 center;
    float radius;
    Material mat;

    __device__ bool hit(const Ray& r, float t_min, float t_max, HitRecord& rec) const {
        Vec3 oc = r.origin - center;
        float a = r.direction.length_squared();
        float half_b = dot(oc, r.direction);
        float c = oc.length_squared() - radius * radius;
        float discriminant = half_b * half_b - a * c;

        if (discriminant < 0.0f) return false;
        float sqrtd = sqrtf(discriminant);

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

__device__ bool world_hit(const Sphere* spheres, int num_spheres, const Ray& r, float t_min, float t_max, HitRecord& rec) {
    HitRecord temp_rec;
    bool hit_anything = false;
    float closest_so_far = t_max;

    for (int i = 0; i < num_spheres; ++i) {
        if (spheres[i].hit(r, t_min, closest_so_far, temp_rec)) {
            hit_anything = true;
            closest_so_far = temp_rec.t;
            rec = temp_rec;
        }
    }
    return hit_anything;
}

// ============================================================================
// 4. KAMERA NA GPU (Camera)
// ============================================================================

struct Camera {
    Vec3 origin;
    Vec3 lower_left_corner;
    Vec3 horizontal;
    Vec3 vertical;

    __host__ __device__ Camera(Vec3 lookfrom, Vec3 lookat, Vec3 vup, float vfov_deg, float aspect_ratio) {
        float theta = vfov_deg * 3.14159265358979323846f / 180.0f;
        float h = tanf(theta / 2.0f);
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

    __device__ Ray get_ray(float s, float t) const {
        return Ray(origin, lower_left_corner + s * horizontal + t * vertical - origin);
    }
};

// ============================================================================
// 5. ITERACYJNY PATH TRACER NA GPU (Brak rekurencji - optymalizacja pod CUDA)
// ============================================================================

__device__ Vec3 trace_path(Ray ray, const Sphere* spheres, int num_spheres, int max_depth, curandState* rand_state) {
    Vec3 accumulated_color(0.0f, 0.0f, 0.0f);
    Vec3 throughput(1.0f, 1.0f, 1.0f);

    for (int bounce = 0; bounce < max_depth; ++bounce) {
        HitRecord rec;
        if (!world_hit(spheres, num_spheres, ray, 0.001f, 1e9f, rec)) {
            // Tło: subtelny gradient nieba
            Vec3 unit_direction = unit_vector(ray.direction);
            float a = 0.5f * (unit_direction.y + 1.0f);
            Vec3 sky = (1.0f - a) * Vec3(0.04f, 0.04f, 0.06f) + a * Vec3(0.12f, 0.18f, 0.28f);
            accumulated_color = accumulated_color + throughput * sky;
            break;
        }

        // Źródło światła
        if (rec.mat.type == MAT_LIGHT) {
            accumulated_color = accumulated_color + throughput * rec.mat.albedo_or_emit;
            break;
        }

        Ray scattered;
        Vec3 attenuation;

        if (rec.mat.type == MAT_LAMBERTIAN) {
            Vec3 scatter_dir = rec.normal + random_unit_vector(rand_state);
            if (scatter_dir.length_squared() < 1e-8f) scatter_dir = rec.normal;
            scattered = Ray(rec.p, unit_vector(scatter_dir));
            attenuation = rec.mat.albedo_or_emit;
        }
        else if (rec.mat.type == MAT_METAL) {
            Vec3 reflected = reflect(unit_vector(ray.direction), rec.normal);
            Vec3 scattered_dir = reflected + rec.mat.roughness_or_ior * random_unit_vector(rand_state);
            scattered = Ray(rec.p, unit_vector(scattered_dir));
            attenuation = rec.mat.albedo_or_emit;
            if (dot(scattered.direction, rec.normal) <= 0.0f) {
                break; // Promień wpadł pod powierzchnię
            }
        }
        else if (rec.mat.type == MAT_DIELECTRIC) {
            attenuation = Vec3(1.0f, 1.0f, 1.0f);
            float refraction_ratio = rec.front_face ? (1.0f / rec.mat.roughness_or_ior) : rec.mat.roughness_or_ior;
            Vec3 unit_dir = unit_vector(ray.direction);
            float cos_theta = fminf(dot(-unit_dir, rec.normal), 1.0f);
            float sin_theta = sqrtf(1.0f - cos_theta * cos_theta);

            bool cannot_refract = refraction_ratio * sin_theta > 1.0f;
            Vec3 direction;
            if (cannot_refract || schlick_reflectance(cos_theta, refraction_ratio) > curand_uniform(rand_state)) {
                direction = reflect(unit_dir, rec.normal);
            } else {
                direction = refract(unit_dir, rec.normal, refraction_ratio);
            }
            scattered = Ray(rec.p, unit_vector(direction));
        }

        throughput = throughput * attenuation;
        ray = scattered;

        // Rosyjska ruletka dla optymalizacji głębokich odbić
        if (bounce > 3) {
            float max_component = fmaxf(throughput.x, fmaxf(throughput.y, throughput.z));
            if (curand_uniform(rand_state) > max_component) {
                break;
            }
            throughput = throughput / max_component;
        }
    }

    return accumulated_color;
}

// ============================================================================
// 6. KERNELE CUDA (Inicjalizacja losowości i renderowanie)
// ============================================================================

__global__ void init_rand_kernel(curandState* rand_states, int width, int height, unsigned long long seed) {
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    int j = blockIdx.y * blockDim.y + threadIdx.y;
    if (i >= width || j >= height) return;

    int pixel_index = j * width + i;
    curand_init(seed + pixel_index, pixel_index, 0, &rand_states[pixel_index]);
}

__global__ void render_kernel(Vec3* fb, int width, int height, int spp, int max_depth,
                              Camera cam, const Sphere* spheres, int num_spheres,
                              curandState* rand_states) {
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    int j = blockIdx.y * blockDim.y + threadIdx.y;
    if (i >= width || j >= height) return;

    int pixel_index = j * width + i;
    curandState local_rand_state = rand_states[pixel_index];

    Vec3 col(0.0f, 0.0f, 0.0f);
    for (int s = 0; s < spp; ++s) {
        float u = (float(i) + curand_uniform(&local_rand_state)) / float(width - 1);
        float v = (float(j) + curand_uniform(&local_rand_state)) / float(height - 1);

        Ray r = cam.get_ray(u, v);
        col = col + trace_path(r, spheres, num_spheres, max_depth, &local_rand_state);
    }

    rand_states[pixel_index] = local_rand_state;
    fb[pixel_index] = col / float(spp);
}

// ============================================================================
// 7. ZAPIS DO PLIKU PNG (stb_image_write)
// ============================================================================

bool save_png(const char* filename, int width, int height, const Vec3* buffer) {
    std::vector<uint8_t> pixels(width * height * 3);

    for (int j = 0; j < height; ++j) {
        // PNG zapisuje wiersze od góry do dołu (top-to-bottom)
        int src_row = height - 1 - j;
        for (int i = 0; i < width; ++i) {
            const Vec3& col = buffer[src_row * width + i];

            // Korekcja gamma 2.0 (sqrt) oraz obcięcie wartości do [0, 1]
            float r = std::clamp(sqrtf(col.x), 0.0f, 0.999f);
            float g = std::clamp(sqrtf(col.y), 0.0f, 0.999f);
            float b = std::clamp(sqrtf(col.z), 0.0f, 0.999f);

            int dst_idx = (j * width + i) * 3;
            pixels[dst_idx + 0] = static_cast<uint8_t>(256.0f * r);
            pixels[dst_idx + 1] = static_cast<uint8_t>(256.0f * g);
            pixels[dst_idx + 2] = static_cast<uint8_t>(256.0f * b);
        }
    }

    return stbi_write_png(filename, width, height, 3, pixels.data(), width * 3) != 0;
}

// ============================================================================
// 8. FUNKCJA GŁÓWNA (HOST)
// ============================================================================

int main() {
    // Informacja o urządzeniu CUDA
    int device_count = 0;
    cudaGetDeviceCount(&device_count);
    if (device_count == 0) {
        std::cerr << "Blad: Nie znaleziono aktywnego urzadzenia CUDA!\n";
        return 1;
    }

    cudaDeviceProp prop;
    cudaGetDeviceProperties(&prop, 0);
    std::cout << "==========================================\n";
    std::cout << "        CUDA GPU PATH TRACER              \n";
    std::cout << "==========================================\n";
    std::cout << "Karta graficzna: " << prop.name << "\n";
    std::cout << "Liczba multiprocesorow (SM): " << prop.multiProcessorCount << "\n";
    std::cout << "Pamiec VRAM: " << (prop.totalGlobalMem / (1024 * 1024)) << " MB\n";

    // Parametry obrazu
    const float aspect_ratio = 16.0f / 9.0f;
    const int width = 1280;
    const int height = static_cast<int>(width / aspect_ratio);
    const int samples_per_pixel = 256; // 256 próbek na piksel (GPU przeliczy to błyskawicznie)
    const int max_depth = 32;
    const size_t num_pixels = width * height;

    std::cout << "Rozdzielczosc: " << width << "x" << height << "\n";
    std::cout << "Probki na piksel (SPP): " << samples_per_pixel << "\n";
    std::cout << "Maksymalna liczba odbic: " << max_depth << "\n";

    // Definicja obiektów na scenie
    std::vector<Sphere> h_spheres = {
        // Podłoże
        Sphere{Vec3(0.0f, -1000.0f, 0.0f), 1000.0f, Material::make_lambertian(Vec3(0.65f, 0.65f, 0.65f))},
        // Centralna czerwona kula (matowa)
        Sphere{Vec3(0.0f, 1.0f, 0.0f), 1.0f, Material::make_lambertian(Vec3(0.85f, 0.2f, 0.15f))},
        // Lewa kula (szkło z załamaniem światła)
        Sphere{Vec3(-2.2f, 1.0f, 0.0f), 1.0f, Material::make_dielectric(1.5f)},
        // Prawa kula (polerowane złoto)
        Sphere{Vec3(2.2f, 1.0f, 0.0f), 1.0f, Material::make_metal(Vec3(0.85f, 0.75f, 0.35f), 0.02f)},
        // Mniejsza kula z przodu (satynowe aluminium)
        Sphere{Vec3(0.7f, 0.4f, 1.3f), 0.4f, Material::make_metal(Vec3(0.7f, 0.8f, 0.9f), 0.25f)},
        // Mniejsza kula zielona
        Sphere{Vec3(-0.8f, 0.35f, 1.2f), 0.35f, Material::make_lambertian(Vec3(0.2f, 0.75f, 0.3f))},
        // Jasna kula emisyjna (światło)
        Sphere{Vec3(0.0f, 6.0f, 1.5f), 2.0f, Material::make_light(Vec3(12.0f, 10.5f, 9.0f))}
    };

    // Alokacja geometrii w pamięci GPU
    Sphere* d_spheres = nullptr;
    size_t spheres_size = h_spheres.size() * sizeof(Sphere);
    CHECK_CUDA(cudaMalloc(&d_spheres, spheres_size));
    CHECK_CUDA(cudaMemcpy(d_spheres, h_spheres.data(), spheres_size, cudaMemcpyHostToDevice));

    // Ustawienia kamery
    Vec3 lookfrom(0.0f, 2.0f, 5.0f);
    Vec3 lookat(0.0f, 0.8f, 0.0f);
    Vec3 vup(0.0f, 1.0f, 0.0f);
    float vfov = 40.0f;
    Camera cam(lookfrom, lookat, vup, vfov, aspect_ratio);

    // Alokacja bufora obrazu i stanów losowych na GPU
    Vec3* d_fb = nullptr;
    curandState* d_rand_states = nullptr;
    CHECK_CUDA(cudaMalloc(&d_fb, num_pixels * sizeof(Vec3)));
    CHECK_CUDA(cudaMalloc(&d_rand_states, num_pixels * sizeof(curandState)));

    // Konfiguracja siatki wątków CUDA (bloki 16x16)
    dim3 threads(16, 16);
    dim3 blocks((width + threads.x - 1) / threads.x, (height + threads.y - 1) / threads.y);

    std::cout << "Inicjalizacja generatorow liczb losowych na GPU...\n";
    init_rand_kernel<<<blocks, threads>>>(d_rand_states, width, height, 1337ULL);
    CHECK_CUDA(cudaGetLastError());
    CHECK_CUDA(cudaDeviceSynchronize());

    std::cout << "Uruchamianie kernela renderujacego na rdzeniach CUDA...\n";
    auto start_time = std::chrono::high_resolution_clock::now();

    render_kernel<<<blocks, threads>>>(
        d_fb, width, height, samples_per_pixel, max_depth,
        cam, d_spheres, static_cast<int>(h_spheres.size()), d_rand_states
    );
    CHECK_CUDA(cudaGetLastError());
    CHECK_CUDA(cudaDeviceSynchronize());

    auto end_time = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = end_time - start_time;
    std::cout << "Renderowanie na GPU zakonczone sukcesem w " << elapsed.count() << " sekund!\n";

    // Kopiowanie wyniku z VRAM (GPU) do pamięci RAM (Host)
    std::vector<Vec3> h_fb(num_pixels);
    CHECK_CUDA(cudaMemcpy(h_fb.data(), d_fb, num_pixels * sizeof(Vec3), cudaMemcpyDeviceToHost));

    // Zwalnianie pamięci GPU
    cudaFree(d_spheres);
    cudaFree(d_fb);
    cudaFree(d_rand_states);

    // Zapis do pliku PNG
    std::cout << "Zapisywanie obrazu do pliku 'output.png'...";
    if (save_png("output.png", width, height, h_fb.data())) {
        std::cout << " OK!\n";
    } else {
        std::cerr << " BLAD zapisu PNG!\n";
        return 1;
    }

    std::cout << "Gotowe! Obraz zostal zapisany jako output.png\n";
    return 0;
}

