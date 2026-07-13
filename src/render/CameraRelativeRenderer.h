#pragma once

// ============================================================
// HoloPhysics Lab 2 — Camera-Relative Rendering (Origin Rebiasing)
//
// Решает: Double Precision vs GPU Float конфликт
// Проблема: физика в double (64-bit), шейдеры в float (32-bit).
//   На расстоянии >1000 единиц точность float падает ниже
//   сантиметра → Z-fighting, spatial jitter.
//
// Решение: все мировые координаты перед отправкой в шейдер
//   пересчитываются относительно камеры в double precision,
//   и только после вычитания кастуются в float.
//   Матрица вида получает T = (0,0,0).
// ============================================================

#include "Renderer.h"
#include "../core/Types.h"
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

namespace hlp {

class CameraRelativeRenderer {
public:
    CameraRelativeRenderer(Renderer& base_renderer);
    ~CameraRelativeRenderer();

    // Включить/выключить CRR
    void set_enabled(bool e) { enabled_ = e; }
    bool enabled() const { return enabled_; }

    // === Главные методы ===
    
    // Получить камеро-относительные матрицы для шейдера
    // origin_world — точка отсчёта (обычно позиция камеры в double)
    struct CameraRelativeMatrices {
        glm::mat4 view_rel;      // Вид с T=(0,0,0)
        glm::mat4 proj;          // Проекция (без изменений)
        glm::mat4 view_proj_rel; // VP с относительным видом
        dvec3 origin;            // Смещение (double)
    };
    
    CameraRelativeMatrices compute_matrices(const dvec3& camera_pos, 
                                             const glm::mat4& proj);
    
    // Преобразовать мировую точку (double) в камеро-относительную (float)
    glm::vec3 world_to_relative(const dvec3& world_pos, 
                                 const dvec3& origin) const;
    
    // Получить VBO-ready данные: все позиции тел относительно камеры
    struct BodyRenderData {
        glm::vec3 position;     // float, relative to camera
        float radius;
        glm::vec3 color;
        float emissive;
        int shape;
    };
    
    std::vector<BodyRenderData> prepare_bodies(
        const std::vector<Body>& bodies,
        const dvec3& origin);
    
    // Для рендерера трейлов
    struct TrailRenderData {
        std::vector<glm::vec3> points; // relative to camera
        glm::vec3 color;
    };
    
    std::vector<TrailRenderData> prepare_trails(
        const std::vector<Body>& bodies,
        const dvec3& origin);

    // === Для Hand Tracking / AR ===
    glm::vec3 hand_to_relative(const dvec3& hand_world,
                                const dvec3& origin) const;

private:
    Renderer& renderer_;
    bool enabled_ = true;
    
    // Вспомогательные функции
    glm::mat4 build_relative_view(const dvec3& origin,
                                   const glm::vec3& forward,
                                   const glm::vec3& up) const;
};

// ==================== Double→Float Safe Converter ====================

class SafeFloatConverter {
public:
    // Конвертирует dvec3 → vec3 с контролем переполнения
    static glm::vec3 to_float(const dvec3& v, const dvec3& origin = dvec3(0));
    
    // То же для массива
    static void to_float_array(const dvec3* src, glm::vec3* dst, 
                                int count, const dvec3& origin);
    
    // Проверка на выход за пределы точности float
    static bool is_safe(const dvec3& v_rel);
    
    // Максимальное безопасное расстояние от камеры
    static constexpr double MAX_SAFE_DIST = 10000.0;
};

} // namespace hlp
