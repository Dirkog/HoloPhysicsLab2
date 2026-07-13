#include "CameraRelativeRenderer.h"
#include <glm/gtc/matrix_transform.hpp>
#include <algorithm>
#include <cmath>

namespace hlp {

CameraRelativeRenderer::CameraRelativeRenderer(Renderer& base_renderer) 
    : renderer_(base_renderer) {}

CameraRelativeRenderer::~CameraRelativeRenderer() = default;

CameraRelativeRenderer::CameraRelativeMatrices 
CameraRelativeRenderer::compute_matrices(const dvec3& camera_pos,
                                          const glm::mat4& proj) {
    CameraRelativeMatrices result;
    result.origin = camera_pos;
    result.proj = proj;
    
    // View matrix with ZERO translation (relative to origin)
    // The camera is at (0,0,0) in relative space
    glm::vec3 eye_rel(0.0f, 0.0f, 0.0f);
    glm::vec3 target_rel(0.0f, 0.0f, -1.0f);
    glm::vec3 up(0.0f, 1.0f, 0.0f);
    
    // Use the same rotation as the original camera
    // Get camera orientation from renderer
    glm::mat4 view_orig = renderer_.view_matrix();
    glm::mat4 view_rot = glm::mat4(glm::mat3(view_orig)); // Extract rotation only
    
    // Build relative view: rotated around origin
    result.view_rel = glm::lookAt(eye_rel, target_rel, up);
    // Apply original rotation
    result.view_rel = view_rot * result.view_rel;
    
    result.view_proj_rel = proj * result.view_rel;
    
    return result;
}

glm::vec3 CameraRelativeRenderer::world_to_relative(const dvec3& world_pos,
                                                      const dvec3& origin) const {
    dvec3 rel = world_pos - origin;
    return SafeFloatConverter::to_float(rel);
}

std::vector<CameraRelativeRenderer::BodyRenderData>
CameraRelativeRenderer::prepare_bodies(const std::vector<Body>& bodies,
                                        const dvec3& origin) {
    std::vector<BodyRenderData> result;
    result.reserve(bodies.size());
    
    for (const auto& b : bodies) {
        BodyRenderData d;
        d.position = world_to_relative(b.state.position, origin);
        d.radius = (float)b.props.radius;
        d.color = b.visual.color;
        d.emissive = b.visual.emissive_intensity;
        d.shape = (int)b.visual.shape;
        result.push_back(d);
    }
    
    return result;
}

std::vector<CameraRelativeRenderer::TrailRenderData>
CameraRelativeRenderer::prepare_trails(const std::vector<Body>& bodies,
                                        const dvec3& origin) {
    std::vector<TrailRenderData> result;
    
    for (const auto& b : bodies) {
        if (b.trail.empty()) continue;
        
        TrailRenderData td;
        td.color = b.visual.color;
        td.points.reserve(b.trail.size());
        
        for (const auto& p : b.trail) {
            td.points.push_back(world_to_relative(p, origin));
        }
        
        result.push_back(std::move(td));
    }
    
    return result;
}

glm::vec3 CameraRelativeRenderer::hand_to_relative(const dvec3& hand_world,
                                                     const dvec3& origin) const {
    return world_to_relative(hand_world, origin);
}

// ==================== SafeFloatConverter ====================

glm::vec3 SafeFloatConverter::to_float(const dvec3& v, const dvec3& origin) {
    dvec3 rel = v - origin;
    
    // Clamp to safe range
    double len = glm::length(rel);
    if (len > MAX_SAFE_DIST) {
        // Scale down to keep within float precision
        rel *= MAX_SAFE_DIST / len;
    }
    
    return glm::vec3((float)rel.x, (float)rel.y, (float)rel.z);
}

void SafeFloatConverter::to_float_array(const dvec3* src, glm::vec3* dst,
                                          int count, const dvec3& origin) {
    for (int i = 0; i < count; i++) {
        dst[i] = to_float(src[i], origin);
    }
}

bool SafeFloatConverter::is_safe(const dvec3& v_rel) {
    double len = glm::length(v_rel);
    return len < MAX_SAFE_DIST;
}

} // namespace hlp
