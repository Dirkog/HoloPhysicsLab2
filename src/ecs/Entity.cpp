#include "Entity.h"
#include "../core/Types.h"

namespace hlp {

void ECSPhysicsSystem::update(Registry& registry, double dt) {
    auto& transforms = registry.transforms();
    auto& physics = registry.physics();
    
    // Apply gravity
    for (size_t i = 0; i < physics.size(); i++) {
        auto& p = physics.data[i];
        auto& t = transforms.data[i];
        
        if (p.is_static) continue;
        
        // Gravity
        p.force.y -= p.mass * gravity * p.gravity_scale;
        
        // Integrate (semi-implicit Euler)
        p.velocity += (p.force / p.mass) * dt;
        p.velocity *= (1.0 - damping);
        t.position += p.velocity * dt;
        
        // Reset force
        p.force = dvec3(0);
    }
}

void ECSRenderSystem::update(Registry& registry, double dt) {
    auto& render = registry.render_components();
    auto& transforms = registry.transforms();
    (void)dt;
    // In production: submit render commands to GPU
    // This would be handled by the actual renderer
}

} // namespace hlp
