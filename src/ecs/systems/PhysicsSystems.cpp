#include "PhysicsSystems.h"
#include <algorithm>
#include <cmath>
#include <iostream>

namespace hlp {

// ==================== GravitySystem ====================

void GravitySystem::update(Registry& registry, double dt) {
    (void)dt;
    auto& physics = registry.physics();
    for (size_t i = 0; i < physics.size(); i++) {
        auto& p = physics.get(physics.entities()[i]);
        if (p.is_static || p.is_kinematic) continue;
        p.force.y -= p.mass * gravity * p.gravity_scale;
    }
}

// ==================== IntegrationSystem ====================

void IntegrationSystem::update(Registry& registry, double dt) {
    switch (method) {
        case SYMPLECTIC: step_symplectic(registry, dt); break;
        default: step_euler(registry, dt); break;
    }
}

void IntegrationSystem::step_symplectic(Registry& reg, double dt) {
    auto& physics = reg.physics();
    auto& transforms = reg.transforms();
    
    for (size_t i = 0; i < physics.size(); i++) {
        Entity e = physics.entities()[i];
        auto& p = physics.get(e);
        if (p.is_static) continue;
        if (!transforms.has(e)) continue;
        auto& t = transforms.get(e);
        
        // Save previous state for interpolation
        t.prev_position = t.position;
        t.prev_rotation = t.rotation;
        
        // Symplectic Euler: v(t+dt) = v(t) + a·dt, x(t+dt) = x(t) + v(t+dt)·dt
        if (!p.is_kinematic) {
            dvec3 accel = p.force * p.inv_mass;
            p.velocity += accel * dt;
        }
        
        // Damping
        p.velocity *= (1.0 - p.damping);
        
        t.position += p.velocity * dt;
        
        // Reset force
        p.force = dvec3(0);
    }
}

void IntegrationSystem::step_euler(Registry& reg, double dt) {
    auto& physics = reg.physics();
    auto& transforms = reg.transforms();
    
    for (size_t i = 0; i < physics.size(); i++) {
        Entity e = physics.entities()[i];
        auto& p = physics.get(e);
        if (p.is_static) continue;
        if (!transforms.has(e)) continue;
        auto& t = transforms.get(e);
        
        t.prev_position = t.position;
        
        if (!p.is_kinematic) {
            dvec3 accel = p.force * p.inv_mass;
            p.velocity += accel * dt;
            p.velocity *= (1.0 - p.damping);
        }
        
        t.position += p.velocity * dt;
        p.force = dvec3(0);
    }
}

// ==================== ECSCollisionSystem ====================

ECSCollisionSystem::ECSCollisionSystem() = default;

uint64_t ECSCollisionSystem::hash_cell(int x, int y, int z) const {
    return (uint64_t)(x * 73856093) ^ (uint64_t)(y * 19349663) ^ (uint64_t)(z * 83492791);
}

void ECSCollisionSystem::broad_phase(Registry& reg, 
                                      std::vector<std::pair<Entity,Entity>>& pairs) {
    pairs.clear();
    grid_.clear();
    
    auto& transforms = reg.transforms();
    auto& colliders = reg.colliders();
    auto& physics = reg.physics();
    
    // Build spatial hash
    for (size_t i = 0; i < colliders.size(); i++) {
        Entity e = colliders.entities()[i];
        if (!transforms.has(e) || !physics.has(e)) continue;
        
        auto& t = transforms.get(e);
        auto& c = colliders.get(e);
        auto& p = physics.get(e);
        if (p.is_static && !c.is_trigger) continue;
        
        int cx = (int)std::floor(t.position.x / cell_size_);
        int cy = (int)std::floor(t.position.y / cell_size_);
        int cz = (int)std::floor(t.position.z / cell_size_);
        
        // Insert into current cell and neighbors
        for (int dx = -1; dx <= 1; dx++)
            for (int dy = -1; dy <= 1; dy++)
                for (int dz = -1; dz <= 1; dz++) {
                    uint64_t h = hash_cell(cx + dx, cy + dy, cz + dz);
                    grid_[h].entities.push_back(e);
                }
    }
    
    // Generate pairs from cells
    for (auto& [hash, cell] : grid_) {
        auto& ents = cell.entities;
        for (size_t i = 0; i < ents.size(); i++) {
            for (size_t j = i + 1; j < ents.size(); j++) {
                Entity a = ents[i], b = ents[j];
                if (a == b) continue;
                if (a > b) std::swap(a, b);
                pairs.push_back({a, b});
            }
        }
    }
    
    // Remove duplicates
    std::sort(pairs.begin(), pairs.end());
    pairs.erase(std::unique(pairs.begin(), pairs.end()), pairs.end());
}

void ECSCollisionSystem::narrow_phase(Registry& reg, 
                                       const std::pair<Entity,Entity>& pair) {
    auto& transforms = reg.transforms();
    auto& colliders = reg.colliders();
    auto& physics = reg.physics();
    
    Entity a = pair.first, b = pair.second;
    if (!transforms.has(a) || !transforms.has(b)) return;
    if (!colliders.has(a) || !colliders.has(b)) return;
    
    auto& ta = transforms.get(a);
    auto& tb = transforms.get(b);
    auto& ca = colliders.get(a);
    auto& cb = colliders.get(b);
    auto& pa = physics.get(a);
    auto& pb = physics.get(b);
    
    if (pa.is_static && pb.is_static) return;
    
    // Sphere-sphere fast path
    dvec3 delta = tb.position - ta.position;
    double dist = glm::length(delta);
    double min_dist = ca.radius + cb.radius;
    
    if (dist >= min_dist || dist < 1e-10) return;
    
    dvec3 normal = delta / dist;
    double penetration = min_dist - dist;
    
    // Trigger check
    if (ca.is_trigger || cb.is_trigger) {
        // Just emit event, don't resolve
        CollisionEventECS ev;
        ev.a = a; ev.b = b;
        ev.point = ta.position + normal * (ca.radius - penetration * 0.5);
        ev.normal = normal;
        ev.impulse = 0;
        events_.push_back(ev);
        return;
    }
    
    resolve_contact(reg, a, b, normal, penetration);
}

void ECSCollisionSystem::resolve_contact(Registry& reg, Entity a, Entity b,
                                          const dvec3& normal, double penetration) {
    auto& transforms = reg.transforms();
    auto& colliders = reg.colliders();
    auto& physics = reg.physics();
    
    auto& ta = transforms.get(a);
    auto& tb = transforms.get(b);
    auto& ca = colliders.get(a);
    auto& cb = colliders.get(b);
    auto& pa = physics.get(a);
    auto& pb = physics.get(b);
    
    double total_mass = pa.mass + pb.mass;
    if (total_mass < 1e-10) return;
    
    // Position correction
    double correction = penetration * 0.8;
    if (!pa.is_static)
        ta.position -= normal * correction * (pb.mass / total_mass);
    if (!pb.is_static)
        tb.position += normal * correction * (pa.mass / total_mass);
    
    // Impulse
    dvec3 rel_v = pa.velocity - pb.velocity;
    double rel_vn = glm::dot(rel_v, normal);
    
    if (rel_vn > 0) return;  // Separating
    
    double e = (ca.restitution + cb.restitution) * 0.5;
    double impulse = -(1.0 + e) * rel_vn / total_mass;
    
    if (!pa.is_static)
        pa.velocity += impulse * pb.mass * normal;
    if (!pb.is_static)
        pb.velocity -= impulse * pa.mass * normal;
    
    // Friction
    dvec3 tangent = rel_v - rel_vn * normal;
    double t_len = glm::length(tangent);
    if (t_len > 1e-10) {
        double f = (ca.friction + cb.friction) * 0.5;
        double jt = std::min(f * impulse, t_len * 0.5);
        if (!pa.is_static)
            pa.velocity -= jt * tangent / t_len / pa.mass;
        if (!pb.is_static)
            pb.velocity += jt * tangent / t_len / pb.mass;
    }
    
    // Record event
    CollisionEventECS ev;
    ev.a = a; ev.b = b;
    ev.point = ta.position + normal * (ca.radius - penetration * 0.5);
    ev.normal = normal;
    ev.impulse = impulse;
    events_.push_back(ev);
}

void ECSCollisionSystem::update(Registry& registry, double dt) {
    (void)dt;
    events_.clear();
    
    std::vector<std::pair<Entity,Entity>> pairs;
    broad_phase(registry, pairs);
    
    for (const auto& pair : pairs) {
        narrow_phase(registry, pair);
    }
}

// ==================== ECSConstraintSystem ====================

void ECSConstraintSystem::solve_distance(Registry& reg, 
                                          const SpringConstraintComponent& s, Entity) {
    auto& transforms = reg.transforms();
    auto& physics = reg.physics();
    
    if (!transforms.has(s.body_a) || !transforms.has(s.body_b)) return;
    if (!physics.has(s.body_a) || !physics.has(s.body_b)) return;
    
    auto& ta = transforms.get(s.body_a);
    auto& tb = transforms.get(s.body_b);
    auto& pa = physics.get(s.body_a);
    auto& pb = physics.get(s.body_b);
    
    dvec3 delta = tb.position - ta.position;
    double dist = glm::length(delta);
    if (dist < 1e-10) return;
    
    double inv_mass_a = pa.is_static ? 0 : pa.inv_mass;
    double inv_mass_b = pb.is_static ? 0 : pb.inv_mass;
    double total_inv = inv_mass_a + inv_mass_b;
    if (total_inv < 1e-10) return;
    
    double error = dist - s.rest_length;
    dvec3 correction = (delta / dist) * error * s.stiffness / total_inv;
    
    if (!pa.is_static) ta.position += correction * inv_mass_a;
    if (!pb.is_static) tb.position -= correction * inv_mass_b;
}

void ECSConstraintSystem::solve_all(Registry& reg, int iterations) {
    auto& springs = reg.springs();
    
    for (int iter = 0; iter < iterations; iter++) {
        for (size_t i = 0; i < springs.size(); i++) {
            const auto& s = springs.get(springs.entities()[i]);
            if (!s.active) continue;
            solve_distance(reg, s, springs.entities()[i]);
        }
    }
}

void ECSConstraintSystem::update(Registry& registry, double dt) {
    (void)dt;
    solve_all(registry, 5);
}

// ==================== ECSCCDSystem ====================

bool ECSCCDSystem::check_sweep(Registry& reg, Entity fast, Entity other, double dt) {
    auto& transforms = reg.transforms();
    auto& colliders = reg.colliders();
    auto& physics = reg.physics();
    
    if (!transforms.has(fast) || !transforms.has(other)) return false;
    
    auto& tf = transforms.get(fast);
    auto& to = transforms.get(other);
    auto& pf = physics.get(fast);
    auto& cf = colliders.get(fast);
    auto& co = colliders.get(other);
    
    dvec3 rel_vel = pf.velocity;
    double speed = glm::length(rel_vel);
    double threshold = 2.0 * cf.radius;
    
    if (speed * dt < threshold) return false;  // Slow enough for discrete
    
    // Ray-sphere sweep
    dvec3 dir = rel_vel / speed;
    dvec3 oc = tf.position - to.position;
    double a = glm::dot(dir, dir);
    double b = 2.0 * glm::dot(oc, dir);
    double c = glm::dot(oc, oc) - (cf.radius + co.radius) * (cf.radius + co.radius);
    double disc = b * b - 4.0 * a * c;
    
    if (disc < 0) return false;
    
    double t = (-b - std::sqrt(disc)) / (2.0 * a);
    if (t > 0 && t < 1.0) {
        // Collision within this step: stop at TOI
        tf.position = tf.position + rel_vel * dt * (t - 1.0);
        return true;
    }
    
    return false;
}

void ECSCCDSystem::update(Registry& registry, double dt) {
    auto& physics = registry.physics();
    auto& colliders = registry.colliders();
    
    for (size_t i = 0; i < physics.size(); i++) {
        Entity fast = physics.entities()[i];
        if (!colliders.has(fast)) continue;
        auto& pf = physics.get(fast);
        if (pf.is_static) continue;
        
        double speed = glm::length(pf.velocity);
        if (speed * dt < 2.0 * colliders.get(fast).radius) continue;
        
        for (size_t j = 0; j < colliders.size(); j++) {
            Entity other = colliders.entities()[j];
            if (fast == other) continue;
            check_sweep(registry, fast, other, dt);
        }
    }
}

// ==================== VirtualCouplingSystem ====================

Entity VirtualCouplingSystem::find_hand_entity(Registry& reg, int hand_index) {
    auto& tags = reg.hand_tags();
    for (size_t i = 0; i < tags.size(); i++) {
        Entity e = tags.entities()[i];
        if (tags.get(e).hand_index == hand_index)
            return e;
    }
    return NULL_ENTITY;
}

void VirtualCouplingSystem::set_hand_target(int hand_index, 
                                             const dvec3& position,
                                             const dvec3& velocity) {
    if (hand_index >= 0 && hand_index < 2) {
        targets_[hand_index].position = position;
        targets_[hand_index].velocity = velocity;
        targets_[hand_index].active = true;
    }
}

void VirtualCouplingSystem::update(Registry& registry, double dt) {
    for (int h = 0; h < 2; h++) {
        if (!targets_[h].active) continue;
        
        Entity hand = find_hand_entity(registry, h);
        if (hand == NULL_ENTITY) continue;
        
        auto& transforms = registry.transforms();
        auto& physics = registry.physics();
        auto& colliders = registry.colliders();
        
        if (!transforms.has(hand) || !physics.has(hand)) continue;
        
        auto& t = transforms.get(hand);
        auto& p = physics.get(hand);
        
        // PD force: Anti-Slingshot
        dvec3 error = targets_[h].position - t.position;
        double dist = glm::length(error);
        
        double current_kp = kp;
        double current_kd = kd;
        
        if (dist > max_stretch) {
            error *= max_stretch / dist;
            double slip = max_stretch / dist;
            current_kp *= (slip * slip);
            current_kd *= 2.0;
            p.velocity *= 0.1;
        }
        
        dvec3 vel_error = targets_[h].velocity - p.velocity;
        dvec3 force = error * current_kp + vel_error * current_kd;
        
        double force_mag = glm::length(force);
        if (force_mag > max_force) {
            force *= max_force / force_mag;
        }
        
        // Apply
        p.force += force;
    }
}

// ==================== ECSPhysicsEngine ====================

ECSPhysicsEngine::ECSPhysicsEngine() {
    systems_.push_back(&gravity);
    systems_.push_back(&integrator);
    systems_.push_back(&collisions);
    systems_.push_back(&constraints);
    systems_.push_back(&ccd);
    systems_.push_back(&coupling);
}

ECSPhysicsEngine::~ECSPhysicsEngine() = default;

void ECSPhysicsEngine::step(Registry& registry, double dt) {
    double sub_dt = dt / substeps_;
    
    for (int s = 0; s < substeps_; s++) {
        for (auto* sys : systems_) {
            sys->update(registry, sub_dt);
        }
    }
}

void ECSPhysicsEngine::add_system(ECS_System* system) {
    systems_.push_back(system);
}

void ECSPhysicsEngine::remove_system(ECS_System* system) {
    systems_.erase(std::remove(systems_.begin(), systems_.end(), system), systems_.end());
}

} // namespace hlp
