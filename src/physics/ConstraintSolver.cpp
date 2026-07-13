#include "ConstraintSolver.h"
#include <cmath>
#include <algorithm>
#include <limits>

namespace hlp {

// ==================== ConstraintSolver ====================

ConstraintSolver::ConstraintSolver() = default;
ConstraintSolver::~ConstraintSolver() = default;

int ConstraintSolver::add_constraint(const ConstraintParams& params) {
    constraints_.push_back(params);
    return (int)constraints_.size() - 1;
}

void ConstraintSolver::remove_constraint(int index) {
    if (index >= 0 && index < (int)constraints_.size())
        constraints_.erase(constraints_.begin() + index);
}

void ConstraintSolver::clear_constraints() { constraints_.clear(); }

ConstraintParams* ConstraintSolver::get_constraint(int index) {
    if (index >= 0 && index < (int)constraints_.size())
        return &constraints_[index];
    return nullptr;
}

int ConstraintSolver::active_constraint_count() const {
    int count = 0;
    for (const auto& c : constraints_)
        if (c.active) count++;
    return count;
}

double ConstraintSolver::stiffness_to_compliance(double k, double dt) {
    // XPBD: compliance = 1/(k * dt^2)
    if (k < 1e-10 || dt < 1e-10) return 0;
    return 1.0 / (k * dt * dt);
}

dvec3 ConstraintSolver::get_world_anchor(const Body& body, const dvec3& local_anchor) {
    // Transform local anchor to world space
    // Simplified: for bodies with no rotation, just add position
    return body.state.position + local_anchor;
}

void ConstraintSolver::solve_distance(std::vector<Body>& bodies,
                                       const ConstraintParams& c) {
    Body* a = nullptr, * b = nullptr;
    for (auto& body : bodies) {
        if (body.id == c.body_a) a = &body;
        if (body.id == c.body_b) b = &body;
    }
    if (!a || !b) return;
    
    dvec3 delta = b->state.position - a->state.position;
    double dist = glm::length(delta);
    if (dist < 1e-10) return;
    
    double inv_mass_a = a->is_static() ? 0 : 1.0 / a->props.mass;
    double inv_mass_b = b->is_static() ? 0 : 1.0 / b->props.mass;
    double total_inv_mass = inv_mass_a + inv_mass_b;
    if (total_inv_mass < 1e-10) return;
    
    double error = dist - c.distance.rest_length;
    dvec3 correction = delta / dist * error * c.distance.stiffness / total_inv_mass;
    
    if (!a->is_static()) a->state.position += correction * inv_mass_a;
    if (!b->is_static()) b->state.position -= correction * inv_mass_b;
}

void ConstraintSolver::solve_ball_socket(std::vector<Body>& bodies,
                                          const ConstraintParams& c) {
    Body* a = nullptr, * b = nullptr;
    for (auto& body : bodies) {
        if (body.id == c.body_a) a = &body;
        if (body.id == c.body_b) b = &body;
    }
    if (!a || !b) return;
    
    dvec3 anchor_a_world = get_world_anchor(*a, c.hinge.anchor_a);
    dvec3 anchor_b_world = get_world_anchor(*b, c.hinge.anchor_b);
    
    dvec3 delta = anchor_b_world - anchor_a_world;
    double dist = glm::length(delta);
    if (dist < 1e-10) return;
    
    double inv_mass_a = a->is_static() ? 0 : 1.0 / a->props.mass;
    double inv_mass_b = b->is_static() ? 0 : 1.0 / b->props.mass;
    double total_inv_mass = inv_mass_a + inv_mass_b;
    if (total_inv_mass < 1e-10) return;
    
    dvec3 correction = delta * c.hinge.stiffness / total_inv_mass;
    
    if (!a->is_static()) a->state.position += correction * inv_mass_a;
    if (!b->is_static()) b->state.position -= correction * inv_mass_b;
}

void ConstraintSolver::solve_hinge(std::vector<Body>& bodies,
                                    const ConstraintParams& c) {
    // Hinge: bodies can rotate around a common axis
    // Implementation: ball-socket + angular constraint around axis
    solve_ball_socket(bodies, c);
    
    // Angular part: project angular velocity onto hinge axis
    Body* a = nullptr, * b = nullptr;
    for (auto& body : bodies) {
        if (body.id == c.body_a) a = &body;
        if (body.id == c.body_b) b = &body;
    }
    if (!a || !b) return;
    
    // Remove angular velocity components not along the hinge axis
    // (simplified - full implementation would use quaternion constraints)
    dvec3 axis = c.hinge.axis_a;
    if (!a->is_static()) {
        dvec3 w = a->state.angular_velocity;
        double along = glm::dot(w, axis);
        a->state.angular_velocity = axis * along;
    }
    if (!b->is_static()) {
        dvec3 w = b->state.angular_velocity;
        double along = glm::dot(w, axis);
        b->state.angular_velocity = axis * along;
    }
}

void ConstraintSolver::solve_slider(std::vector<Body>& bodies,
                                     const ConstraintParams& c) {
    Body* a = nullptr, * b = nullptr;
    for (auto& body : bodies) {
        if (body.id == c.body_a) a = &body;
        if (body.id == c.body_b) b = &body;
    }
    if (!a || !b) return;
    
    dvec3 axis = glm::normalize(c.slider.axis);
    dvec3 delta = b->state.position - a->state.position;
    
    // Project delta onto axis (allowed) and perpendicular (constrained)
    double along = glm::dot(delta, axis);
    dvec3 delta_perp = delta - axis * along;
    
    double dist_perp = glm::length(delta_perp);
    if (dist_perp < 1e-10) return;
    
    double inv_mass_a = a->is_static() ? 0 : 1.0 / a->props.mass;
    double inv_mass_b = b->is_static() ? 0 : 1.0 / b->props.mass;
    double total_inv_mass = inv_mass_a + inv_mass_b;
    if (total_inv_mass < 1e-10) return;
    
    dvec3 correction = delta_perp * c.slider.stiffness / total_inv_mass;
    
    if (!a->is_static()) a->state.position += correction * inv_mass_a;
    if (!b->is_static()) b->state.position -= correction * inv_mass_b;
}

void ConstraintSolver::solve_fixed(std::vector<Body>& bodies,
                                    const ConstraintParams& c) {
    Body* a = nullptr, * b = nullptr;
    for (auto& body : bodies) {
        if (body.id == c.body_a) a = &body;
        if (body.id == c.body_b) b = &body;
    }
    if (!a || !b) return;
    
    // Position constraint
    dvec3 target_pos = b->state.position;
    dvec3 delta = target_pos - a->state.position;
    
    double inv_mass_a = a->is_static() ? 0 : 1.0 / a->props.mass;
    double inv_mass_b = b->is_static() ? 0 : 1.0 / b->props.mass;
    double total_inv_mass = inv_mass_a + inv_mass_b;
    
    if (total_inv_mass > 1e-10) {
        dvec3 correction = delta * c.fixed.stiffness / total_inv_mass;
        if (!a->is_static()) a->state.position += correction * inv_mass_a;
        if (!b->is_static()) b->state.position -= correction * inv_mass_b;
    }
}

void ConstraintSolver::solve_spring(std::vector<Body>& bodies,
                                     const ConstraintParams& c, double dt) {
    Body* a = nullptr, * b = nullptr;
    for (auto& body : bodies) {
        if (body.id == c.body_a) a = &body;
        if (body.id == c.body_b) b = &body;
    }
    if (!a || !b) return;
    
    dvec3 delta = b->state.position - a->state.position;
    double dist = glm::length(delta);
    if (dist < 1e-10) return;
    
    dvec3 dir = delta / dist;
    double stretch = dist - c.spring.rest_length;
    
    // Spring force
    double force_mag = c.spring.stiffness * stretch;
    
    // Damping
    dvec3 rel_v = a->state.velocity - b->state.velocity;
    double damp_mag = c.spring.damping * glm::dot(rel_v, dir);
    
    double total_force = (force_mag + damp_mag) * dt;
    
    double inv_mass_a = a->is_static() ? 0 : 1.0 / a->props.mass;
    double inv_mass_b = b->is_static() ? 0 : 1.0 / b->props.mass;
    
    dvec3 impulse = dir * total_force;
    if (!a->is_static()) a->state.velocity += impulse * inv_mass_a;
    if (!b->is_static()) b->state.velocity -= impulse * inv_mass_b;
}

void ConstraintSolver::solve_angular_spring(std::vector<Body>& bodies,
                                             const ConstraintParams& c, double dt) {
    Body* a = nullptr, * b = nullptr;
    for (auto& body : bodies) {
        if (body.id == c.body_a) a = &body;
        if (body.id == c.body_b) b = &body;
    }
    if (!a || !b) return;
    
    // Simplified: apply restoring torque proportional to angular difference
    dvec3 ang_diff = a->state.angular_velocity - b->state.angular_velocity;
    dvec3 torque = -ang_diff * c.angular_spring.stiffness * dt;
    
    if (!a->is_static())
        a->state.angular_velocity += torque * (1.0 / a->props.mass);
    if (!b->is_static())
        b->state.angular_velocity -= torque * (1.0 / b->props.mass);
}

void ConstraintSolver::solve(std::vector<Body>& bodies, double dt, int iterations) {
    for (int iter = 0; iter < iterations; iter++) {
        for (size_t i = 0; i < constraints_.size(); i++) {
            if (!constraints_[i].active) continue;
            solve_constraint(bodies, i, dt);
        }
    }
}

void ConstraintSolver::solve_constraint(std::vector<Body>& bodies, int index, double dt) {
    if (index < 0 || index >= (int)constraints_.size()) return;
    auto& c = constraints_[index];
    if (!c.active) return;
    
    switch (c.type) {
        case ConstraintType::DISTANCE:
            solve_distance(bodies, c);
            break;
        case ConstraintType::BALL_SOCKET:
            solve_ball_socket(bodies, c);
            break;
        case ConstraintType::HINGE:
            solve_hinge(bodies, c);
            break;
        case ConstraintType::SLIDER:
            solve_slider(bodies, c);
            break;
        case ConstraintType::FIXED:
            solve_fixed(bodies, c);
            break;
        case ConstraintType::SPRING:
            solve_spring(bodies, c, dt);
            break;
        case ConstraintType::ANGULAR_SPRING:
            solve_angular_spring(bodies, c, dt);
            break;
        default:
            break;
    }
}

double ConstraintSolver::compute_constraint_error(const ConstraintParams& c,
                                                   const std::vector<Body>& bodies) {
    const Body* a = nullptr, * b = nullptr;
    for (const auto& body : bodies) {
        if (body.id == c.body_a) a = &body;
        if (body.id == c.body_b) b = &body;
    }
    if (!a || !b) return std::numeric_limits<double>::max();
    
    switch (c.type) {
        case ConstraintType::DISTANCE: {
            double dist = glm::length(b->state.position - a->state.position);
            return std::abs(dist - c.distance.rest_length);
        }
        case ConstraintType::BALL_SOCKET: {
            dvec3 aw = get_world_anchor(*a, c.hinge.anchor_a);
            dvec3 bw = get_world_anchor(*b, c.hinge.anchor_b);
            return glm::length(bw - aw);
        }
        default:
            return 0;
    }
}

// ==================== SoftBodySystem ====================

void SoftBodySystem::create_cloth(std::vector<Body>& bodies,
                                   const dvec3& origin, int width, int depth,
                                   double spacing, double mass,
                                   bool pin_corners) {
    constraints_.clear();
    
    // Create grid of bodies
    for (int z = 0; z < depth; z++) {
        for (int x = 0; x < width; x++) {
            BodyState state;
            state.position = origin + dvec3(x * spacing, 0, z * spacing);
            
            BodyProperties props;
            props.mass = mass;
            props.radius = spacing * 0.15;
            
            VisualProperties visual;
            visual.color = glm::vec3(0.0f, 0.8f, 0.6f);
            visual.shape = ShapeType::Sphere;
            
            uint32_t flags = BODY_VISIBLE | BODY_GRAVITY;
            // Pin corners
            if (pin_corners && 
                ((x == 0 && z == 0) || (x == width-1 && z == 0) ||
                 (x == 0 && z == depth-1) || (x == width-1 && z == depth-1))) {
                flags |= BODY_STATIC;
            }
            
            // We store the body ID indirectly via the return value
            // For now, assume bodies are added externally
        }
    }
    
    // Create structural springs (horizontal and vertical)
    for (int z = 0; z < depth; z++) {
        for (int x = 0; x < width; x++) {
            int idx = z * width + x;
            
            // Right neighbor
            if (x < width - 1) {
                ConstraintParams c;
                c.type = ConstraintType::DISTANCE;
                c.body_a = idx;
                c.body_b = idx + 1;
                c.distance.rest_length = spacing;
                c.distance.stiffness = 0.8;
                constraints_.push_back(c);
            }
            
            // Bottom neighbor
            if (z < depth - 1) {
                ConstraintParams c;
                c.type = ConstraintType::DISTANCE;
                c.body_a = idx;
                c.body_b = idx + width;
                c.distance.rest_length = spacing;
                c.distance.stiffness = 0.8;
                constraints_.push_back(c);
            }
            
            // Diagonal (shear)
            if (x < width - 1 && z < depth - 1) {
                double diag = std::sqrt(2.0) * spacing;
                ConstraintParams c;
                c.type = ConstraintType::DISTANCE;
                c.body_a = idx;
                c.body_b = idx + width + 1;
                c.distance.rest_length = diag;
                c.distance.stiffness = 0.5;
                constraints_.push_back(c);
                
                ConstraintParams c2;
                c2.type = ConstraintType::DISTANCE;
                c2.body_a = idx + 1;
                c2.body_b = idx + width;
                c2.distance.rest_length = diag;
                c2.distance.stiffness = 0.5;
                constraints_.push_back(c2);
            }
        }
    }
    
    // Bend springs (skip one)
    for (int z = 0; z < depth; z++) {
        for (int x = 0; x < width - 2; x++) {
            int idx = z * width + x;
            ConstraintParams c;
            c.type = ConstraintType::DISTANCE;
            c.body_a = idx;
            c.body_b = idx + 2;
            c.distance.rest_length = spacing * 2;
            c.distance.stiffness = 0.3;
            constraints_.push_back(c);
        }
    }
    for (int z = 0; z < depth - 2; z++) {
        for (int x = 0; x < width; x++) {
            int idx = z * width + x;
            ConstraintParams c;
            c.type = ConstraintType::DISTANCE;
            c.body_a = idx;
            c.body_b = idx + width * 2;
            c.distance.rest_length = spacing * 2;
            c.distance.stiffness = 0.3;
            constraints_.push_back(c);
        }
    }
}

void SoftBodySystem::create_chain(std::vector<Body>& bodies,
                                   const dvec3& start, const dvec3& end,
                                   int segments, double mass, double radius) {
    constraints_.clear();
    dvec3 dir = (end - start) / (double)segments;
    double length = glm::length(end - start);
    double seg_length = length / segments;
    
    for (int i = 0; i <= segments; i++) {
        BodyState state;
        state.position = start + dir * (double)i;
        
        BodyProperties props;
        props.mass = mass / segments;
        props.radius = radius;
        
        VisualProperties visual;
        visual.color = glm::vec3(0.0f, 0.5f + i * 0.05f, 0.8f);
        
        uint32_t flags = BODY_VISIBLE | BODY_GRAVITY;
        if (i == 0) flags |= BODY_STATIC; // Pin first body
    }
    
    // Connect segments
    for (int i = 0; i < segments; i++) {
        ConstraintParams c;
        c.type = ConstraintType::DISTANCE;
        c.body_a = i;
        c.body_b = i + 1;
        c.distance.rest_length = seg_length;
        c.distance.stiffness = 0.9;
        constraints_.push_back(c);
    }
}

void SoftBodySystem::create_jelly(std::vector<Body>& bodies,
                                   const dvec3& center, double radius,
                                   int shell_particles, double mass) {
    constraints_.clear();
    
    // Central body
    BodyState center_state;
    center_state.position = center;
    BodyProperties center_props;
    center_props.mass = mass * 0.3;
    center_props.radius = radius * 0.5;
    
    // Shell bodies
    for (int i = 0; i < shell_particles; i++) {
        double theta = (double)i / shell_particles * 2.0 * M_PI;
        double phi = std::acos(1.0 - 2.0 * (double)i / shell_particles);
        if (i == 0 || i == shell_particles - 1) phi = (double)i * 0.1;
        
        BodyState shell_state;
        shell_state.position = center + dvec3(
            radius * std::sin(phi) * std::cos(theta),
            radius * std::cos(phi),
            radius * std::sin(phi) * std::sin(theta)
        );
        
        // Connect to center
        ConstraintParams c;
        c.type = ConstraintType::DISTANCE;
        c.body_a = 0; // Center body
        c.body_b = i + 1;
        c.distance.rest_length = radius;
        c.distance.stiffness = 0.7;
        constraints_.push_back(c);
        
        // Connect neighbors on shell
        if (i > 0) {
            ConstraintParams c2;
            c2.type = ConstraintType::DISTANCE;
            c2.body_a = i;
            c2.body_b = i + 1;
            double angle = 2.0 * M_PI / shell_particles;
            c2.distance.rest_length = 2.0 * radius * std::sin(angle * 0.5);
            c2.distance.stiffness = 0.5;
            constraints_.push_back(c2);
        }
    }
    
    // Connect last to first
    if (shell_particles > 2) {
        ConstraintParams c;
        c.type = ConstraintType::DISTANCE;
        c.body_a = 1;
        c.body_b = shell_particles;
        double angle = 2.0 * M_PI / shell_particles;
        c.distance.rest_length = 2.0 * radius * std::sin(angle * 0.5);
        c.distance.stiffness = 0.5;
        constraints_.push_back(c);
    }
}

} // namespace hlp
