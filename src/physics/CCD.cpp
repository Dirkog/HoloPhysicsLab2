#include "CCD.h"
#include <algorithm>
#include <cmath>
#include <iostream>

namespace hlp {

CCDSolver::CCDSolver() = default;
CCDSolver::~CCDSolver() = default;

double CCDSolver::get_bounding_radius(const Body& b) {
    // Conservative bounding sphere for any shape
    switch (b.visual.shape) {
        case ShapeType::Sphere:
            return b.props.radius;
        case ShapeType::Box:
            return b.props.radius * 1.732;  // sqrt(3) for box diagonal
        case ShapeType::Cylinder:
            return std::sqrt(b.props.radius * b.props.radius + 
                             b.props.radius * b.props.radius * 0.25); // r² + (h/2)²
        case ShapeType::Torus:
            return b.props.radius;  // outer radius
        default:
            return b.props.radius;
    }
}

dvec3 CCDSolver::get_bounding_center(const Body& b) {
    return b.state.position;  // Use COM for all shapes
}

double CCDSolver::ray_sphere_toi(const dvec3& origin, const dvec3& dir,
                                   double radius, const dvec3& sphere_center,
                                   double sphere_radius) {
    dvec3 oc = origin - sphere_center;
    double a = glm::dot(dir, dir);
    double b = 2.0 * glm::dot(oc, dir);
    double c = glm::dot(oc, oc) - (radius + sphere_radius) * (radius + sphere_radius);
    
    double disc = b * b - 4.0 * a * c;
    if (disc < 0) return 1.0;  // No intersection
    
    double sqrt_disc = std::sqrt(disc);
    double t1 = (-b - sqrt_disc) / (2.0 * a);
    double t2 = (-b + sqrt_disc) / (2.0 * a);
    
    // Return earliest positive TOI
    if (t1 > 0 && t1 < 1.0) return t1;
    if (t2 > 0 && t2 < 1.0) return t2;
    return 1.0;  // No impact within [0,1]
}

double CCDSolver::ray_box_toi(const dvec3& origin, const dvec3& dir,
                               const dvec3& box_min, const dvec3& box_max) {
    // Slabs method
    double tmin = -1e10, tmax = 1e10;
    
    for (int i = 0; i < 3; i++) {
        if (std::abs(dir[i]) < 1e-10) {
            if (origin[i] < box_min[i] || origin[i] > box_max[i])
                return 1.0;
        } else {
            double t1 = (box_min[i] - origin[i]) / dir[i];
            double t2 = (box_max[i] - origin[i]) / dir[i];
            if (t1 > t2) std::swap(t1, t2);
            tmin = std::max(tmin, t1);
            tmax = std::min(tmax, t2);
            if (tmin > tmax) return 1.0;
        }
    }
    
    if (tmin < 0) tmin = 0;
    if (tmin < 1.0) return tmin;
    return 1.0;
}

CCDSolver::TOIResult CCDSolver::swept_sphere_sphere(const Body& a, const Body& b, double dt) {
    TOIResult result;
    
    dvec3 a_pos = get_bounding_center(a);
    dvec3 b_pos = get_bounding_center(b);
    dvec3 a_vel = a.state.velocity;
    dvec3 b_vel = b.state.velocity;
    dvec3 rel_vel = a_vel - b_vel;
    
    double a_radius = get_bounding_radius(a);
    double b_radius = get_bounding_radius(b);
    
    // Ray-sphere test: shrink B to point, grow A to sum of radii
    double toi = ray_sphere_toi(a_pos, rel_vel * dt, a_radius + b_radius, 
                                b_pos, 0.0);
    
    if (toi < 1.0) {
        result.toi = toi;
        result.body_a = a.id;
        result.body_b = b.id;
        result.has_impact = true;
        
        // Contact point and normal
        dvec3 sweep = rel_vel * dt * toi;
        dvec3 contact_a = a_pos + sweep;
        dvec3 delta = contact_a - b_pos;
        double dist = glm::length(delta);
        
        if (dist > 1e-10) {
            result.contact_normal = delta / dist;
            result.contact_point = b_pos + result.contact_normal * b_radius;
        }
    }
    
    return result;
}

CCDSolver::TOIResult CCDSolver::conservative_advancement(const Body& a, const Body& b, double dt) {
    TOIResult result;
    
    dvec3 a_pos = get_bounding_center(a);
    dvec3 b_pos = get_bounding_center(b);
    dvec3 a_vel = a.state.velocity;
    dvec3 b_vel = b.state.velocity;
    dvec3 rel_vel = a_vel - b_vel;
    double rel_speed = glm::length(rel_vel);
    
    // Only CCD for fast bodies
    if (rel_speed * dt < a.props.radius + b.props.radius) {
        // Standard collision detection will handle this
        return result;
    }
    
    double a_radius = get_bounding_radius(a);
    double b_radius = get_bounding_radius(b);
    double total_radius = a_radius + b_radius;
    
    dvec3 sweep_dir = rel_vel / rel_speed;
    double t = 0;
    int iter = 0;
    
    while (t < 1.0 && iter < config_.max_iterations) {
        iter++;
        
        dvec3 current_pos = a_pos + sweep_dir * rel_speed * t * dt;
        dvec3 delta = current_pos - b_pos;
        double dist = glm::length(delta);
        
        if (dist < total_radius) {
            // Collision detected
            result.toi = t;
            result.has_impact = true;
            result.body_a = a.id;
            result.body_b = b.id;
            
            if (dist > 1e-10) {
                result.contact_normal = delta / dist;
                result.contact_point = b_pos + result.contact_normal * b_radius;
            }
            return result;
        }
        
        // Advance by conservative step
        double advance = (dist - total_radius) / rel_speed / dt;
        advance = std::max(advance, config_.min_toi);
        t += advance;
    }
    
    return result;
}

void CCDSolver::solve(std::vector<Body>& bodies, double dt) {
    if (!config_.enabled) return;
    
    last_checks_ = 0;
    last_hits_ = 0;
    
    for (size_t i = 0; i < bodies.size(); i++) {
        if (bodies[i].is_static()) continue;
        
        double speed = glm::length(bodies[i].state.velocity);
        if (speed * dt < config_.ccd_threshold * bodies[i].props.radius)
            continue;  // Body is slow enough for standard collision detection
        
        for (size_t j = i + 1; j < bodies.size(); j++) {
            last_checks_++;
            
            if (bodies[j].is_static()) continue;
            
            TOIResult toi = conservative_advancement(bodies[i], bodies[j], dt);
            
            if (toi.has_impact) {
                last_hits_++;
                
                // Stop body at TOI and resolve collision
                double ti = toi.toi;
                bodies[i].state.position -= bodies[i].state.velocity * dt * (1.0 - ti);
                bodies[j].state.position -= bodies[j].state.velocity * dt * (1.0 - ti);
                
                // Apply elastic collision response at contact point
                dvec3 n = toi.contact_normal;
                dvec3 rel_v = bodies[i].state.velocity - bodies[j].state.velocity;
                double rel_vn = glm::dot(rel_v, n);
                
                if (rel_vn > 0) {
                    double e = (bodies[i].props.restitution + bodies[j].props.restitution) * 0.5;
                    double total_mass = bodies[i].props.mass + bodies[j].props.mass;
                    double impulse = (1.0 + e) * rel_vn / total_mass;
                    
                    bodies[i].state.velocity -= impulse * bodies[j].props.mass * n;
                    bodies[j].state.velocity += impulse * bodies[i].props.mass * n;
                }
            }
        }
    }
}

} // namespace hlp
