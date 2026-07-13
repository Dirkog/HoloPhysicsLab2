#include "VirtualCoupling.h"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>

namespace hlp {

VirtualCoupling::VirtualCoupling(Engine& engine) : engine_(engine) {
    for (int i = 0; i < 2; i++) {
        hand_bodies_[i].mass = config_.hand_mass;
        hand_bodies_[i].radius = config_.hand_radius;
    }
}

VirtualCoupling::~VirtualCoupling() = default;

void VirtualCoupling::update(const VirtualHand& hand, double dt) {
    if (!hand.is_active()) {
        for (auto& hb : hand_bodies_) {
            hb.state.velocity *= 0.9;
            if (glm::length(hb.state.velocity) < 0.01)
                hb.state.velocity = dvec3(0);
        }
        active_ = false;
        return;
    }

    active_ = true;
    int idx = hand.is_right_hand ? 0 : 1;
    auto& hb = hand_bodies_[idx];
    hb.enabled = true;
    
    // 1. Update target from CV
    hb.target.position = hand.palm_center();
    
    static dvec3 prev_palm(0);
    hb.target.velocity = (hand.palm_center() - prev_palm) / std::max(dt, 1e-6);
    prev_palm = hand.palm_center();
    
    double target_speed = glm::length(hb.target.velocity);
    if (target_speed > config_.max_correction_speed) {
        hb.target.velocity *= config_.max_correction_speed / target_speed;
    }
    
    // 2. Anti-Slingshot PD force
    compute_pd_force(hb, hb.target.position, hb.target.velocity, dt);
    
    // 3. Integrate virtual hand position
    dvec3 acceleration = hb.force / hb.mass;
    hb.state.velocity += acceleration * dt;
    hb.state.velocity *= 0.98;
    
    double vel_mag = glm::length(hb.state.velocity);
    if (vel_mag > config_.max_correction_speed) {
        hb.state.velocity *= config_.max_correction_speed / vel_mag;
    }
    hb.state.position += hb.state.velocity * dt;
    
    // 4. Collision detection
    detect_hand_collisions(hb);
    apply_contact_forces(hb, dt);
    
    last_force_ = hb.force;
}

void VirtualCoupling::compute_pd_force(VirtualHandBody& hb,
                                        const dvec3& target_pos,
                                        const dvec3& target_vel,
                                        double dt) {
    dvec3 error = target_pos - hb.state.position;
    double dist_sq = glm::length2(error);
    
    // ===== ANTI-SLINGSHOT PROTECTION =====
    // If physical hand is stretched >12cm from target (e.g. hitting wall)
    // enter slip mode: reduce stiffness, increase damping
    const double MAX_STRETCH = 0.12;
    const double MAX_STRETCH_SQ = MAX_STRETCH * MAX_STRETCH;
    
    double current_kp = config_.kp_position;
    double current_kd = config_.kd_velocity;
    
    if (dist_sq > MAX_STRETCH_SQ) {
        double dist = std::sqrt(dist_sq);
        error = (error / dist) * MAX_STRETCH;  // Clamp error vector
        
        double slip = MAX_STRETCH / dist;       // [0..1]
        current_kp *= (slip * slip);             // Kp → 0 quadratically
        current_kd *= 2.0;                       // Double damping
        
        // Emergency velocity damping
        hb.state.velocity *= 0.1;
    }
    
    dvec3 vel_error = target_vel - hb.state.velocity;
    hb.force = error * current_kp + vel_error * current_kd;
    
    // Force clamping
    double force_mag = glm::length(hb.force);
    if (force_mag > config_.max_force) {
        hb.force *= config_.max_force / force_mag;
    }
    
    // CFL check: predict next velocity
    dvec3 accel = hb.force / hb.mass;
    dvec3 next_vel = hb.state.velocity + accel * dt;
    double next_speed = glm::length(next_vel);
    
    if (next_speed > config_.max_correction_speed) {
        dvec3 clamped = (next_vel / next_speed) * config_.max_correction_speed;
        hb.force = (clamped - hb.state.velocity) * hb.mass / dt;
    }
}

void VirtualCoupling::detect_hand_collisions(VirtualHandBody& hb) {
    contacting_.clear();
    max_penetration_ = 0;
    if (!hb.enabled) return;
    
    for (const auto& body : engine_.scene().bodies()) {
        if (body.is_static()) continue;
        dvec3 delta = body.state.position - hb.state.position;
        double dist = glm::length(delta);
        double min_dist = body.props.radius + hb.radius;
        if (dist < min_dist && dist > 0.001) {
            max_penetration_ = std::max(max_penetration_, min_dist - dist);
            contacting_.push_back(body.id);
        }
    }
}

void VirtualCoupling::apply_contact_forces(VirtualHandBody& hb, double dt) {
    if (!hb.enabled) return;
    
    // Momentum conservation: track impulse applied to bodies
    dvec3 total_impulse(0);
    
    for (auto body_id : contacting_) {
        auto* body = engine_.scene().get_body(body_id);
        if (!body || body->is_static()) continue;
        
        dvec3 delta = body->state.position - hb.state.position;
        double dist = glm::length(delta);
        if (dist < 0.001) continue;
        
        double min_dist = body->props.radius + hb.radius;
        double penetration = min_dist - dist;
        
        if (penetration > 0) {
            dvec3 normal = delta / dist;
            double correction = penetration * 0.5;
            body->state.position += normal * correction;
            
            dvec3 rel_vel = hb.state.velocity - body->state.velocity;
            double rel_vn = glm::dot(rel_vel, normal);
            
            if (rel_vn > 0) {
                double e = body->props.restitution;
                double reduced_mass = 1.0 / (1.0/body->props.mass + 1.0/hb.mass);
                double impulse = (1.0 + e) * rel_vn * reduced_mass;
                
                dvec3 impulse_vec = normal * impulse / body->props.mass;
                body->state.velocity += impulse_vec;
                total_impulse -= impulse_vec * body->props.mass;
                
                dvec3 tangent = rel_vel - rel_vn * normal;
                double t_len = glm::length(tangent);
                if (t_len > 0.001) {
                    double jt = std::min(body->props.friction * impulse, t_len * reduced_mass);
                    body->state.velocity -= jt * tangent / (t_len * body->props.mass);
                }
            }
        }
    }
    
    // Apply reaction impulse to virtual hand (momentum conservation)
    if (!contacting_.empty()) {
        hb.state.velocity += total_impulse / hb.mass;
    }
}

// ==================== SoftGrabController ====================

uint64_t SoftGrabController::find_nearest(const dvec3& point, double radius,
                                           const std::vector<Body>& bodies) {
    uint64_t nearest = UINT64_MAX;
    double nearest_dist = radius;
    for (const auto& b : bodies) {
        if (b.is_static()) continue;
        double dist = glm::length(b.state.position - point);
        double threshold = b.props.radius + radius;
        if (dist < threshold && dist < nearest_dist) {
            nearest_dist = dist;
            nearest = b.id;
        }
    }
    return nearest;
}

SoftGrabController::GrabData SoftGrabController::update(
    const VirtualHand& hand,
    const VirtualHandBody& hand_body,
    const std::vector<Body>& scene_bodies,
    double dt) {
    
    dvec3 hand_pos = hand_body.state.position;
    dvec3 hand_vel = hand_body.state.velocity;
    
    switch (grab_data_.state) {
        case IDLE:
            if (hand.gesture() == HandGesture::PINCH_CLOSE ||
                hand.gesture() == HandGesture::PINCH) {
                uint64_t target = find_nearest(hand_pos, config_.grab_radius, scene_bodies);
                if (target != UINT64_MAX) {
                    grab_data_.state = REACHING;
                    grab_data_.grabbed_body = target;
                    for (const auto& b : scene_bodies)
                        if (b.id == target)
                            grab_data_.grab_offset = b.state.position - hand_pos;
                }
            }
            break;
            
        case REACHING:
        case GRABBING:
        case HOLDING:
            if (hand.gesture() != HandGesture::PINCH) {
                grab_data_.state = RELEASING;
                grab_data_.release_velocity = hand_vel * config_.release_velocity_gain;
            } else {
                for (const auto& b : scene_bodies) {
                    if (b.id == grab_data_.grabbed_body) {
                        double dist = glm::length(b.state.position - hand_pos);
                        if (dist > config_.release_radius) {
                            grab_data_.state = RELEASING;
                            grab_data_.release_velocity = hand_vel * 0.5;
                        }
                        break;
                    }
                }
                if (grab_data_.state == REACHING) {
                    for (const auto& b : scene_bodies) {
                        if (b.id == grab_data_.grabbed_body) {
                            if (glm::length(b.state.position - hand_pos) < config_.grab_radius * 0.8)
                                grab_data_.state = GRABBING;
                            break;
                        }
                    }
                }
                if (grab_data_.state == GRABBING || grab_data_.state == HOLDING) {
                    grab_data_.state = HOLDING;
                }
            }
            break;
            
        case RELEASING:
            grab_data_.state = IDLE;
            grab_data_.grabbed_body = UINT64_MAX;
            break;
    }
    return grab_data_;
}

void SoftGrabController::apply_grab_force(Body& body, const GrabData& grab,
                                           const dvec3& hand_pos,
                                           const dvec3& hand_vel,
                                           double dt) {
    if (grab.state != HOLDING && grab.state != GRABBING) return;
    
    dvec3 target_pos = hand_pos + grab.grab_offset;
    
    // Spring-like follow with critical damping
    dvec3 error = target_pos - body.state.position;
    dvec3 vel_error = hand_vel - body.state.velocity;
    
    double kp = config_.grab_stiffness;
    double kd = 2.0 * std::sqrt(kp * body.props.mass); // Critical damping
    
    dvec3 spring_accel = error * kp + vel_error * kd;
    spring_accel /= body.props.mass;
    
    // Semi-implicit Euler
    body.state.velocity += spring_accel * dt;
    body.state.position += body.state.velocity * dt;
    
    // Add release velocity on last frame
    if (grab.state == RELEASING) {
        body.state.velocity += grab.release_velocity;
    }
}

} // namespace hlp
