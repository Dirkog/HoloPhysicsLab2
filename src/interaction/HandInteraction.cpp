#include "HandInteraction.h"
#include <algorithm>
#include <limits>
#include <iostream>

namespace hlp {

HandInteractionSystem::HandInteractionSystem(Engine& engine, 
                                             ConstraintSolver& constraint_solver)
    : engine_(engine), constraints_(constraint_solver) {
    for (int i = 0; i < MAX_HANDS; i++) {
        is_grabbing_[i] = false;
        grabbed_body_[i] = UINT64_MAX;
        grab_constraint_idx_[i] = -1;
        last_hand_pos_[i] = dvec3(0);
    }
}

HandInteractionSystem::~HandInteractionSystem() {
    release_all();
}

void HandInteractionSystem::update(const GestureController& gesture_ctrl, double dt) {
    // Reset debug
    debug_.affected_bodies.clear();
    debug_.show_ray = false;
    debug_.show_force_field = false;
    
    // Process each hand
    auto& right = const_cast<VirtualHand&>(gesture_ctrl.right_hand());
    auto& left = const_cast<VirtualHand&>(gesture_ctrl.left_hand());
    
    if (right.is_active()) process_hand(right, 0, dt);
    if (left.is_active()) process_hand(left, 1, dt);
}

void HandInteractionSystem::process_hand(VirtualHand& hand, int hand_idx, double dt) {
    auto gesture = hand.gesture();
    auto& interaction = hand.interaction();
    
    switch (gesture) {
        case HandGesture::PINCH_CLOSE:
            try_grab(hand, hand_idx);
            break;
            
        case HandGesture::PINCH:
            if (interaction.is_grabbing) {
                update_grab(hand, hand_idx, dt);
            }
            break;
            
        case HandGesture::PINCH_OPEN:
            if (interaction.is_grabbing) {
                apply_throw(hand, hand_idx);
                release_grab(hand, hand_idx);
            }
            break;
            
        case HandGesture::FIST:
            apply_repulsion(hand);
            break;
            
        case HandGesture::POINTING:
            if (mode_ == InteractionMode::RAY_SELECT || mode_ == InteractionMode::MIXED) {
                raycast_select(hand);
            }
            break;
            
        case HandGesture::OPEN_PALM:
            if (mode_ == InteractionMode::FORCE_FIELD || mode_ == InteractionMode::MIXED) {
                apply_force_field(hand);
            }
            break;
            
        default:
            // UNKNOWN etc.: clear highlight
            highlighted_body_ = UINT64_MAX;
            interaction.hovered_body_id = UINT64_MAX;
            debug_.show_ray = false;
            break;
    }
}

// ==================== GRAB SYSTEM ====================

uint64_t HandInteractionSystem::find_nearest_body(const dvec3& point, double radius,
                                                    uint64_t exclude) {
    uint64_t nearest = UINT64_MAX;
    double nearest_dist = radius;
    
    for (const auto& body : engine_.scene().bodies()) {
        if (body.is_static() || body.id == exclude) continue;
        double dist = glm::length(body.state.position - point);
        if (dist < nearest_dist && dist < body.props.radius * 3.0) {
            nearest_dist = dist;
            nearest = body.id;
        }
    }
    return nearest;
}

void HandInteractionSystem::try_grab(VirtualHand& hand, int hand_idx) {
    if (is_grabbing_[hand_idx]) return;
    
    // Find body near index finger tip
    uint64_t target = find_nearest_body(hand.index_tip(), config_.grab_radius);
    if (target == UINT64_MAX) return;
    
    auto* body = engine_.scene().get_body(target);
    if (!body) return;
    
    // Create a virtual spring constraint between hand and body
    // Method 1: Using the existing ConstraintSolver
    ConstraintParams c;
    c.type = ConstraintType::DISTANCE;
    c.body_a = target;
    c.body_b = target;  // Will be linked to virtual anchor
    c.distance.rest_length = 0.05;  // Very short = rigid
    c.distance.stiffness = config_.grab_stiffness;
    
    // Instead of proper virtual body constraint (which needs refactoring),
    // use a simpler approach: direct position manipulation
    // (adequate for POC, can be upgraded to full constraint later)
    
    // Store grab offset
    hand.interaction().grab_offset = body->state.position - hand.index_tip();
    hand.interaction().grab_start_time = engine_.scene().time();
    
    // Mark as grabbed
    is_grabbing_[hand_idx] = true;
    grabbed_body_[hand_idx] = target;
    hand.interaction().is_grabbing = true;
    hand.interaction().grabbed_body_id = target;
    last_hand_pos_[hand_idx] = hand.index_tip();
    
    // Visual feedback
    body->flags |= BODY_SELECTED;
    highlighted_body_ = target;
    debug_.interaction_point = hand.index_tip();
    
    std::cout << "[Interaction] Grabbed body #" << target << std::endl;
}

void HandInteractionSystem::update_grab(VirtualHand& hand, int hand_idx, double dt) {
    if (!is_grabbing_[hand_idx]) return;
    
    auto* body = engine_.scene().get_body(grabbed_body_[hand_idx]);
    if (!body) { release_grab(hand, hand_idx); return; }
    
    // Spring-like movement: body follows hand position
    dvec3 target_pos = hand.index_tip(); // + hand.interaction().grab_offset;
    dvec3 delta = target_pos - body->state.position;
    
    // Stiff spring (critical damping)
    double k = config_.grab_stiffness * dt;
    double damping = config_.grab_damping * dt;
    
    body->state.position += delta * std::min(k, 1.0);
    body->state.velocity = (body->state.position - last_hand_pos_[hand_idx]) / std::max(dt, 1e-6);
    
    // Update last position for velocity calculation
    last_hand_pos_[hand_idx] = body->state.position;
    
    // Debug
    debug_.interaction_point = hand.index_tip();
}

void HandInteractionSystem::release_grab(VirtualHand& hand, int hand_idx) {
    if (!is_grabbing_[hand_idx]) return;
    
    // Remove constraint if any
    if (grab_constraint_idx_[hand_idx] >= 0) {
        constraints_.remove_constraint(grab_constraint_idx_[hand_idx]);
        grab_constraint_idx_[hand_idx] = -1;
        grab_constraint_count_--;
    }
    
    // Unhighlight
    auto* body = engine_.scene().get_body(grabbed_body_[hand_idx]);
    if (body) body->flags &= ~BODY_SELECTED;
    
    // Reset state
    is_grabbing_[hand_idx] = false;
    grabbed_body_[hand_idx] = UINT64_MAX;
    hand.interaction().is_grabbing = false;
    hand.interaction().grabbed_body_id = UINT64_MAX;
    highlighted_body_ = UINT64_MAX;
    
    std::cout << "[Interaction] Released" << std::endl;
}

void HandInteractionSystem::apply_throw(VirtualHand& hand, int hand_idx) {
    if (!is_grabbing_[hand_idx]) return;
    
    auto* body = engine_.scene().get_body(grabbed_body_[hand_idx]);
    if (!body) return;
    
    // Apply hand velocity × gain to body
    dvec3 throw_vel = hand.smoothed_velocity() * config_.throw_gain;
    
    // Minimum throw velocity (prevents "dropping" with no velocity)
    if (glm::length(throw_vel) < 0.5) {
        throw_vel = body->state.velocity * 0.5; // Keep existing momentum
    }
    
    body->state.velocity += throw_vel;
    
    // Debug
    std::cout << "[Interaction] Threw body #" << grabbed_body_[hand_idx] 
              << " with velocity (" << throw_vel.x << ", " 
              << throw_vel.y << ", " << throw_vel.z << ")" << std::endl;
}

void HandInteractionSystem::release_all() {
    for (int i = 0; i < MAX_HANDS; i++) {
        if (is_grabbing_[i]) {
            auto* body = engine_.scene().get_body(grabbed_body_[i]);
            if (body) body->flags &= ~BODY_SELECTED;
        }
        is_grabbing_[i] = false;
        grabbed_body_[i] = UINT64_MAX;
        if (grab_constraint_idx_[i] >= 0) {
            constraints_.remove_constraint(grab_constraint_idx_[i]);
            grab_constraint_idx_[i] = -1;
        }
    }
    grab_constraint_count_ = 0;
    highlighted_body_ = UINT64_MAX;
}

// ==================== FORCE FIELDS ====================

void HandInteractionSystem::apply_repulsion(VirtualHand& hand) {
    // Fist → push everything away from palm center
    dvec3 center = hand.palm_center();
    double radius = config_.repulsion_radius;
    
    for (auto& body : engine_.scene().bodies()) {
        if (body.is_static()) continue;
        
        dvec3 delta = body.state.position - center;
        double dist = glm::length(delta);
        
        if (dist < radius && dist > 0.01) {
            double force = config_.repulsion_force * (1.0 - dist / radius);
            body.state.velocity += (delta / dist) * force * 0.02;
            debug_.affected_bodies.push_back(body.id);
        }
    }
    
    debug_.show_force_field = true;
    debug_.interaction_point = center;
}

void HandInteractionSystem::apply_force_field(VirtualHand& hand) {
    // Open palm → gentle attraction/repulsion field
    dvec3 center = hand.palm_center();
    double radius = config_.repulsion_radius * 1.5;
    double strength = config_.force_field_strength;
    
    for (auto& body : engine_.scene().bodies()) {
        if (body.is_static()) continue;
        
        dvec3 delta = body.state.position - center;
        double dist = glm::length(delta);
        
        if (dist < radius && dist > 0.01) {
            // Gentle vortex-like force
            double force = strength * (1.0 - dist / radius) * 0.01;
            // Tangential component (vortex)
            dvec3 tangent = dvec3(-delta.z, 0, delta.x);
            body.state.velocity += (tangent / dist) * force * 0.5;
            // Slight attraction toward palm
            body.state.velocity += (delta / dist) * force * 0.3;
            debug_.affected_bodies.push_back(body.id);
        }
    }
    
    debug_.show_force_field = true;
    debug_.interaction_point = center;
}

// ==================== RAYCAST SELECT ====================

void HandInteractionSystem::raycast_select(VirtualHand& hand) {
    // Pointing → ray from index tip along forward direction
    dvec3 origin = hand.index_tip();
    dvec3 dir = glm::normalize(hand.palm_center() - hand.index_tip());
    
    // If direction is degenerate, use a default
    if (glm::length(dir) < 0.001) {
        dir = dvec3(0, 0, -1);
    }
    
    double max_dist = config_.raycast_max_dist;
    uint64_t hit_id = UINT64_MAX;
    double hit_dist = max_dist;
    dvec3 hit_point(0);
    
    for (const auto& body : engine_.scene().bodies()) {
        // Sphere-ray intersection
        dvec3 oc = body.state.position - origin;
        double t = glm::dot(oc, dir);
        if (t < 0) continue;
        
        dvec3 projection = origin + dir * t;
        double dist = glm::length(projection - body.state.position);
        
        if (dist < body.props.radius && t < hit_dist) {
            hit_dist = t;
            hit_id = body.id;
            hit_point = projection;
        }
    }
    
    // Update state
    highlighted_body_ = hit_id;
    hand.interaction().hovered_body_id = hit_id;
    hand.interaction().hover_point = hit_point;
    
    // Visual feedback
    if (hit_id != UINT64_MAX) {
        auto* body = engine_.scene().get_body(hit_id);
        if (body) body->flags |= BODY_SELECTED;
        
        // Unhighlight others
        for (auto& b : engine_.scene().bodies()) {
            if (b.id != hit_id) b.flags &= ~BODY_SELECTED;
        }
    }
    
    // Debug ray
    debug_.show_ray = true;
    debug_.interaction_point = origin;
    debug_.ray_end = origin + dir * max_dist;
}

// ==================== FORCE APPLICATION HELPERS ====================

void HandInteractionSystem::apply_force_at_point(const dvec3& point, 
                                                   const dvec3& force, double radius) {
    for (auto& body : engine_.scene().bodies()) {
        if (body.is_static()) continue;
        double dist = glm::length(body.state.position - point);
        if (dist < radius && dist > 0.01) {
            double strength = 1.0 - dist / radius;
            body.state.velocity += force * strength / body.props.mass;
        }
    }
}

void HandInteractionSystem::grab_body_at_point(uint64_t body_id) {
    // Scripting API: grab a specific body
    for (int i = 0; i < MAX_HANDS; i++) {
        if (!is_grabbing_[i]) {
            grabbed_body_[i] = body_id;
            is_grabbing_[i] = true;
            
            auto* body = engine_.scene().get_body(body_id);
            if (body) {
                body->flags |= BODY_SELECTED;
                last_hand_pos_[i] = body->state.position;
            }
            
            std::cout << "[Interaction] Script grab body #" << body_id << std::endl;
            return;
        }
    }
}

} // namespace hlp
