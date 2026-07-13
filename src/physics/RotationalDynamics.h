#pragma once

// ============================================================
// HoloPhysics Lab 2 — Rotational Dynamics
// Quaternion orientation, inertia tensors, torque integration
// ============================================================

#include "../core/Types.h"
#include <glm/gtc/quaternion.hpp>

namespace hlp {

struct InertiaTensor {
    double Ixx, Iyy, Izz;  // Principal moments of inertia
    double Ixy = 0, Ixz = 0, Iyz = 0;  // Products of inertia
    
    // Compute inverse
    InertiaTensor inverse() const;
    
    // Apply to vector: result = I * v
    dvec3 apply(const dvec3& v) const;
    
    // Factory methods for basic shapes
    static InertiaTensor sphere(double mass, double radius);
    static InertiaTensor box(double mass, double w, double h, double d);
    static InertiaTensor cylinder(double mass, double radius, double height);
    static InertiaTensor torus(double mass, double major, double minor);
};

// ==================== Rotational State ====================

struct RotationalState {
    dquat orientation{1.0, 0.0, 0.0, 0.0};  // Unit quaternion
    dvec3 angular_velocity{0.0};              // rad/s in world space
    dvec3 torque{0.0};                        // N*m in world space
    InertiaTensor inertia;                    // Body-fixed inertia tensor
    InertiaTensor inv_inertia_world;           // World-space inverse inertia
    
    void update_inv_inertia();
};

// ==================== Rotational Dynamics System ====================

class RotationalDynamics {
public:
    RotationalDynamics() = default;
    
    // Compute inertia tensor for a body based on its shape
    static InertiaTensor compute_inertia(const Body& body);
    
    // Compute torque on a body
    static dvec3 compute_torque(const Body& body, const SimulationParams& params);
    
    // Integrate rotation (semi-implicit Euler)
    static void integrate_rotation(Body& body, double dt);
    
    // Integrate with quaternion (RK4 for quaternions)
    static void integrate_rotation_rk4(Body& body, double dt);
    
    // Apply impulse at a point (produces both linear and angular effects)
    static void apply_impulse_at_point(Body& body, const dvec3& impulse,
                                       const dvec3& world_point);
    
    // Update world-space inverse inertia from orientation
    static void update_inertia(Body& body);
    
    // ============== Gyroscopic effects ==============
    
    // Compute gyroscopic torque
    static dvec3 gyroscopic_torque(const Body& body);
    
    // Apply gyroscopic precession (for gyroscope experiment)
    static void apply_gyroscopic_precession(Body& body, double dt);
    
    // ============== Utility ==============
    
    // Quaternion from axis-angle
    static dquat from_axis_angle(const dvec3& axis, double angle);
    
    // Convert angular velocity to quaternion derivative (dq/dt = 0.5 * ω ⊗ q)
    static dquat quaternion_derivative(const dquat& q, const dvec3& omega);
    
    // Normalize quaternion
    static void normalize_quat(dquat& q);
};

} // namespace hlp
