#include "RotationalDynamics.h"
#include <cmath>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtx/quaternion.hpp>

namespace hlp {

// ==================== InertiaTensor ====================

InertiaTensor InertiaTensor::inverse() const {
    double det = Ixx * (Iyy * Izz - Iyz * Iyz)
               - Ixy * (Ixy * Izz - Ixz * Iyz)
               + Ixz * (Ixy * Iyz - Ixz * Iyy);
    
    if (std::abs(det) < 1e-20) return InertiaTensor();  // Singular
    
    double inv_det = 1.0 / det;
    InertiaTensor inv;
    inv.Ixx = (Iyy * Izz - Iyz * Iyz) * inv_det;
    inv.Ixy = (Ixz * Iyz - Ixy * Izz) * inv_det;
    inv.Ixz = (Ixy * Iyz - Ixz * Iyy) * inv_det;
    inv.Iyy = (Ixx * Izz - Ixz * Ixz) * inv_det;
    inv.Iyz = (Ixz * Ixy - Ixx * Iyz) * inv_det;
    inv.Izz = (Ixx * Iyy - Ixy * Ixy) * inv_det;
    return inv;
}

dvec3 InertiaTensor::apply(const dvec3& v) const {
    return dvec3(
        Ixx * v.x + Ixy * v.y + Ixz * v.z,
        Ixy * v.x + Iyy * v.y + Iyz * v.z,
        Ixz * v.x + Iyz * v.y + Izz * v.z
    );
}

InertiaTensor InertiaTensor::sphere(double mass, double radius) {
    double I = 0.4 * mass * radius * radius;
    return {I, I, I, 0, 0, 0};
}

InertiaTensor InertiaTensor::box(double mass, double w, double h, double d) {
    return {
        mass / 12.0 * (h * h + d * d),  // Ixx
        mass / 12.0 * (w * w + d * d),  // Iyy
        mass / 12.0 * (w * w + h * h),  // Izz
        0, 0, 0
    };
}

InertiaTensor InertiaTensor::cylinder(double mass, double radius, double height) {
    return {
        mass / 12.0 * (3 * radius * radius + height * height),  // Ixx (perpendicular)
        mass / 12.0 * (3 * radius * radius + height * height),  // Izz
        0.5 * mass * radius * radius,  // Iyy (along axis)
        0, 0, 0
    };
}

InertiaTensor InertiaTensor::torus(double mass, double major, double minor) {
    double a = major, b = minor;
    double I_perp = mass * (0.75 * b * b + a * a);
    double I_axial = 0.5 * mass * (a * a + 0.75 * b * b);
    return {I_perp, I_axial, I_perp, 0, 0, 0};
}

// ==================== RotationalState ====================

void RotationalState::update_inv_inertia() {
    inv_inertia_world = inertia.inverse();
    // Apply orientation rotation to get world-space inertia
    // Simplified: for principal axes, just rotate
}

// ==================== RotationalDynamics ====================

InertiaTensor RotationalDynamics::compute_inertia(const Body& body) {
    double m = body.props.mass;
    double r = body.props.radius;
    
    switch (body.visual.shape) {
        case ShapeType::Sphere:
            return InertiaTensor::sphere(m, r);
        case ShapeType::Box:
            return InertiaTensor::box(m, r * 2, r * 2, r * 2);
        case ShapeType::Cylinder:
            return InertiaTensor::cylinder(m, r, r);
        case ShapeType::Torus:
            return InertiaTensor::torus(m, r * 0.7, r * 0.3);
        default:
            return InertiaTensor::sphere(m, r);
    }
}

dvec3 RotationalDynamics::compute_torque(const Body& body,
                                          const SimulationParams& params) {
    dvec3 torque(0.0);
    
    // Magnetic torque (if body has magnetic moment)
    if (body.props.magnetic_moment > 0 && params.enable_magnet) {
        // τ = μ × B
        dvec3 B(0, params.magnetic_field, 0);
        torque += glm::cross(dvec3(0, body.props.magnetic_moment, 0), B);
    }
    
    // Damping torque (viscous rotational damping)
    if (params.damping > 0) {
        torque -= body.state.angular_velocity * params.damping * 0.1;
    }
    
    return torque;
}

void RotationalDynamics::integrate_rotation(Body& body, double dt) {
    if (body.is_static()) return;
    
    // Semi-implicit Euler for rotation
    // τ = I * α → α = I⁻¹ * τ
    InertiaTensor I = compute_inertia(body);
    dvec3 alpha = I.inverse().apply(RotationalDynamics::compute_torque(body, SimulationParams()));
    
    // Update angular velocity
    body.state.angular_velocity += alpha * dt;
    
    // Gyroscopic torque
    body.state.angular_velocity += gyroscopic_torque(body) * dt;
    
    // Update orientation (quaternion)
    // q' = 0.5 * ω * q (in quaternion algebra)
    dquat q = body.state.orientation;
    dquat dq = 0.5 * dquat(0, body.state.angular_velocity.x, 
                             body.state.angular_velocity.y,
                             body.state.angular_velocity.z) * q;
    
    q += dq * dt;
    normalize_quat(q);
    body.state.orientation = q;
}

void RotationalDynamics::integrate_rotation_rk4(Body& body, double dt) {
    if (body.is_static()) return;
    
    // RK4 for quaternion integration
    auto deriv = [](const dquat& q, const dvec3& w) -> dquat {
        dquat w_quat(0, w.x, w.y, w.z);
        return 0.5 * w_quat * q;
    };
    
    dquat q = body.state.orientation;
    dvec3 w = body.state.angular_velocity;
    
    dquat k1 = deriv(q, w);
    dquat k2 = deriv(q + 0.5 * dt * k1, w);
    dquat k3 = deriv(q + 0.5 * dt * k2, w);
    dquat k4 = deriv(q + dt * k3, w);
    
    body.state.orientation = q + (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4);
    normalize_quat(body.state.orientation);
}

void RotationalDynamics::apply_impulse_at_point(Body& body, const dvec3& impulse,
                                                 const dvec3& world_point) {
    if (body.is_static()) return;
    
    // Linear part
    body.state.velocity += impulse / body.props.mass;
    
    // Angular part: L = r × p
    dvec3 r = world_point - body.state.position;
    dvec3 angular_impulse = glm::cross(r, impulse);
    
    InertiaTensor I = compute_inertia(body);
    body.state.angular_velocity += I.inverse().apply(angular_impulse);
}

void RotationalDynamics::update_inertia(Body& body) {
    // Update world-space inertia based on orientation
    // (simplified - for complex shapes, this needs rotation of inertia tensor)
}

dvec3 RotationalDynamics::gyroscopic_torque(const Body& body) {
    // τ = -ω × (I * ω)
    InertiaTensor I = compute_inertia(body);
    dvec3 Iw = I.apply(body.state.angular_velocity);
    return -glm::cross(body.state.angular_velocity, Iw);
}

void RotationalDynamics::apply_gyroscopic_precession(Body& body, double dt) {
    // For gyroscope: precession = torque / (I * ω * sin(θ))
    if (glm::length(body.state.angular_velocity) < 1e-10) return;
    
    dvec3 w = body.state.angular_velocity;
    dvec3 torque = gyroscopic_torque(body);
    
    // Precession angular velocity
    double Iw = compute_inertia(body).apply(w).y; // Component along Y
    if (std::abs(Iw) > 1e-10) {
        double precession_rate = glm::length(torque) / std::abs(Iw);
        body.state.angular_velocity.y += precession_rate * dt;
    }
}

dquat RotationalDynamics::from_axis_angle(const dvec3& axis, double angle) {
    double half = angle * 0.5;
    double s = std::sin(half);
    return dquat(std::cos(half), axis.x * s, axis.y * s, axis.z * s);
}

void RotationalDynamics::normalize_quat(dquat& q) {
    double len = std::sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
    if (len > 1e-10) {
        q /= len;
    } else {
        q = dquat(1.0, 0.0, 0.0, 0.0);
    }
}

} // namespace hlp
