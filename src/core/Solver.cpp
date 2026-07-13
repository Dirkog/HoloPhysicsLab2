#define _USE_MATH_DEFINES
#include <cmath>
#include "Solver.h"
#include <cmath>
#include <cstring>
#include <algorithm>

namespace hlp {

// ===================== Integrator Names =====================
const char* Solver::integrator_name(IntegratorType t) {
    switch (t) {
        case IntegratorType::Euler:         return "Явный Эйлер (1-й порядок)";
        case IntegratorType::SymplecticEuler: return "Симплектический Эйлер (1-й порядок)";
        case IntegratorType::Verlet:        return "Velocity Verlet (2-й порядок)";
        case IntegratorType::RK4:           return "Рунге-Кутты 4 (4-й порядок)";
        case IntegratorType::RK5:           return "Рунге-Кутты-Фельберг (5-й порядок)";
        case IntegratorType::Ruth:          return "Рут (3-й порядок, симплектический)";
        default:                            return "Неизвестный";
    }
}

const char** Solver::integrator_names() {
    static const char* names[] = {
        "Явный Эйлер (1-й порядок)",
        "Симплектический Эйлер (1-й порядок)",
        "Velocity Verlet (2-й порядок)",
        "Рунге-Кутты 4 (4-й порядок)",
        "Рунге-Кутты-Фельберг (5-й порядок)",
        "Рут (3-й порядок, симплектический)",
        nullptr
    };
    return names;
}

int Solver::integrator_count() { return 6; }

void Solver::set_integrator(IntegratorType type) {
    integrator_ = type;
}

void Solver::step(StateVector& state, DerivFunction derivs,
                   double& t, double dt) {
    switch (integrator_) {
        case IntegratorType::Euler:          step_euler(state, derivs, t, dt); break;
        case IntegratorType::SymplecticEuler:step_symplectic(state, derivs, t, dt); break;
        case IntegratorType::Verlet:         step_verlet(state, derivs, t, dt); break;
        case IntegratorType::RK4:            step_rk4(state, derivs, t, dt); break;
        case IntegratorType::RK5:            step_rk5(state, derivs, t, dt); break;
        case IntegratorType::Ruth:           step_ruth(state, derivs, t, dt); break;
    }
}

// ---- Euler (явный) ----
void Solver::step_euler(StateVector& s, DerivFunction f, double& t, double dt) {
    StateVector k(s.size(), 0.0);
    f(s, k, t);
    for (size_t i = 0; i < s.size(); ++i) s[i] += k[i] * dt;
    t += dt;
}

// ---- Symplectic Euler (сначала скорость, потом позиция) ----
void Solver::step_symplectic(StateVector& s, DerivFunction f, double& t, double dt) {
    size_t n = s.size() / 2;
    // Compute accelerations
    StateVector accel(n, 0.0);
    StateVector tmp(s.size(), 0.0);
    f(s, tmp, t);
    for (size_t i = 0; i < n; ++i) accel[i] = tmp[n + i];
    
    // Update velocities (first half of state = pos, second half = vel)
    for (size_t i = 0; i < n; ++i) {
        s[n + i] += accel[i] * dt;     // v += a * dt
        s[i] += s[n + i] * dt;          // x += v_new * dt
    }
    t += dt;
}

// ---- Velocity Verlet ----
void Solver::step_verlet(StateVector& s, DerivFunction f, double& t, double dt) {
    size_t n = s.size() / 2;
    
    // Current accelerations
    StateVector a0(n, 0.0);
    StateVector tmp0(s.size(), 0.0);
    f(s, tmp0, t);
    for (size_t i = 0; i < n; ++i) a0[i] = tmp0[n + i];
    
    // Update positions
    for (size_t i = 0; i < n; ++i) {
        s[i] += s[n + i] * dt + 0.5 * a0[i] * dt * dt;
    }
    
    // Update velocities (half-step approximation)
    for (size_t i = 0; i < n; ++i) {
        s[n + i] += 0.5 * a0[i] * dt;
    }
    
    // New accelerations at t+dt
    StateVector a1(n, 0.0);
    StateVector tmp1(s.size(), 0.0);
    f(s, tmp1, t + dt);
    for (size_t i = 0; i < n; ++i) a1[i] = tmp1[n + i];
    
    // Final velocity update
    for (size_t i = 0; i < n; ++i) {
        s[n + i] += 0.5 * a1[i] * dt;
    }
    t += dt;
}

// ---- RK4 ----
void Solver::step_rk4(StateVector& s, DerivFunction f, double& t, double dt) {
    size_t sz = s.size();
    StateVector k1(sz, 0.0), k2(sz, 0.0), k3(sz, 0.0), k4(sz, 0.0);
    StateVector tmp(sz, 0.0);
    
    f(s, k1, t);
    
    for (size_t i = 0; i < sz; ++i) tmp[i] = s[i] + 0.5 * dt * k1[i];
    f(tmp, k2, t + 0.5 * dt);
    
    for (size_t i = 0; i < sz; ++i) tmp[i] = s[i] + 0.5 * dt * k2[i];
    f(tmp, k3, t + 0.5 * dt);
    
    for (size_t i = 0; i < sz; ++i) tmp[i] = s[i] + dt * k3[i];
    f(tmp, k4, t + dt);
    
    for (size_t i = 0; i < sz; ++i) {
        s[i] += (dt / 6.0) * (k1[i] + 2.0 * k2[i] + 2.0 * k3[i] + k4[i]);
    }
    t += dt;
}

// ---- RK5 (Fehlberg) ----
void Solver::step_rk5(StateVector& s, DerivFunction f, double& t, double dt) {
    size_t sz = s.size();
    StateVector k1(sz,0.0), k2(sz,0.0), k3(sz,0.0), k4(sz,0.0), k5(sz,0.0), k6(sz,0.0);
    StateVector tmp(sz, 0.0);
    
    // Butcher tableau for RKF45
    f(s, k1, t);
    
    for (size_t i = 0; i < sz; ++i) tmp[i] = s[i] + dt * (0.25 * k1[i]);
    f(tmp, k2, t + 0.25 * dt);
    
    for (size_t i = 0; i < sz; ++i) tmp[i] = s[i] + dt * (3.0/32.0 * k1[i] + 9.0/32.0 * k2[i]);
    f(tmp, k3, t + 3.0/8.0 * dt);
    
    for (size_t i = 0; i < sz; ++i) tmp[i] = s[i] + dt * (1932.0/2197.0 * k1[i] - 7200.0/2197.0 * k2[i] + 7296.0/2197.0 * k3[i]);
    f(tmp, k4, t + 12.0/13.0 * dt);
    
    for (size_t i = 0; i < sz; ++i) tmp[i] = s[i] + dt * (439.0/216.0 * k1[i] - 8.0 * k2[i] + 3680.0/513.0 * k3[i] - 845.0/4104.0 * k4[i]);
    f(tmp, k5, t + dt);
    
    for (size_t i = 0; i < sz; ++i) tmp[i] = s[i] + dt * (-8.0/27.0 * k1[i] + 2.0 * k2[i] - 3544.0/2565.0 * k3[i] + 1859.0/4104.0 * k4[i] - 11.0/40.0 * k5[i]);
    f(tmp, k6, t + 0.5 * dt);
    
    // 5th order update
    for (size_t i = 0; i < sz; ++i) {
        s[i] += dt * (16.0/135.0 * k1[i] + 6656.0/12825.0 * k3[i] + 28561.0/56430.0 * k4[i] 
                      - 9.0/50.0 * k5[i] + 2.0/55.0 * k6[i]);
    }
    
    // Error estimate (4th - 5th order difference)
    last_error_ = 0.0;
    for (size_t i = 0; i < sz; ++i) {
        double e = dt * std::abs(k1[i] * (1.0/360.0 - 16.0/135.0) + k3[i] * (-128.0/4275.0 - 6656.0/12825.0)
                                 + k4[i] * (-2197.0/75240.0 - 28561.0/56430.0) + k5[i] * (1.0/50.0 + 9.0/50.0)
                                 + k6[i] * (2.0/55.0));
        last_error_ += e * e;
    }
    last_error_ = std::sqrt(last_error_ / sz);
    t += dt;
}

// ---- Ruth (3rd order symplectic) ----
void Solver::step_ruth(StateVector& s, DerivFunction f, double& t, double dt) {
    size_t n = s.size() / 2;
    const double c1 = 2.0 / 3.0;
    const double c2 = -2.0 / 3.0;
    const double c3 = 1.0;
    const double d1 = 7.0 / 24.0;
    const double d2 = 3.0 / 4.0;
    const double d3 = -1.0 / 24.0;
    
    StateVector tmp(s.size(), 0.0);
    StateVector acc(n, 0.0);
    
    // Step 1: v += c1 * a * dt
    f(s, tmp, t);
    for (size_t i = 0; i < n; ++i) acc[i] = tmp[n + i];
    for (size_t i = 0; i < n; ++i) s[n + i] += c1 * acc[i] * dt;
    
    // Step 2: x += d1 * v * dt
    for (size_t i = 0; i < n; ++i) s[i] += d1 * s[n + i] * dt;
    
    // Step 3: v += c2 * a * dt
    f(s, tmp, t + d1 * dt);
    for (size_t i = 0; i < n; ++i) acc[i] = tmp[n + i];
    for (size_t i = 0; i < n; ++i) s[n + i] += c2 * acc[i] * dt;
    
    // Step 4: x += d2 * v * dt
    for (size_t i = 0; i < n; ++i) s[i] += d2 * s[n + i] * dt;
    
    // Step 5: v += c3 * a * dt
    f(s, tmp, t + (d1 + d2) * dt);
    for (size_t i = 0; i < n; ++i) acc[i] = tmp[n + i];
    for (size_t i = 0; i < n; ++i) s[n + i] += c3 * acc[i] * dt;
    
    // Step 6: x += d3 * v * dt
    for (size_t i = 0; i < n; ++i) s[i] += d3 * s[n + i] * dt;
    
    t += dt;
}


// =================== PhysicsSystem ===================

StateVector PhysicsSystem::pack_state(const std::vector<Body>& bodies) {
    StateVector state(bodies.size() * 6, 0.0); // 3 pos + 3 vel per body
    for (size_t i = 0; i < bodies.size(); ++i) {
        size_t idx = i * 6;
        state[idx + 0] = bodies[i].state.position.x;
        state[idx + 1] = bodies[i].state.position.y;
        state[idx + 2] = bodies[i].state.position.z;
        state[idx + 3] = bodies[i].state.velocity.x;
        state[idx + 4] = bodies[i].state.velocity.y;
        state[idx + 5] = bodies[i].state.velocity.z;
    }
    return state;
}

void PhysicsSystem::unpack_state(const StateVector& state, std::vector<Body>& bodies) {
    for (size_t i = 0; i < bodies.size(); ++i) {
        size_t idx = i * 6;
        bodies[i].state.position.x = state[idx + 0];
        bodies[i].state.position.y = state[idx + 1];
        bodies[i].state.position.z = state[idx + 2];
        bodies[i].state.velocity.x = state[idx + 3];
        bodies[i].state.velocity.y = state[idx + 4];
        bodies[i].state.velocity.z = state[idx + 5];
    }
}

void PhysicsSystem::compute_derivatives(const StateVector& state, StateVector& deriv,
                                         double t, const SimulationParams& params,
                                         const std::vector<SpringConstraint>& springs,
                                         const std::vector<Body>& bodies_ref) {
    size_t n = bodies_ref.size();
    if (deriv.size() != state.size()) deriv.resize(state.size(), 0.0);
    
    // Unpack state into temporary bodies for force computation
    std::vector<Body> bodies = bodies_ref;
    
    // Update positions and velocities from state
    for (size_t i = 0; i < n; ++i) {
        size_t idx = i * 6;
        bodies[i].state.position.x = state[idx + 0];
        bodies[i].state.position.y = state[idx + 1];
        bodies[i].state.position.z = state[idx + 2];
        bodies[i].state.velocity.x = state[idx + 3];
        bodies[i].state.velocity.y = state[idx + 4];
        bodies[i].state.velocity.z = state[idx + 5];
    }
    
    // Clear forces
    for (auto& b : bodies) {
        b.force = dvec3(0.0);
    }
    
    // Apply all enabled forces
    if (std::abs(params.gravity) > 1e-12) {
        apply_gravity(bodies, params.gravity);
    }
    if (params.enable_wind && glm::length(params.wind) > 1e-12) {
        apply_wind(bodies, params.wind);
    }
    if (params.enable_drag && params.air_density > 1e-12) {
        apply_drag(bodies, params.air_density);
    }
    if (params.enable_buoyancy && params.buoyancy > 1e-12) {
        apply_buoyancy(bodies, params.buoyancy);
    }
    if (params.enable_magnet && std::abs(params.magnetic_field) > 1e-12) {
        apply_magnetic(bodies, params.magnetic_field, params.electric_field);
    }
    if (params.enable_central_g && params.central_mass > 1e-12) {
        apply_central_gravity(bodies, params.central_mass);
    }
    if (params.enable_coulomb && std::abs(params.coulomb_k) > 1e-12) {
        apply_coulomb(bodies, params.coulomb_k);
    }
    if (params.enable_vortex && std::abs(params.vortex_strength) > 1e-12) {
        apply_vortex(bodies, params.vortex_strength);
    }
    if (params.enable_coriolis && std::abs(params.coriolis) > 1e-12) {
        apply_coriolis(bodies, params.coriolis);
    }
    if (params.enable_turbulence && std::abs(params.turbulence) > 1e-12) {
        apply_turbulence(bodies, params.turbulence, t);
    }
    if (params.enable_cohesion && std::abs(params.cohesion) > 1e-12) {
        apply_cohesion(bodies, params.cohesion);
    }
    if (params.enable_springs && !springs.empty()) {
        apply_springs(bodies, springs);
    }
    
    // Pack derivatives: dx/dt = v, dv/dt = a = F/m
    for (size_t i = 0; i < n; ++i) {
        size_t idx = i * 6;
        // Position derivative = velocity
        deriv[idx + 0] = bodies[i].state.velocity.x;
        deriv[idx + 1] = bodies[i].state.velocity.y;
        deriv[idx + 2] = bodies[i].state.velocity.z;
        
        // Velocity derivative = acceleration
        double inv_mass = bodies[i].is_static() ? 0.0 : (1.0 / bodies[i].props.mass);
        deriv[idx + 3] = bodies[i].force.x * inv_mass;
        deriv[idx + 4] = bodies[i].force.y * inv_mass;
        deriv[idx + 5] = bodies[i].force.z * inv_mass;
    }
}

void PhysicsSystem::detect_and_resolve_collisions(std::vector<Body>& bodies,
                                                   const SimulationParams& params,
                                                   double dt,
                                                   std::vector<CollisionEvent>& events) {
    for (size_t i = 0; i < bodies.size(); ++i) {
        for (size_t j = i + 1; j < bodies.size(); ++j) {
            auto& a = bodies[i];
            auto& b = bodies[j];
            if (a.is_static() && b.is_static()) continue;
            
            dvec3 delta = b.state.position - a.state.position;
            double dist = glm::length(delta);
            double min_dist = a.props.radius + b.props.radius;
            
            if (dist < min_dist && dist > 1e-10) {
                dvec3 n = delta / dist;
                double overlap = min_dist - dist;
                double total_mass = a.props.mass + b.props.mass;
                
                // Position correction
                if (!a.is_static())
                    a.state.position -= n * overlap * (b.props.mass / total_mass);
                if (!b.is_static())
                    b.state.position += n * overlap * (a.props.mass / total_mass);
                
                // Relative velocity
                dvec3 rel_v = a.state.velocity - b.state.velocity;
                double rel_vn = glm::dot(rel_v, n);
                
                if (rel_vn > 0) {
                    double e = (a.props.restitution + b.props.restitution) * 0.5;
                    double impulse = (1.0 + e) * rel_vn / total_mass;
                    
                    if (!a.is_static())
                        a.state.velocity -= impulse * b.props.mass * n;
                    if (!b.is_static())
                        b.state.velocity += impulse * a.props.mass * n;
                    
                    // Friction
                    dvec3 tangent = rel_v - rel_vn * n;
                    double tlen = glm::length(tangent);
                    if (tlen > 1e-10 && params.friction > 0) {
                        double f_impulse = std::min(params.friction * impulse, tlen / total_mass);
                        dvec3 t_dir = tangent / tlen;
                        if (!a.is_static())
                            a.state.velocity -= f_impulse * b.props.mass * t_dir;
                        if (!b.is_static())
                            b.state.velocity += f_impulse * a.props.mass * t_dir;
                    }
                    
                    events.push_back({
                        a.id, b.id,
                        (a.state.position + b.state.position) * 0.5,
                        n, impulse, 0.0
                    });
                }
            }
        }
    }
}

// ============= Force implementations =============

void PhysicsSystem::apply_gravity(std::vector<Body>& bodies, double g) {
    for (auto& b : bodies) {
        if (!b.is_static()) {
            b.force.y -= b.props.mass * g;
        }
    }
}

void PhysicsSystem::apply_wind(std::vector<Body>& bodies, const dvec3& wind) {
    for (auto& b : bodies) {
        if (!b.is_static()) {
            b.force += wind * b.props.mass;
        }
    }
}

void PhysicsSystem::apply_drag(std::vector<Body>& bodies, double rho) {
    for (auto& b : bodies) {
        if (b.is_static()) continue;
        double v2 = glm::dot(b.state.velocity, b.state.velocity);
        if (v2 > 1e-12) {
            double v = std::sqrt(v2);
            double area = M_PI * b.props.radius * b.props.radius;
            double drag = 0.5 * rho * v2 * area * 0.47;
            b.force -= (b.state.velocity / v) * drag;
        }
    }
}

void PhysicsSystem::apply_buoyancy(std::vector<Body>& bodies, double buoyancy) {
    for (auto& b : bodies) {
        if (!b.is_static()) {
            double vol = (4.0 / 3.0) * M_PI * std::pow(b.props.radius, 3);
            b.force.y += buoyancy * vol;
        }
    }
}

void PhysicsSystem::apply_magnetic(std::vector<Body>& bodies, double B, double E) {
    for (auto& b : bodies) {
        if (b.is_static() || std::abs(b.props.charge) < 1e-12) continue;
        // Lorentz force: F = q * (E + v × B)
        dvec3 B_dir(0.0, B, 0.0);
        dvec3 v_cross_B = glm::cross(b.state.velocity, B_dir);
        b.force += b.props.charge * (dvec3(E, 0.0, 0.0) + v_cross_B);
    }
}

void PhysicsSystem::apply_central_gravity(std::vector<Body>& bodies, double M) {
    const double G = 5.0;
    for (auto& b : bodies) {
        if (b.is_static()) continue;
        double dist2 = glm::dot(b.state.position, b.state.position);
        if (dist2 < 0.01) continue;
        double dist = std::sqrt(dist2);
        double F = G * M * b.props.mass / dist2;
        b.force -= (b.state.position / dist) * F;
    }
}

void PhysicsSystem::apply_coulomb(std::vector<Body>& bodies, double k) {
    for (size_t i = 0; i < bodies.size(); ++i) {
        for (size_t j = i + 1; j < bodies.size(); ++j) {
            auto& a = bodies[i], & b = bodies[j];
            if (std::abs(a.props.charge) < 1e-12 && std::abs(b.props.charge) < 1e-12) continue;
            dvec3 delta = b.state.position - a.state.position;
            double dist2 = glm::dot(delta, delta);
            if (dist2 < 0.01) continue;
            double dist = std::sqrt(dist2);
            double F = k * a.props.charge * b.props.charge / dist2;
            dvec3 f = (delta / dist) * F;
            if (!a.is_static()) a.force += f;
            if (!b.is_static()) b.force -= f;
        }
    }
}

void PhysicsSystem::apply_vortex(std::vector<Body>& bodies, double strength) {
    for (auto& b : bodies) {
        if (b.is_static()) continue;
        double r = std::sqrt(b.state.position.x * b.state.position.x + 
                             b.state.position.z * b.state.position.z);
        if (r > 0.1) {
            double vf = strength / (r + 0.5);
            b.force.x += (-b.state.position.z / r) * vf * b.props.mass;
            b.force.z += (b.state.position.x / r) * vf * b.props.mass;
            // Inward spiral
            b.force.x -= (b.state.position.x / r) * vf * 0.1 * b.props.mass;
            b.force.z -= (b.state.position.z / r) * vf * 0.1 * b.props.mass;
        }
    }
}

void PhysicsSystem::apply_coriolis(std::vector<Body>& bodies, double omega) {
    for (auto& b : bodies) {
        if (b.is_static()) continue;
        b.force.x += 2.0 * b.props.mass * omega * b.state.velocity.z;
        b.force.z -= 2.0 * b.props.mass * omega * b.state.velocity.x;
    }
}

void PhysicsSystem::apply_turbulence(std::vector<Body>& bodies, double amp, double t) {
    for (auto& b : bodies) {
        if (b.is_static()) continue;
        // Deterministic pseudo-random using body id and time
        double seed = (double)(b.id * 7919) + t * 137.0;
        b.force.x += std::sin(seed * 13.37) * amp * b.props.mass;
        b.force.y += std::cos(seed * 42.0) * amp * b.props.mass;
        b.force.z += std::sin(seed * 73.1 + 1.0) * amp * b.props.mass;
    }
}

void PhysicsSystem::apply_cohesion(std::vector<Body>& bodies, double strength) {
    for (size_t i = 0; i < bodies.size(); ++i) {
        for (size_t j = i + 1; j < bodies.size(); ++j) {
            auto& a = bodies[i], & b = bodies[j];
            if (a.is_static() && b.is_static()) continue;
            dvec3 delta = b.state.position - a.state.position;
            double dist = glm::length(delta);
            double range = a.props.radius + b.props.radius + 2.0;
            if (dist < range && dist > 0.01) {
                double s = strength * (1.0 - dist / range);
                dvec3 f = (delta / dist) * s;
                if (!a.is_static()) a.force += f;
                if (!b.is_static()) b.force -= f;
            }
        }
    }
}

void PhysicsSystem::apply_springs(std::vector<Body>& bodies,
                                   const std::vector<SpringConstraint>& springs) {
    for (const auto& sp : springs) {
        auto a = std::find_if(bodies.begin(), bodies.end(), 
                               [&](const Body& b) { return b.id == sp.body_a; });
        auto b = std::find_if(bodies.begin(), bodies.end(),
                               [&](const Body& b) { return b.id == sp.body_b; });
        if (a == bodies.end() || b == bodies.end()) continue;
        
        dvec3 delta = b->state.position - a->state.position;
        double dist = glm::length(delta);
        if (dist < 1e-10) continue;
        
        double stretch = dist - sp.rest_length;
        double force_mag = sp.stiffness * stretch;
        
        // Damping
        dvec3 rv = b->state.velocity - a->state.velocity;
        double damp = sp.damping * glm::dot(rv, delta) / dist;
        
        dvec3 f = (delta / dist) * (force_mag + damp);
        if (!a->is_static()) a->force += f;
        if (!b->is_static()) b->force -= f;
    }
}

} // namespace hlp
