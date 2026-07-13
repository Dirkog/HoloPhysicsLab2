#pragma once

// ============================================================
// HoloPhysics Lab 2 — Numerical Integrators
// Высокоточные методы интегрирования для научных симуляций
// ============================================================

#include "Types.h"
#include <vector>
#include <functional>

namespace hlp {

// System state: flat array [pos0_x, pos0_y, pos0_z, vel0_x, vel0_y, vel0_z, ...]
using StateVector = std::vector<double>;
using DerivFunction = std::function<void(const StateVector&, StateVector&, double)>;

class Solver {
public:
    Solver() = default;

    // Set the integration method
    void set_integrator(IntegratorType type);
    IntegratorType current_integrator() const { return integrator_; }

    // Names for UI
    static const char* integrator_name(IntegratorType t);
    static const char** integrator_names();
    static int integrator_count();

    // Step the physics forward
    void step(StateVector& state, DerivFunction derivs, 
              double& t, double dt);

    // Get last step error estimate (for adaptive methods)
    double last_error() const { return last_error_; }

    // Energy conservation diagnostics
    struct EnergyDiag {
        double e_initial;
        double e_current;
        double drift;       // |E - E0| / |E0|
        double drift_rate;  // drift per second
    };

private:
    IntegratorType integrator_ = IntegratorType::SymplecticEuler;
    double last_error_ = 0.0;

    // Integrator implementations
    void step_euler(StateVector&, DerivFunction, double&, double);
    void step_symplectic(StateVector&, DerivFunction, double&, double);
    void step_verlet(StateVector&, DerivFunction, double&, double);
    void step_rk4(StateVector&, DerivFunction, double&, double);
    void step_rk5(StateVector&, DerivFunction, double&, double);
    void step_ruth(StateVector&, DerivFunction, double&, double);
};

// ============================================================
// PhysicsSystem — applies all forces to bodies
// ============================================================

class PhysicsSystem {
public:
    PhysicsSystem() = default;

    // Convert bodies to state vector
    static StateVector pack_state(const std::vector<Body>& bodies);
    
    // Convert state vector back to bodies
    static void unpack_state(const StateVector& state, std::vector<Body>& bodies);

    // Compute derivatives for the entire system
    void compute_derivatives(const StateVector& state, StateVector& deriv,
                             double t, const SimulationParams& params,
                             const std::vector<SpringConstraint>& springs,
                             const std::vector<Body>& bodies_ref);

    // Apply collision response
    void detect_and_resolve_collisions(std::vector<Body>& bodies,
                                       const SimulationParams& params,
                                       double dt,
                                       std::vector<CollisionEvent>& events);

    // Single force components
    static void apply_gravity(std::vector<Body>& bodies, double g);
    static void apply_wind(std::vector<Body>& bodies, const dvec3& wind);
    static void apply_drag(std::vector<Body>& bodies, double air_density);
    static void apply_buoyancy(std::vector<Body>& bodies, double buoyancy);
    static void apply_magnetic(std::vector<Body>& bodies, double B, double E);
    static void apply_central_gravity(std::vector<Body>& bodies, double M);
    static void apply_coulomb(std::vector<Body>& bodies, double k);
    static void apply_vortex(std::vector<Body>& bodies, double strength);
    static void apply_coriolis(std::vector<Body>& bodies, double omega);
    static void apply_turbulence(std::vector<Body>& bodies, double amp, double t);
    static void apply_cohesion(std::vector<Body>& bodies, double strength);
    static void apply_springs(std::vector<Body>& bodies, 
                              const std::vector<SpringConstraint>& springs);

private:
    // Temporary storage for derivative computation
    StateVector temp_deriv_;
};

} // namespace hlp
