#pragma once

// ============================================================
// HoloPhysics Lab 2 — Adaptive Timestep Controller
// Error estimation and automatic dt adjustment
// ============================================================

#include "../core/Types.h"
#include "../core/Solver.h"

namespace hlp {

struct AdaptiveTimestepConfig {
    double min_dt = 1e-6;        // Minimum timestep
    double max_dt = 1.0 / 30.0;  // Maximum timestep
    double target_dt = 1.0 / 60.0; // Target timestep
    double safety_factor = 0.9;  // Safety factor for dt adjustment
    double error_tolerance = 1e-4; // Relative error tolerance
    
    // Growth/shrink limits per step
    double max_growth = 2.0;
    double max_shrink = 0.2;
    
    // CFL condition (for fluid simulations)
    bool use_cfl = false;
    double cfl_number = 0.5;
    
    // Energy conservation check
    bool check_energy = true;
    double max_energy_drift = 1e-3; // Per step
    
    // Convergence check
    int max_convergence_iterations = 5;
    double convergence_tol = 1e-6;
};

class AdaptiveTimestep {
public:
    AdaptiveTimestep();
    ~AdaptiveTimestep();

    // Set configuration
    void set_config(const AdaptiveTimestepConfig& config) { config_ = config; }
    const AdaptiveTimestepConfig& config() const { return config_; }
    AdaptiveTimestepConfig& config() { return config_; }

    // Compute next timestep based on error estimate
    double compute_dt(const PhysicsSystem& physics,
                      const std::vector<Body>& bodies,
                      const SimulationParams& params,
                      const Solver& solver,
                      double current_dt,
                      double sim_time);
    
    // CFL-based timestep
    double compute_cfl_dt(const std::vector<Body>& bodies) const;
    
    // Energy-based timestep
    double compute_energy_dt(const std::vector<Body>& bodies,
                             double current_dt,
                             double initial_energy) const;
    
    // Get last error estimate
    double last_error() const { return last_error_; }
    double last_energy_drift() const { return last_energy_drift_; }
    int rejected_steps() const { return rejected_steps_; }
    int accepted_steps() const { return accepted_steps_; }
    
    // Reset statistics
    void reset();
    
    // Get recommended substeps for fixed-step integrators
    int recommended_substeps(double target_dt) const;

private:
    AdaptiveTimestepConfig config_;
    double last_error_ = 0.0;
    double last_energy_drift_ = 0.0;
    int rejected_steps_ = 0;
    int accepted_steps_ = 0;
    
    // Previous state for error estimation
    StateVector prev_state_;
    bool have_prev_state_ = false;
    
    // Step doubling error estimation
    double estimate_error_step_doubling(const PhysicsSystem& physics,
                                        const std::vector<Body>& bodies,
                                        const StateVector& state,
                                        double dt, double sim_time);
};

} // namespace hlp
