#include "AdaptiveTimestep.h"
#include <algorithm>
#include <cmath>

namespace hlp {

AdaptiveTimestep::AdaptiveTimestep() = default;
AdaptiveTimestep::~AdaptiveTimestep() = default;

void AdaptiveTimestep::reset() {
    rejected_steps_ = 0;
    accepted_steps_ = 0;
    have_prev_state_ = false;
    last_error_ = 0.0;
}

double AdaptiveTimestep::compute_dt(const PhysicsSystem& physics,
                                     const std::vector<Body>& bodies,
                                     const SimulationParams& params,
                                     const Solver& solver,
                                     double current_dt,
                                     double sim_time) {
    double new_dt = current_dt;
    
    // 1. Error-based estimation (step doubling)
    if (solver.current_integrator() >= IntegratorType::RK4) {
        StateVector state = PhysicsSystem::pack_state(bodies);
        double error = 0;
        
        // Compute with dt/2 twice
        StateVector state_half = state;
        // (simplified - full implementation would actually step twice)
        
        new_dt = current_dt * std::pow(config_.error_tolerance / std::max(error, 1e-10), 
                                        0.25);
        last_error_ = error;
    }
    
    // 2. CFL condition
    if (config_.use_cfl) {
        double cfl_dt = compute_cfl_dt(bodies);
        new_dt = std::min(new_dt, cfl_dt);
    }
    
    // 3. Energy conservation
    if (config_.check_energy) {
        double energy_dt = compute_energy_dt(bodies, new_dt, 0);
        new_dt = std::min(new_dt, energy_dt);
    }
    
    // Clamp
    new_dt = std::clamp(new_dt, config_.min_dt, config_.max_dt);
    
    // Limit growth/shrink
    double max_allowed = current_dt * config_.max_growth;
    double min_allowed = current_dt * config_.max_shrink;
    new_dt = std::clamp(new_dt, min_allowed, max_allowed);
    
    // Rejection if too large change
    if (new_dt < current_dt * 0.5 && last_error_ > config_.error_tolerance) {
        rejected_steps_++;
    } else {
        accepted_steps_++;
    }
    
    return new_dt;
}

double AdaptiveTimestep::compute_cfl_dt(const std::vector<Body>& bodies) const {
    double max_speed = 0.0;
    double min_spacing = 1e10;
    
    for (const auto& b : bodies) {
        if (b.is_static()) continue;
        max_speed = std::max(max_speed, glm::length(b.state.velocity));
        
        // Find minimum distance to nearest neighbor
        for (const auto& other : bodies) {
            if (&b == &other) continue;
            double dist = glm::length(other.state.position - b.state.position);
            if (dist > 0) min_spacing = std::min(min_spacing, dist);
        }
    }
    
    if (max_speed < 1e-10 || min_spacing > 1e9) return config_.max_dt;
    return config_.cfl_number * min_spacing / max_speed;
}

double AdaptiveTimestep::compute_energy_dt(const std::vector<Body>& bodies,
                                            double current_dt,
                                            double initial_energy) const {
    if (!config_.check_energy) return current_dt;
    
    // Compute total energy variation rate
    double ke = 0, pe = 0;
    for (const auto& b : bodies) {
        if (b.is_static()) continue;
        ke += 0.5 * b.props.mass * glm::dot(b.state.velocity, b.state.velocity);
        pe += b.props.mass * 9.81 * std::abs(b.state.position.y);
    }
    
    double total = ke + pe;
    if (total < 1e-10) return current_dt;
    
    // Estimate energy drift per unit time
    double estimated_drift = 0.01 * total; // 1% of total energy
    if (estimated_drift < 1e-10) return current_dt;
    
    // Drift-based: limit dt so drift doesn't exceed tolerance
    double max_dt_by_energy = config_.max_energy_drift * total / estimated_drift;
    return std::min(current_dt, max_dt_by_energy);
}

int AdaptiveTimestep::recommended_substeps(double target_dt) const {
    // For fixed-step integrators, recommend substeps
    if (target_dt < 1e-10) return 1;
    return std::max(1, (int)std::ceil(target_dt / config_.target_dt));
}

} // namespace hlp
