#pragma once

// ============================================================
// HoloPhysics Lab 2 — Scene Manager
// Сохранение, загрузка, управление экспериментами
// ============================================================

#include "Types.h"
#include "Solver.h"
#include <string>
#include <memory>
#include <functional>

namespace hlp {

// == Experiment preset definition ==
struct PresetDefinition {
    std::string id;
    std::string name;
    std::string description;
    std::function<void(std::vector<Body>&, SimulationParams&, 
                       std::vector<SpringConstraint>&)> setup;
};

class Scene {
public:
    Scene();
    ~Scene();

    // Body management
    Body& add_body(const Body& body);
    Body& add_body(const BodyState& state, const BodyProperties& props,
                   const VisualProperties& visual, uint32_t flags = BODY_VISIBLE | BODY_GRAVITY);
    void remove_body(uint64_t id);
    void clear_bodies();
    Body* get_body(uint64_t id);
    const std::vector<Body>& bodies() const { return bodies_; }
    std::vector<Body>& bodies() { return bodies_; }
    size_t body_count() const { return bodies_.size(); }

    // Physics parameters
    SimulationParams& params() { return params_; }
    const SimulationParams& params() const { return params_; }

    // Springs
    void add_spring(uint64_t a, uint64_t b, double k = 10.0, 
                    double damping = 0.1, double rest_len = 0.0);
    void remove_spring(uint64_t a, uint64_t b);
    std::vector<SpringConstraint>& springs() { return springs_; }

    // Simulation time
    double time() const { return sim_time_; }
    void set_time(double t) { sim_time_ = t; }

    // Integrator
    void set_integrator(IntegratorType type) { integrator_ = type; }
    IntegratorType integrator() const { return integrator_; }

    // Energy diagnostics
    double total_energy() const;
    double kinetic_energy() const;
    double potential_energy() const;
    double linear_momentum() const;
    double energy_drift() const;
    void reset_energy_tracking();

    // Trails
    void update_trails(double dt);
    void clear_trails();

    // Experiment presets
    void load_preset(const std::string& id);
    static std::vector<PresetDefinition> available_presets();

    // Serialization
    std::string serialize_to_json() const;
    bool deserialize_from_json(const std::string& json);
    bool save_to_file(const std::string& path) const;
    bool load_from_file(const std::string& path);

    // Experiment ID
    std::string experiment_id() const { return experiment_id_; }
    void set_experiment_id(const std::string& id) { experiment_id_ = id; }

private:
    std::vector<Body> bodies_;
    std::vector<SpringConstraint> springs_;
    SimulationParams params_;
    double sim_time_ = 0.0;
    IntegratorType integrator_ = IntegratorType::SymplecticEuler;
    std::string experiment_id_ = "sandbox";
    
    // Energy tracking
    double initial_energy_ = 0.0;
    bool energy_tracked_ = false;
    
    uint64_t next_id_ = 0;
};

} // namespace hlp
