#include "Engine.h"
#include <random>
#include <cmath>

namespace hlp {

Engine::Engine() 
    : scene_(std::make_unique<Scene>()) {
    scene_->load_preset("sandbox_demo");
}

Engine::~Engine() = default;

void Engine::step_physics() {
    if (paused_) return;
    
    const double dt = (1.0 / 60.0) / substeps_;
    auto& params = scene_->params();
    auto& bodies = scene_->bodies();
    auto& springs = scene_->springs();
    
    // Gravity well (applied every substep)
    if (gravity_well_active_) {
        for (auto& b : bodies) {
            if (b.is_static()) continue;
            dvec3 delta = gravity_well_point_ - b.state.position;
            double dist = glm::length(delta);
            if (dist > 0.1) {
                double f = gravity_well_strength_ / (dist * dist + 0.1);
                b.state.velocity += (delta / dist) * f * dt;
            }
        }
    }
    
    collision_events_.clear();
    double t = scene_->time();
    
    for (int substep = 0; substep < substeps_; ++substep) {
        // Pack state
        StateVector state = PhysicsSystem::pack_state(bodies);
        size_t n_bodies = bodies.size();
        
        // Save reference bodies for derivative computation
        auto bodies_ref = bodies;
        
        // Create derivative function
        auto derivs = [&](const StateVector& s, StateVector& d, double time) {
            physics_.compute_derivatives(s, d, time, params, springs, bodies_ref);
        };
        
        // Step the solver
        solver_.step(state, derivs, t, dt);
        PhysicsSystem::unpack_state(state, bodies);
        
        // Collisions
        if (n_bodies > 1) {
            physics_.detect_and_resolve_collisions(bodies, params, dt, collision_events_);
        }
        
        // Trails
        if (params.show_trails) {
            scene_->update_trails(dt);
        }
    }
    
    scene_->set_time(t);
    stats_.step_count++;
    stats_.sim_time = t;
    stats_.body_count = bodies.size();
    stats_.spring_count = springs.size();
    stats_.total_energy = scene_->total_energy();
    stats_.momentum = scene_->linear_momentum();
    stats_.energy_drift = scene_->energy_drift();
    stats_.integrator = solver_.current_integrator();
}

void Engine::apply_force(uint64_t body_id, const dvec3& force) {
    auto* body = scene_->get_body(body_id);
    if (body) body->force += force;
}

void Engine::apply_impulse(uint64_t body_id, const dvec3& impulse) {
    auto* body = scene_->get_body(body_id);
    if (body && !body->is_static()) {
        body->state.velocity += impulse / body->props.mass;
    }
}

void Engine::explode(const dvec3& center, double radius, double force) {
    for (auto& b : scene_->bodies()) {
        if (b.is_static()) continue;
        dvec3 delta = b.state.position - center;
        double dist = glm::length(delta);
        if (dist < radius && dist > 0.01) {
            double mag = force * (1.0 - dist / radius) / b.props.mass;
            b.state.velocity += (delta / dist) * mag;
        }
    }
}

void Engine::freeze_all() {
    for (auto& b : scene_->bodies()) {
        if (!b.is_static()) {
            b.state.velocity = dvec3(0.0);
            b.state.angular_velocity = dvec3(0.0);
        }
    }
}

void Engine::randomize_velocities(double magnitude) {
    std::mt19937 rng(std::random_device{}());
    std::uniform_real_distribution<double> dist(-magnitude, magnitude);
    for (auto& b : scene_->bodies()) {
        if (!b.is_static()) {
            b.state.velocity = dvec3(dist(rng), dist(rng), dist(rng));
        }
    }
}

void Engine::spawn_random_bodies(int count) {
    std::mt19937 rng(std::random_device{}());
    std::uniform_real_distribution<double> pos(-5.0, 5.0);
    std::uniform_real_distribution<double> height(2.0, 8.0);
    std::uniform_real_distribution<double> size(0.2, 0.6);
    std::uniform_real_distribution<double> mass(0.3, 2.0);
    
    glm::vec3 colors[] = {
        {0.0f, 1.0f, 0.8f}, {1.0f, 0.0f, 0.67f}, {0.27f, 0.53f, 1.0f},
        {1.0f, 0.8f, 0.0f}, {0.0f, 1.0f, 0.53f}, {1.0f, 0.4f, 0.0f},
        {0.67f, 0.27f, 1.0f}, {1.0f, 0.2f, 0.33f}
    };
    
    for (int i = 0; i < count; ++i) {
        BodyState state;
        state.position = dvec3(pos(rng), height(rng), pos(rng));
        state.velocity = dvec3((rng() % 200 - 100) * 0.01, (rng() % 200 - 100) * 0.01,
                               (rng() % 200 - 100) * 0.01);
        
        BodyProperties props;
        props.mass = mass(rng);
        props.radius = size(rng);
        props.restitution = 0.3 + (rng() % 50) * 0.01;
        
        VisualProperties visual;
        visual.color = colors[rng() % 8];
        visual.shape = static_cast<ShapeType>(rng() % 4);
        
        scene_->add_body(state, props, visual);
    }
}

void Engine::reset_stats() {
    stats_ = EngineStats();
    scene_->reset_energy_tracking();
}

} // namespace hlp
