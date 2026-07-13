#include "Scene.h"
#include <nlohmann/json.hpp>
#include <fstream>
#include <iostream>
#include <random>

namespace hlp {

Scene::Scene() {}
Scene::~Scene() {}

Body& Scene::add_body(const Body& body) {
    bodies_.push_back(body);
    bodies_.back().id = next_id_++;
    return bodies_.back();
}

Body& Scene::add_body(const BodyState& state, const BodyProperties& props,
                       const VisualProperties& visual, uint32_t flags) {
    Body body(next_id_++);
    body.state = state;
    body.props = props;
    body.visual = visual;
    body.flags = flags;
    bodies_.push_back(body);
    return bodies_.back();
}

void Scene::remove_body(uint64_t id) {
    bodies_.erase(std::remove_if(bodies_.begin(), bodies_.end(),
        [id](const Body& b) { return b.id == id; }), bodies_.end());
    springs_.erase(std::remove_if(springs_.begin(), springs_.end(),
        [id](const SpringConstraint& s) { return s.body_a == id || s.body_b == id; }),
        springs_.end());
}

void Scene::clear_bodies() {
    bodies_.clear();
    springs_.clear();
    sim_time_ = 0.0;
}

Body* Scene::get_body(uint64_t id) {
    for (auto& b : bodies_) {
        if (b.id == id) return &b;
    }
    return nullptr;
}

void Scene::add_spring(uint64_t a, uint64_t b, double k, double damping, double rest_len) {
    if (rest_len <= 0) {
        auto* ba = get_body(a);
        auto* bb = get_body(b);
        if (ba && bb) {
            rest_len = glm::length(bb->state.position - ba->state.position);
        }
    }
    springs_.push_back({a, b, rest_len, k, damping});
}

void Scene::remove_spring(uint64_t a, uint64_t b) {
    springs_.erase(std::remove_if(springs_.begin(), springs_.end(),
        [a, b](const SpringConstraint& s) { 
            return (s.body_a == a && s.body_b == b) || (s.body_a == b && s.body_b == a); 
        }), springs_.end());
}

double Scene::total_energy() const {
    double e = 0;
    for (const auto& b : bodies_) {
        if (!b.is_static()) e += b.kinetic_energy();
        // Potential from gravity and central field
        if (!b.is_static()) {
            e += b.props.mass * std::abs(params_.gravity) * std::abs(b.state.position.y);
        }
    }
    // Spring potential
    for (const auto& s : springs_) {
        auto* a = get_body(s.body_a);
        auto* b = get_body(s.body_b);
        if (a && b) {
            double dist = glm::length(b->state.position - a->state.position);
            e += 0.5 * s.stiffness * (dist - s.rest_length) * (dist - s.rest_length);
        }
    }
    return e;
}

double Scene::kinetic_energy() const {
    double e = 0;
    for (const auto& b : bodies_) {
        if (!b.is_static()) e += b.kinetic_energy();
    }
    return e;
}

double Scene::potential_energy() const {
    double e = 0;
    for (const auto& b : bodies_) {
        if (!b.is_static()) {
            e += b.props.mass * std::abs(params_.gravity) * std::abs(b.state.position.y);
        }
    }
    return total_energy() - kinetic_energy();
}

double Scene::linear_momentum() const {
    dvec3 p(0.0);
    for (const auto& b : bodies_) {
        if (!b.is_static()) p += b.props.mass * b.state.velocity;
    }
    return glm::length(p);
}

double Scene::energy_drift() const {
    if (!energy_tracked_ || initial_energy_ < 1e-30) return 0.0;
    return std::abs(total_energy() - initial_energy_) / std::abs(initial_energy_);
}

void Scene::reset_energy_tracking() {
    initial_energy_ = total_energy();
    energy_tracked_ = true;
}

void Scene::update_trails(double dt) {
    if (!params_.show_trails) return;
    for (auto& b : bodies_) {
        if (b.is_static()) continue;
        b.trail.push_back(b.state.position);
        if (b.trail.size() > b.max_trail) {
            b.trail.erase(b.trail.begin());
        }
    }
}

void Scene::clear_trails() {
    for (auto& b : bodies_) b.trail.clear();
}

// ==================== PRESETS ====================
std::vector<PresetDefinition> Scene::available_presets() {
    std::vector<PresetDefinition> presets;
    
    presets.push_back({"sandbox_demo", "Песочница (демо)", "3 тела для начала",
        [](std::vector<Body>& bodies, SimulationParams& params, 
           std::vector<SpringConstraint>& springs) {
            bodies.clear(); springs.clear();
            params = SimulationParams();
            params.gravity = 9.81;
            std::mt19937 rng(42);
            std::uniform_real_distribution<double> dist(-3.0, 3.0);
            for (int i = 0; i < 3; ++i) {
                Body b(i);
                b.state.position = dvec3(dist(rng), 3.0 + i * 2.0, dist(rng));
                b.state.velocity = dvec3(dist(rng) * 0.3, 0, dist(rng) * 0.3);
                b.props.mass = 0.5 + i * 0.5;
                b.props.radius = 0.3 + i * 0.15;
                b.visual.color = glm::vec3(0.0f, 0.8f + i * 0.1f, 0.8f - i * 0.2f);
                bodies.push_back(b);
            }
        }
    });
    
    presets.push_back({"solar_system", "Солнечная система", 
        "Центральная звезда + 6 планет на орбитах",
        [](std::vector<Body>& bodies, SimulationParams& params,
           std::vector<SpringConstraint>& springs) {
            bodies.clear(); springs.clear();
            params = SimulationParams();
            params.gravity = 0;  // No global gravity
            params.enable_central_g = true;
            params.central_mass = 200;
            
            // Star
            Body star(0);
            star.props.mass = 200;
            star.props.radius = 1.8;
            star.visual.color = glm::vec3(1.0f, 0.8f, 0.0f);
            star.visual.glow_color = glm::vec3(1.0f, 0.5f, 0.0f);
            star.flags = BODY_STATIC | BODY_CENTRAL | BODY_VISIBLE;
            bodies.push_back(star);
            
            struct Planet { double dist, radius, mass; glm::vec3 color; double speed; };
            std::vector<Planet> planets = {
                {3, 0.3, 0.5, glm::vec3(0.27f, 0.53f, 1.0f), 1.3},
                {5, 0.4, 0.8, glm::vec3(0.0f, 1.0f, 0.53f), 1.0},
                {7, 0.5, 1.2, glm::vec3(1.0f, 0.0f, 0.67f), 0.8},
                {9, 0.6, 2.0, glm::vec3(1.0f, 0.4f, 0.0f), 0.65},
                {11, 0.35, 0.4, glm::vec3(0.67f, 0.27f, 1.0f), 0.55},
                {13, 0.45, 0.7, glm::vec3(0.0f, 1.0f, 0.8f), 0.45},
            };
            
            for (size_t i = 0; i < planets.size(); ++i) {
                Body p(i + 1);
                double angle = (double)i / planets.size() * 2.0 * M_PI + 0.3;
                double os = planets[i].speed * 2.0;
                p.state.position = dvec3(std::cos(angle) * planets[i].dist, 0, std::sin(angle) * planets[i].dist);
                p.state.velocity = dvec3(-std::sin(angle) * os, 0.1, std::cos(angle) * os);
                p.props.mass = planets[i].mass;
                p.props.radius = planets[i].radius;
                p.visual.color = planets[i].color;
                bodies.push_back(p);
            }
        }
    });
    
    presets.push_back({"bouncing_balls", "Прыгающие шары", "30 шаров разного размера",
        [](std::vector<Body>& bodies, SimulationParams& params,
           std::vector<SpringConstraint>& springs) {
            bodies.clear(); springs.clear();
            params = SimulationParams();
            params.gravity = 15.0;
            params.restitution = 0.6;
            std::mt19937 rng(123);
            std::uniform_real_distribution<double> pos(-8.0, 8.0);
            std::uniform_real_distribution<double> height(1.0, 12.0);
            std::uniform_real_distribution<double> size(0.15, 0.6);
            std::uniform_real_distribution<double> mass(0.2, 2.0);
            
            glm::vec3 colors[] = {
                {0.0f, 1.0f, 0.8f}, {1.0f, 0.0f, 0.67f}, 
                {0.27f, 0.53f, 1.0f}, {1.0f, 0.8f, 0.0f}, {0.0f, 1.0f, 0.53f}
            };
            
            for (int i = 0; i < 30; ++i) {
                Body b(i);
                b.state.position = dvec3(pos(rng), height(rng), pos(rng));
                b.state.velocity = dvec3((rng() % 200 - 100) * 0.01, 0, (rng() % 200 - 100) * 0.01);
                b.props.mass = mass(rng);
                b.props.radius = size(rng);
                b.props.restitution = 0.3 + (rng() % 50) * 0.01;
                b.visual.color = colors[i % 5];
                bodies.push_back(b);
            }
        }
    });
    
    presets.push_back({"domino", "Домино", "15 падающих блоков",
        [](std::vector<Body>& bodies, SimulationParams& params,
           std::vector<SpringConstraint>& springs) {
            bodies.clear(); springs.clear();
            params = SimulationParams();
            params.gravity = 9.81;
            glm::vec3 colors[] = {
                {0.0f, 1.0f, 0.8f}, {0.27f, 0.53f, 1.0f}, {1.0f, 0.0f, 0.67f}, {1.0f, 0.8f, 0.0f}
            };
            for (int i = 0; i < 15; ++i) {
                Body b(i);
                b.state.position = dvec3(i * 1.2 - 7.5, 0.5, 0);
                b.props.mass = 1.0 + i * 0.1;
                b.props.radius = 0.3;
                b.props.restitution = 0.05;
                b.visual.color = colors[i % 4];
                b.visual.shape = ShapeType::Box;
                bodies.push_back(b);
            }
            // Push first domino
            if (!bodies.empty()) bodies[0].state.velocity.x = 0.5;
        }
    });
    
    presets.push_back({"fountain", "Фонтан", "40 частиц, вылетающих вверх",
        [](std::vector<Body>& bodies, SimulationParams& params,
           std::vector<SpringConstraint>& springs) {
            bodies.clear(); springs.clear();
            params = SimulationParams();
            params.gravity = 9.81;
            params.damping = 0.005;
            glm::vec3 colors[] = {
                {0.0f, 1.0f, 0.8f}, {0.27f, 0.53f, 1.0f}, {0.0f, 1.0f, 0.53f}, {1.0f, 0.8f, 0.0f}
            };
            std::mt19937 rng(42);
            for (int i = 0; i < 40; ++i) {
                Body b(i);
                double angle = (double)i / 40.0 * 2.0 * M_PI;
                double spd = 3.0 + (rng() % 300) * 0.01;
                b.state.position = dvec3(0, 0.5, 0);
                b.state.velocity = dvec3(std::cos(angle) * spd * 0.3, 
                                         spd * (0.6 + (rng() % 40) * 0.01),
                                         std::sin(angle) * spd * 0.3);
                b.props.mass = 0.2 + (rng() % 30) * 0.01;
                b.props.radius = 0.15 + (rng() % 10) * 0.01;
                b.props.restitution = 0.15;
                b.visual.color = colors[i % 4];
                bodies.push_back(b);
            }
        }
    });
    
    presets.push_back({"galaxy", "Мини-галактика", "80 звёзд во вращающемся диске",
        [](std::vector<Body>& bodies, SimulationParams& params,
           std::vector<SpringConstraint>& springs) {
            bodies.clear(); springs.clear();
            params = SimulationParams();
            params.gravity = 0;
            params.enable_central_g = true;
            params.central_mass = 150;
            
            Body core(0);
            core.props.mass = 150;
            core.props.radius = 1.0;
            core.visual.color = glm::vec3(1.0f, 0.8f, 0.0f);
            core.visual.glow_color = glm::vec3(1.0f, 0.5f, 0.0f);
            core.flags = BODY_STATIC | BODY_CENTRAL | BODY_VISIBLE;
            bodies.push_back(core);
            
            glm::vec3 colors[] = {
                {0.0f, 1.0f, 0.8f}, {0.27f, 0.53f, 1.0f}, {0.67f, 0.27f, 1.0f},
                {1.0f, 0.0f, 0.67f}, {1.0f, 0.8f, 0.0f}
            };
            std::mt19937 rng(42);
            for (int i = 0; i < 80; ++i) {
                Body b(i + 1);
                double dist = 1.5 + (rng() % 700) * 0.01;
                double angle = (rng() % 1000) * 0.006283;
                double os = 2.0 / std::sqrt(dist);
                b.state.position = dvec3(std::cos(angle) * dist, (rng() % 100 - 50) * 0.005, std::sin(angle) * dist);
                b.state.velocity = dvec3(-std::sin(angle) * os, (rng() % 100 - 50) * 0.002, std::cos(angle) * os);
                b.props.mass = 0.1 + (rng() % 30) * 0.01;
                b.props.radius = 0.08 + (rng() % 10) * 0.01;
                b.visual.color = colors[i % 5];
                bodies.push_back(b);
            }
        }
    });
    
    presets.push_back({"collision_wall", "Стена и снаряд", "Снаряд врезается в стену из блоков",
        [](std::vector<Body>& bodies, SimulationParams& params,
           std::vector<SpringConstraint>& springs) {
            bodies.clear(); springs.clear();
            params = SimulationParams();
            params.gravity = 0;
            params.restitution = 0.3;
            
            // Wall
            for (int x = 0; x < 4; ++x) {
                for (int y = 0; y < 5; ++y) {
                    Body b(x * 5 + y);
                    b.state.position = dvec3(x * 0.9 + 4, y * 0.9 + 0.5, 0);
                    b.props.mass = 0.5;
                    b.props.radius = 0.3;
                    b.visual.color = glm::vec3(0.27f, 0.53f, 1.0f);
                    b.visual.shape = ShapeType::Box;
                    bodies.push_back(b);
                }
            }
            // Projectile
            Body proj(20);
            proj.state.position = dvec3(-8, 2.5, 0);
            proj.state.velocity = dvec3(12, 0.5, 0);
            proj.props.mass = 3.0;
            proj.props.radius = 0.7;
            proj.visual.color = glm::vec3(1.0f, 0.2f, 0.2f);
            proj.visual.glow_color = glm::vec3(1.0f, 0.0f, 0.0f);
            bodies.push_back(proj);
        }
    });
    
    presets.push_back({"black_hole", "Чёрная дыра", "Массивный центр, притягивающий частицы",
        [](std::vector<Body>& bodies, SimulationParams& params,
           std::vector<SpringConstraint>& springs) {
            bodies.clear(); springs.clear();
            params = SimulationParams();
            params.gravity = 0;
            params.enable_central_g = true;
            params.central_mass = 500;
            
            Body hole(0);
            hole.props.mass = 500;
            hole.props.radius = 0.8;
            hole.visual.color = glm::vec3(0.0f, 0.0f, 0.0f);
            hole.visual.glow_color = glm::vec3(0.27f, 0.0f, 0.67f);
            hole.visual.emissive_intensity = 0.3f;
            hole.flags = BODY_STATIC | BODY_CENTRAL | BODY_VISIBLE;
            bodies.push_back(hole);
            
            glm::vec3 colors[] = {
                {1.0f, 0.4f, 0.0f}, {1.0f, 0.8f, 0.0f}, {1.0f, 0.53f, 0.0f},
                {1.0f, 0.27f, 0.0f}, {0.67f, 0.27f, 1.0f}
            };
            std::mt19937 rng(42);
            for (int i = 0; i < 60; ++i) {
                Body b(i + 1);
                double dist = 1.2 + (rng() % 500) * 0.01;
                double angle = (rng() % 1000) * 0.006283;
                double os = 3.0 / std::sqrt(dist);
                b.state.position = dvec3(std::cos(angle) * dist, (rng() % 100 - 50) * 0.003, std::sin(angle) * dist);
                b.state.velocity = dvec3(-std::sin(angle) * os, (rng() % 100 - 50) * 0.001, std::cos(angle) * os);
                b.props.mass = 0.05 + (rng() % 10) * 0.01;
                b.props.radius = 0.08 + (rng() % 8) * 0.01;
                b.visual.color = colors[i % 5];
                bodies.push_back(b);
            }
        }
    });
    
    presets.push_back({"levitation", "Левитация", "Парящие тела над платформой",
        [](std::vector<Body>& bodies, SimulationParams& params,
           std::vector<SpringConstraint>& springs) {
            bodies.clear(); springs.clear();
            params = SimulationParams();
            params.gravity = 9.81;
            params.enable_buoyancy = true;
            params.buoyancy = 0.8;
            
            glm::vec3 colors[] = {
                {0.0f, 1.0f, 0.8f}, {1.0f, 0.0f, 0.67f}, {0.27f, 0.53f, 1.0f},
                {1.0f, 0.8f, 0.0f}, {0.0f, 1.0f, 0.53f}
            };
            for (int i = 0; i < 8; ++i) {
                Body b(i);
                double angle = (double)i / 8.0 * 2.0 * M_PI;
                b.state.position = dvec3(std::cos(angle) * 2, 1.5 + std::sin(i * 2) * 0.5, std::sin(angle) * 2);
                b.state.velocity = dvec3(std::cos(angle + M_PI/2) * 0.5, 0.2, std::sin(angle + M_PI/2) * 0.5);
                b.props.mass = 0.5;
                b.props.radius = 0.3;
                b.props.restitution = 0.8;
                b.visual.color = colors[i % 5];
                bodies.push_back(b);
            }
        }
    });
    
    return presets;
}

void Scene::load_preset(const std::string& id) {
    auto presets = available_presets();
    for (auto& p : presets) {
        if (p.id == id) {
            clear_bodies();
            p.setup(bodies_, params_, springs_);
            // Fix IDs
            next_id_ = 0;
            for (auto& b : bodies_) b.id = next_id_++;
            experiment_id_ = id;
            reset_energy_tracking();
            return;
        }
    }
}

// ==================== Serialization ====================
std::string Scene::serialize_to_json() const {
    nlohmann::json j;
    j["version"] = "2.0";
    j["experiment_id"] = experiment_id_;
    j["sim_time"] = sim_time_;
    j["params"] = params_;
    j["integrator"] = static_cast<int>(integrator_);
    
    nlohmann::json bodies_arr = nlohmann::json::array();
    for (const auto& b : bodies_) {
        nlohmann::json jb;
        to_json(jb, b);
        bodies_arr.push_back(jb);
    }
    j["bodies"] = bodies_arr;
    
    nlohmann::json springs_arr = nlohmann::json::array();
    for (const auto& s : springs_) {
        springs_arr.push_back({
            {"body_a", s.body_a}, {"body_b", s.body_b},
            {"rest_length", s.rest_length}, {"stiffness", s.stiffness},
            {"damping", s.damping}
        });
    }
    j["springs"] = springs_arr;
    
    return j.dump(2);
}

bool Scene::deserialize_from_json(const std::string& json) {
    try {
        auto j = nlohmann::json::parse(json);
        clear_bodies();
        
        experiment_id_ = j.value("experiment_id", "sandbox");
        sim_time_ = j.value("sim_time", 0.0);
        
        if (j.contains("params")) from_json(j["params"], params_);
        if (j.contains("integrator")) 
            integrator_ = static_cast<IntegratorType>(j["integrator"].get<int>());
        
        if (j.contains("bodies")) {
            for (const auto& jb : j["bodies"]) {
                Body b(0);
                from_json(jb, b);
                bodies_.push_back(b);
            }
        }
        
        if (j.contains("springs")) {
            for (const auto& js : j["springs"]) {
                springs_.push_back({
                    js["body_a"], js["body_b"],
                    js["rest_length"], js["stiffness"], js["damping"]
                });
            }
        }
        
        next_id_ = bodies_.size();
        reset_energy_tracking();
        return true;
    } catch (const std::exception& e) {
        std::cerr << "Deserialization error: " << e.what() << std::endl;
        return false;
    }
}

bool Scene::save_to_file(const std::string& path) const {
    try {
        std::ofstream file(path);
        file << serialize_to_json();
        return true;
    } catch (...) {
        return false;
    }
}

bool Scene::load_from_file(const std::string& path) {
    try {
        std::ifstream file(path);
        std::string content((std::istreambuf_iterator<char>(file)),
                             std::istreambuf_iterator<char>());
        return deserialize_from_json(content);
    } catch (...) {
        return false;
    }
}

} // namespace hlp
