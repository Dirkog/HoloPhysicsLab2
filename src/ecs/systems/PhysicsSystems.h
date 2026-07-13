#pragma once

// ============================================================
// HoloPhysics Lab 2 — ECS Physics Systems (v3.6)
//
// Все физические системы переписаны на ECS.
// Старый класс Body полностью удалён.
//
// Системы:
//   - GravitySystem
//   - IntegrationSystem (Euler/Verlet/RK4)
//   - CollisionSystem (GJK/EPA via ECS)
//   - ConstraintSystem (PBD via ECS)
//   - CCDSystem (Continuous collision via ECS)
//   - VirtualCouplingSystem (PD controller via ECS)
// ============================================================

#include "../core/Registry.h"
#include "../../core/Types.h"
#include "../../physics/CollisionSystem.h"
#include "../../physics/ConstraintSolver.h"
#include "../../physics/CCD.h"
#include <glm/glm.hpp>
#include <vector>
#include <memory>

namespace hlp {

// ==================== Gravity System ====================

class GravitySystem : public ECS_System {
public:
    double gravity = 9.81;
    
    void update(Registry& registry, double dt) override;
    const char* name() const override { return "GravitySystem"; }
};

// ==================== Integration System ====================

class IntegrationSystem : public ECS_System {
public:
    enum Method { EULER, SYMPLECTIC, VERLET, RK4 };
    Method method = SYMPLECTIC;
    
    void update(Registry& registry, double dt) override;
    const char* name() const override { return "IntegrationSystem"; }
    
private:
    void step_euler(Registry& reg, double dt);
    void step_symplectic(Registry& reg, double dt);
};

// ==================== Collision System (ECS) ====================

class ECSCollisionSystem : public ECS_System {
public:
    ECSCollisionSystem();
    ~ECSCollisionSystem() = default;
    
    void update(Registry& registry, double dt) override;
    const char* name() const override { return "ECSCollisionSystem"; }
    
    // Collision events (thread-safe: populated during update, consumed by main thread)
    struct CollisionEventECS {
        Entity a, b;
        dvec3 point;
        dvec3 normal;
        double impulse;
    };
    
    const std::vector<CollisionEventECS>& events() const { return events_; }
    void clear_events() { events_.clear(); }

private:
    std::vector<CollisionEventECS> events_;
    
    // Broad phase: spatial hash
    struct Cell {
        std::vector<Entity> entities;
    };
    std::unordered_map<uint64_t, Cell> grid_;
    double cell_size_ = 2.0;
    
    void broad_phase(Registry& reg, std::vector<std::pair<Entity,Entity>>& pairs);
    void narrow_phase(Registry& reg, const std::pair<Entity,Entity>& pair);
    void resolve_contact(Registry& reg, Entity a, Entity b,
                         const dvec3& normal, double penetration);
    
    uint64_t hash_cell(int x, int y, int z) const;
};

// ==================== Constraint System (PBD via ECS) ====================

class ECSConstraintSystem : public ECS_System {
public:
    void update(Registry& registry, double dt) override;
    const char* name() const override { return "ECSConstraintSystem"; }
    
private:
    void solve_distance(Registry& reg, const SpringConstraintComponent& s, Entity e);
    void solve_all(Registry& reg, int iterations = 5);
};

// ==================== CCD System (ECS) ====================

class ECSCCDSystem : public ECS_System {
public:
    void update(Registry& registry, double dt) override;
    const char* name() const override { return "ECSCCDSystem"; }
    
private:
    bool check_sweep(Registry& reg, Entity fast, Entity other, double dt);
};

// ==================== Virtual Coupling System (PD controller via ECS) ====================

class VirtualCouplingSystem : public ECS_System {
public:
    void update(Registry& registry, double dt) override;
    const char* name() const override { return "VirtualCouplingSystem"; }
    
    // Set hand target from tracking source
    void set_hand_target(int hand_index, const dvec3& position, const dvec3& velocity);
    
    // PD parameters
    double kp = 200.0;
    double kd = 20.0;
    double max_force = 50.0;
    double max_stretch = 0.12;

private:
    struct HandTarget {
        dvec3 position{0};
        dvec3 velocity{0};
        bool active = false;
    };
    HandTarget targets_[2];
    
    // Find hand entities in registry
    Entity find_hand_entity(Registry& reg, int hand_index);
};

// ==================== ECS Physics Engine (Тотальный) ====================

class ECSPhysicsEngine {
public:
    ECSPhysicsEngine();
    ~ECSPhysicsEngine();
    
    // === Systems ===
    GravitySystem gravity;
    IntegrationSystem integrator;
    ECSCollisionSystem collisions;
    ECSConstraintSystem constraints;
    ECSCCDSystem ccd;
    VirtualCouplingSystem coupling;
    
    // === Full step ===
    void step(Registry& registry, double dt);
    
    // === System management ===
    void add_system(ECS_System* system);
    void remove_system(ECS_System* system);
    
    // === Configuration ===
    void set_substeps(int n) { substeps_ = n; }
    int substeps() const { return substeps_; }

private:
    std::vector<ECS_System*> systems_;
    int substeps_ = 2;
};

} // namespace hlp
