#pragma once

// ============================================================
// HoloPhysics Lab 2 — Унифицированный ECS (v3.6)
// 
// ЕДИНСТВЕННАЯ точка правды для всех физических данных.
// Старый класс Body полностью удалён.
//
// Все системы (VirtualCoupling, GJK/EPA, PBD, CCD) работают
// исключительно через ECS Registry.
// ============================================================

#include "../../core/Types.h"
#include <vector>
#include <bitset>
#include <unordered_map>
#include <functional>
#include <atomic>
#include <cstring>
#include <glm/glm.hpp>

namespace hlp {

// ==================== Entity ====================

using Entity = uint32_t;
constexpr Entity NULL_ENTITY = UINT32_MAX;

// ==================== Component Types ====================

enum class ComponentType : uint8_t {
    TRANSFORM = 0,
    PHYSICS,
    COLLIDER,
    RENDER,
    SPRING_CONSTRAINT,
    HAND_TAG,       // Marks entity as a virtual hand
    SCRIPT_TAG,     // Has attached Lua script
    GHOST_TAG,      // Trigger-only collider (no physics response)
    COUNT
};

// ==================== Components (SoA layout) ====================

struct TransformComponent {
    dvec3 position{0};
    dquat rotation{1, 0, 0, 0};
    dvec3 scale{1, 1, 1};
    
    // History for interpolation
    dvec3 prev_position{0};
    dquat prev_rotation{1, 0, 0, 0};
};

struct PhysicsComponent {
    dvec3 velocity{0};
    dvec3 force{0};
    dvec3 torque{0};
    dvec3 angular_velocity{0};
    double mass = 1.0;
    double inv_mass = 1.0;
    double gravity_scale = 1.0;
    double damping = 0.01;
    bool is_static = false;
    bool is_kinematic = false;  // PD-controlled (hand)
};

struct ColliderComponent {
    ShapeType shape = ShapeType::Sphere;
    double radius = 0.5;
    double restitution = 0.5;
    double friction = 0.3;
    double charge = 0.0;
    uint64_t collision_mask = 0xFFFFFFFF;
    bool is_trigger = false;
    
    // For GJK
    dvec3 half_extents{0.5, 0.5, 0.5};
};

struct RenderComponent {
    glm::vec3 color{0.0f, 1.0f, 0.8f};
    glm::vec3 glow_color{0.0f, 1.0f, 0.8f};
    float emissive_intensity = 0.15f;
    float opacity = 0.92f;
    bool visible = true;
    bool wireframe = false;
};

struct SpringConstraintComponent {
    Entity body_a = NULL_ENTITY;
    Entity body_b = NULL_ENTITY;
    double rest_length = 1.0;
    double stiffness = 100.0;
    double damping = 1.0;
    bool active = true;
};

struct HandTagComponent {
    int hand_index = 0;      // 0=right, 1=left
    bool is_pinch = false;
    double pinch_strength = 0;
};

// ==================== Sparse Set (для быстрых lookup/removal) ====================

template<typename T>
class SparseSet {
public:
    SparseSet() : dense_(nullptr), sparse_(nullptr), capacity_(0), size_(0) {}
    
    ~SparseSet() {
        free(dense_);
        free(sparse_);
    }
    
    void init(int max_entities = 10000) {
        capacity_ = max_entities;
        dense_ = (T*)malloc(max_entities * sizeof(T));
        sparse_ = (int*)malloc(max_entities * sizeof(int));
        memset(sparse_, -1, max_entities * sizeof(int));
        entity_ids_.reserve(max_entities);
    }
    
    bool has(Entity e) const {
        if (e >= (Entity)capacity_) return false;
        int idx = sparse_[e];
        return idx >= 0 && idx < size_ && entity_ids_[idx] == e;
    }
    
    T& get(Entity e) {
        return dense_[sparse_[e]];
    }
    
    const T& get(Entity e) const {
        return dense_[sparse_[e]];
    }
    
    T& add(Entity e, const T& comp = T{}) {
        if (has(e)) return get(e);
        
        int idx = size_++;
        if (idx >= capacity_) {
            capacity_ *= 2;
            dense_ = (T*)realloc(dense_, capacity_ * sizeof(T));
            sparse_ = (int*)realloc(sparse_, capacity_ * sizeof(int));
            entity_ids_.reserve(capacity_);
        }
        
        dense_[idx] = comp;
        sparse_[e] = idx;
        entity_ids_[idx] = e;
        return dense_[idx];
    }
    
    void remove(Entity e) {
        if (!has(e)) return;
        
        int idx = sparse_[e];
        int last = --size_;
        
        // Swap with last element
        Entity last_entity = entity_ids_[last];
        dense_[idx] = dense_[last];
        sparse_[last_entity] = idx;
        entity_ids_[idx] = last_entity;
        
        sparse_[e] = -1;
    }
    
    void clear() { size_ = 0; }
    size_t size() const { return size_; }
    bool empty() const { return size_ == 0; }
    
    // Iteration
    int* begin() { return sparse_; }
    int* end() { return sparse_ + capacity_; }
    
    const std::vector<Entity>& entities() const { return entity_ids_; }

private:
    T* dense_;
    int* sparse_;
    std::vector<Entity> entity_ids_;
    int capacity_;
    int size_;
};

// ==================== Registry (ЕДИНСТВЕННАЯ точка правды) ====================

class Registry {
public:
    Registry() {
        transforms_.init();
        physics_.init();
        colliders_.init();
        render_.init();
        springs_.init();
        hand_tags_.init();
    }
    
    // === Entity Management ===
    Entity create() {
        if (!free_list_.empty()) {
            Entity e = free_list_.back();
            free_list_.pop_back();
            return e;
        }
        return next_entity_++;
    }
    
    void destroy(Entity e) {
        if (e >= next_entity_) return;
        free_list_.push_back(e);
        
        transforms_.remove(e);
        physics_.remove(e);
        colliders_.remove(e);
        render_.remove(e);
        springs_.remove(e);
        hand_tags_.remove(e);
    }
    
    // === Component Access ===
    SparseSet<TransformComponent>& transforms() { return transforms_; }
    SparseSet<PhysicsComponent>& physics() { return physics_; }
    SparseSet<ColliderComponent>& colliders() { return colliders_; }
    SparseSet<RenderComponent>& render_components() { return render_; }
    SparseSet<SpringConstraintComponent>& springs() { return springs_; }
    SparseSet<HandTagComponent>& hand_tags() { return hand_tags_; }
    
    // === Component Existence Check ===
    bool has_transform(Entity e) const { return transforms_.has(e); }
    bool has_physics(Entity e) const { return physics_.has(e); }
    bool has_collider(Entity e) const { return colliders_.has(e); }
    bool has_render(Entity e) const { return render_.has(e); }
    bool has_spring(Entity e) const { return springs_.has(e); }
    bool has_hand_tag(Entity e) const { return hand_tags_.has(e); }
    
    // === Query: all entities with a given set of components ===
    template<typename... Comps>
    std::vector<Entity> query() const {
        std::vector<Entity> result;
        // Simple approach: iterate transform (most common component)
        for (size_t i = 0; i < transforms_.size(); i++) {
            Entity e = transforms_.entities()[i];
            if (check_components<Comps...>(e)) {
                result.push_back(e);
            }
        }
        return result;
    }
    
    // === Utility ===
    size_t entity_count() const { return next_entity_ - free_list_.size(); }
    size_t component_count() const {
        return transforms_.size() + physics_.size() + colliders_.size() 
             + render_.size() + springs_.size() + hand_tags_.size();
    }
    
    void clear() {
        transforms_.clear();
        physics_.clear();
        colliders_.clear();
        render_.clear();
        springs_.clear();
        hand_tags_.clear();
        free_list_.clear();
        next_entity_ = 0;
    }

private:
    Entity next_entity_ = 1;
    std::vector<Entity> free_list_;
    
    SparseSet<TransformComponent> transforms_;
    SparseSet<PhysicsComponent> physics_;
    SparseSet<ColliderComponent> colliders_;
    SparseSet<RenderComponent> render_;
    SparseSet<SpringConstraintComponent> springs_;
    SparseSet<HandTagComponent> hand_tags_;
    
    template<typename Comp, typename... Rest>
    bool check_components(Entity e) const {
        return has_component<Comp>(e) && check_components<Rest...>(e);
    }
    
    bool check_components() const { return true; }
    
    template<typename Comp>
    bool has_component(Entity e) const;
};

// Component-specific has_component specializations
template<> inline bool Registry::has_component<TransformComponent>(Entity e) const { return transforms_.has(e); }
template<> inline bool Registry::has_component<PhysicsComponent>(Entity e) const { return physics_.has(e); }
template<> inline bool Registry::has_component<ColliderComponent>(Entity e) const { return colliders_.has(e); }
template<> inline bool Registry::has_component<RenderComponent>(Entity e) const { return render_.has(e); }
template<> inline bool Registry::has_component<SpringConstraintComponent>(Entity e) const { return springs_.has(e); }
template<> inline bool Registry::has_component<HandTagComponent>(Entity e) const { return hand_tags_.has(e); }

// ==================== Base System ====================

class ECS_System {
public:
    virtual ~ECS_System() = default;
    virtual void update(Registry& registry, double dt) = 0;
    virtual const char* name() const = 0;
};

} // namespace hlp
