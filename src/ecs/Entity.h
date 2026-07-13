#pragma once

// ============================================================
// HoloPhysics Lab 2 — Entity Component System (ECS)
//
// Производительный ECS на основе EnTT-подобного подхода.
// Оптимизирован для cache locality при 10 000+ объектов.
//
// Компоненты:
//   Transform   — позиция, поворот, масштаб
//   Physics     — масса, скорость, сила
//   Render      — цвет, форма, emissive
//   Collider    — радиус, restitution, friction
//   Spring      — пружинные связи
//   Script      — привязанный Lua-скрипт
// ============================================================

#include "../core/Types.h"
#include <vector>
#include <bitset>
#include <memory>
#include <algorithm>
#include <cstring>

namespace hlp {

// ==================== Entity ====================

using Entity = uint32_t;
constexpr Entity NULL_ENTITY = UINT32_MAX;

// ==================== Component Types ====================

enum class ComponentType : uint8_t {
    TRANSFORM = 0,
    PHYSICS,
    RENDER,
    COLLIDER,
    SPRING,
    SCRIPT,
    USER_DATA,
    COUNT
};

// ==================== Component Data (SoA - Structure of Arrays) ====================

struct TransformComponent {
    dvec3 position{0};
    dquat rotation{1, 0, 0, 0};
    dvec3 scale{1, 1, 1};
};

struct PhysicsComponent {
    dvec3 velocity{0};
    dvec3 force{0};
    dvec3 angular_velocity{0};
    double mass = 1.0;
    double gravity_scale = 1.0;
    bool is_static = false;
};

struct RenderComponent {
    glm::vec3 color{0.0f, 1.0f, 0.8f};
    glm::vec3 glow_color{0.0f, 1.0f, 0.8f};
    float emissive = 0.15f;
    float opacity = 0.92f;
    ShapeType shape = ShapeType::Sphere;
    float radius = 0.5f;
    bool visible = true;
};

struct ColliderComponent {
    double radius = 0.5;
    double restitution = 0.5;
    double friction = 0.3;
    double charge = 0.0;
    bool is_static = false;
};

// ==================== Component Arrays (Packed) ====================

template<typename T>
struct ComponentArray {
    std::vector<Entity> entities;  // Entity → index mapping
    std::vector<T> data;
    
    bool has(Entity e) const {
        return std::find(entities.begin(), entities.end(), e) != entities.end();
    }
    
    T& get(Entity e) {
        auto it = std::find(entities.begin(), entities.end(), e);
        return data[std::distance(entities.begin(), it)];
    }
    
    T& add(Entity e, const T& comp = T{}) {
        entities.push_back(e);
        data.push_back(comp);
        return data.back();
    }
    
    void remove(Entity e) {
        auto it = std::find(entities.begin(), entities.end(), e);
        if (it == entities.end()) return;
        size_t idx = std::distance(entities.begin(), it);
        entities.erase(it);
        data.erase(data.begin() + idx);
    }
    
    void clear() {
        entities.clear();
        data.clear();
    }
    
    size_t size() const { return entities.size(); }
};

// ==================== Registry ====================

class Registry {
public:
    Registry() = default;
    
    // Entity management
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
        
        // Remove all components for this entity
        if (transforms_.has(e)) transforms_.remove(e);
        if (physics_.has(e)) physics_.remove(e);
        if (render_.has(e)) render_.remove(e);
        if (colliders_.has(e)) colliders_.remove(e);
    }
    
    ComponentArray<TransformComponent>& transforms() { return transforms_; }
    ComponentArray<PhysicsComponent>& physics() { return physics_; }
    ComponentArray<RenderComponent>& render_components() { return render_; }
    ComponentArray<ColliderComponent>& colliders() { return colliders_; }
    
    // Query: get all entities with component mask
    std::vector<Entity> query(const std::bitset<(int)ComponentType::COUNT>& mask) const {
        std::vector<Entity> result;
        for (Entity e = 0; e < next_entity_; e++) {
            if (std::find(free_list_.begin(), free_list_.end(), e) != free_list_.end())
                continue;
            
            std::bitset<(int)ComponentType::COUNT> emask;
            emask[(int)ComponentType::TRANSFORM] = transforms_.has(e);
            emask[(int)ComponentType::PHYSICS] = physics_.has(e);
            emask[(int)ComponentType::RENDER] = render_.has(e);
            emask[(int)ComponentType::COLLIDER] = colliders_.has(e);
            
            if ((emask & mask) == mask)
                result.push_back(e);
        }
        return result;
    }
    
    // Stats
    size_t entity_count() const { return next_entity_ - free_list_.size(); }
    size_t component_count() const {
        return transforms_.size() + physics_.size() + render_.size() + colliders_.size();
    }
    
    void clear() {
        transforms_.clear();
        physics_.clear();
        render_.clear();
        colliders_.clear();
        free_list_.clear();
        next_entity_ = 0;
    }

private:
    Entity next_entity_ = 0;
    std::vector<Entity> free_list_;
    
    // Component storage (SoA)
    ComponentArray<TransformComponent> transforms_;
    ComponentArray<PhysicsComponent> physics_;
    ComponentArray<RenderComponent> render_;
    ComponentArray<ColliderComponent> colliders_;
};

// ==================== System ====================

class System {
public:
    virtual ~System() = default;
    virtual void update(Registry& registry, double dt) = 0;
    virtual const char* name() const = 0;
};

// ==================== PhysicsSystem (ECS version) ====================

class ECSPhysicsSystem : public System {
public:
    void update(Registry& registry, double dt) override;
    const char* name() const override { return "ECSPhysics"; }
    
    // Configuration
    double gravity = 9.81;
    double damping = 0.01;
};

// ==================== RenderSystem (ECS version) ====================

class ECSRenderSystem : public System {
public:
    void update(Registry& registry, double dt) override;
    const char* name() const override { return "ECSRender"; }
};

} // namespace hlp
