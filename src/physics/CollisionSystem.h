#pragma once

// ============================================================
// HoloPhysics Lab 2 — Collision Detection & Response System
// Broad phase (BVH/SpatialHash) + Narrow phase (GJK/EPA) + Response
// ============================================================

#include "../core/Types.h"
#include <vector>
#include <array>
#include <unordered_map>
#include <functional>
#include <memory>

namespace hlp {

// ==================== Collision Primitives ====================

struct Collider {
    enum Type { SPHERE = 0, PLANE, BOX, CAPSULE, MESH };
    Type type;
    double radius = 0.5;          // Sphere/Capsule radius
    double half_height = 0.0;     // Box half-height/Capsule half-height
    dvec3 half_extents{0.5, 0.5, 0.5}; // Box extents
    dvec3 normal{0, 1, 0};        // Plane normal
    double offset = 0;            // Plane offset from origin
    
    // For mesh colliders
    std::vector<dvec3> mesh_vertices;
    std::vector<std::array<int, 3>> mesh_triangles;
};

struct ContactPoint {
    dvec3 point;           // World-space contact point
    dvec3 normal;          // Normal from A to B
    double penetration;    // Penetration depth
    double restitution;    // Combined restitution coefficient
    double friction;       // Combined friction coefficient
    uint64_t body_a_id;    // First body ID
    uint64_t body_b_id;    // Second body ID (UINT64_MAX for static)
    bool is_static_b;      // Is body B static?
};

struct CollisionPair {
    uint64_t a, b;
    double distance_sq;
};

// ==================== BVH Node ====================
struct BVHNode {
    dvec3 min_bound, max_bound;
    int left_child = -1;       // Index of left child
    int right_child = -1;      // Index of right child
    int body_id = -1;          // Body ID if leaf, -1 if internal
    int axis = 0;              // Split axis (0=x, 1=y, 2=z)
    bool is_leaf = false;
};

class BVH {
public:
    void build(const std::vector<Body>& bodies);
    void query(const dvec3& min_b, const dvec3& max_b, 
               std::vector<int>& results) const;
    void query_ray(const dvec3& origin, const dvec3& dir,
                   std::vector<int>& results) const;
    void clear();
    
private:
    std::vector<BVHNode> nodes_;
    int root_ = -1;
    
    int build_recursive(std::vector<int>& indices, int depth);
    dvec3 compute_centroid(const std::vector<Body>& bodies, 
                           const std::vector<int>& indices);
    void compute_bounds(const std::vector<Body>& bodies,
                        const std::vector<int>& indices,
                        dvec3& min_b, dvec3& max_b);
};

// ==================== Spatial Hash Grid ====================
class SpatialHash {
public:
    SpatialHash(double cell_size = 2.0);
    void build(const std::vector<Body>& bodies);
    void get_potential_pairs(std::vector<CollisionPair>& pairs) const;
    void clear();
    
private:
    double cell_size_;
    std::unordered_map<uint64_t, std::vector<int>> grid_;
    
    uint64_t hash_coord(int x, int y, int z) const;
    void get_cells_for_body(const Body& b, std::vector<uint64_t>& cells) const;
};

// ==================== GJK + EPA ====================
class GJK {
public:
    // Returns true if shapes intersect
    bool intersect(const Body& a, const Body& b);
    
    // Full GJK+EPA: returns contact point details
    bool get_contact(const Body& a, const Body& b,
                     dvec3& contact_point, dvec3& normal, 
                     double& penetration);
    
private:
    // Support functions for different shapes
    dvec3 support(const Body& body, const dvec3& dir) const;
    dvec3 support_sphere(const Body& body, const dvec3& dir) const;
    dvec3 support_box(const Body& body, const dvec3& dir) const;
    dvec3 support_cylinder(const Body& body, const dvec3& dir) const;
    dvec3 support_torus(const Body& body, const dvec3& dir) const;
    
    // Simplex operations
    bool solve_simplex(std::vector<dvec3>& simplex, dvec3& dir);
    bool point_in_tetrahedron(const std::vector<dvec3>& simplex, 
                              const dvec3& point) const;
    
    // EPA (Expanding Polytope Algorithm)
    bool epa(const std::vector<dvec3>& simplex,
             dvec3& contact_point, dvec3& normal, double& penetration);
};

// ==================== Collision System ====================
class CollisionSystem {
public:
    CollisionSystem();
    ~CollisionSystem();
    
    // Main collision detection pass
    void detect_collisions(std::vector<Body>& bodies,
                           const SimulationParams& params,
                           double dt,
                           std::vector<CollisionEvent>& events);
    
    // Resolve collisions with impulse-based response
    void resolve_collisions(std::vector<Body>& bodies,
                            const std::vector<ContactPoint>& contacts,
                            double dt);
    
    // Sequential impulse solver for stacked bodies
    void solve_constraints(std::vector<Body>& bodies,
                           std::vector<ContactPoint>& contacts,
                           int iterations = 10);
    
    // Static colliders management
    void add_static_collider(const Collider& collider);
    void clear_static_colliders();
    const std::vector<Collider>& static_colliders() const { return static_colliders_; }
    
    // Debug data
    const std::vector<ContactPoint>& last_contacts() const { return last_contacts_; }
    void clear_last_contacts() { last_contacts_.clear(); }
    
    // Spatial structure selection
    enum BroadPhaseType { BRUTE_FORCE, SPATIAL_HASH, BVH };
    void set_broad_phase(BroadPhaseType type) { broad_phase_ = type; }
    
private:
    // Broad phase
    void broad_phase(const std::vector<Body>& bodies,
                     std::vector<CollisionPair>& pairs);
    
    // Narrow phase
    void narrow_phase(const std::vector<Body>& bodies,
                      const std::vector<CollisionPair>& pairs,
                      std::vector<ContactPoint>& contacts);
    
    // Sphere-sphere fast path
    bool detect_sphere_sphere(const Body& a, const Body& b,
                              ContactPoint& contact);
    
    // Static collider detection
    void detect_static_collisions(std::vector<Body>& bodies,
                                  std::vector<ContactPoint>& contacts);
    
    // Collider-body intersection
    bool collider_vs_body(const Collider& collider, const Body& body,
                          ContactPoint& contact);
    
    // Apply impulse at contact
    void apply_contact_impulse(Body& a, Body& b,
                               const ContactPoint& contact);
    
    // Configuration
    BroadPhaseType broad_phase_ = SPATIAL_HASH;
    std::unique_ptr<SpatialHash> spatial_hash_;
    std::unique_ptr<BVH> bvh_;
    GJK gjk_;
    
    // Static environment
    std::vector<Collider> static_colliders_;
    
    // Last frame data
    std::vector<ContactPoint> last_contacts_;
    
    // Warm starting (store accumulated impulses)
    struct WarmContact {
        uint64_t a, b;
        double accumulated_normal_impulse = 0;
        double accumulated_tangent_impulse = 0;
    };
    std::vector<WarmContact> warm_contacts_;
};

} // namespace hlp
