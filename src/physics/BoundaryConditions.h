#pragma once

// ============================================================
// HoloPhysics Lab 2 — Boundary Conditions
// Walls, periodic boundaries, absorbing layers, SDF
// ============================================================

#include "../core/Types.h"
#include <vector>
#include <functional>

namespace hlp {

// ==================== Boundary Types ====================

enum class BoundaryType : uint8_t {
    REFLECTIVE,      // Bounce back (with restitution)
    ABSORBING,       // Remove particles that cross
    PERIODIC,        // Wrap around (torus)
    OPEN,            // Let through (dissipative)
    THERMAL,         // Fixed temperature wall
    SLIP,            // Free-slip (tangential movement allowed)
    STICKY           // No-slip (zero velocity at wall)
};

struct PlaneBoundary {
    dvec3 normal{0, 1, 0};   // Normal pointing inward
    double offset = -10;       // Distance from origin along normal
    BoundaryType type = BoundaryType::REFLECTIVE;
    double restitution = 0.5;
    double friction = 0.3;
    double temperature = 293.0;
};

struct BoxBoundary {
    dvec3 min{-10, -1, -10};
    dvec3 max{10, 20, 10};
    BoundaryType type = BoundaryType::REFLECTIVE;
    double restitution = 0.5;
    double friction = 0.3;
};

struct SphereBoundary {
    dvec3 center{0, 0, 0};
    double radius = 15.0;
    BoundaryType type = BoundaryType::REFLECTIVE;
    double restitution = 0.5;
};

// ==================== Signed Distance Functions ====================

class SDF {
public:
    SDF() = default;
    
    // Distance to a point (negative = inside)
    double distance(const dvec3& point) const;
    
    // Gradient at a point (surface normal)
    dvec3 gradient(const dvec3& point) const;
    
    // Add primitives
    void add_sphere(const dvec3& center, double radius);
    void add_box(const dvec3& center, const dvec3& half_extents);
    void add_plane(const dvec3& normal, double offset);
    void add_cylinder(const dvec3& center, double radius, double height);
    void add_torus(const dvec3& center, double major, double minor);
    
    // CSG operations
    void union_with(const SDF& other);
    void intersect_with(const SDF& other);
    void subtract(const SDF& other);
    
    // Clear
    void clear();
    
private:
    struct SDFNode {
        enum Type { SPHERE, BOX, PLANE, CYLINDER, TORUS, UNION, INTERSECT, SUBTRACT };
        Type type;
        dvec3 center{0};
        double radius = 1.0;
        dvec3 half_extents{1, 1, 1};
        dvec3 normal{0, 1, 0};
        double offset = 0;
        double height = 1.0;
        double major = 1.0, minor = 0.3;
        
        std::unique_ptr<SDFNode> left, right;
    };
    
    std::unique_ptr<SDFNode> root_;
    
    double eval_node(const SDFNode* node, const dvec3& point) const;
};

// ==================== Boundary Conditions System ====================

class BoundarySystem {
public:
    BoundarySystem();
    ~BoundarySystem();

    // === Configuration ===
    void add_plane_boundary(const PlaneBoundary& boundary);
    void add_box_boundary(const BoxBoundary& boundary);
    void add_sphere_boundary(const SphereBoundary& boundary);
    
    void clear_boundaries();
    
    // SDF-based boundary
    void set_sdf(std::unique_ptr<SDF> sdf);
    SDF* sdf() { return sdf_.get(); }
    
    // === Application ===
    // Apply all boundaries to all bodies
    void apply(std::vector<Body>& bodies, double dt);
    
    // Apply to single body
    void apply_to_body(Body& body, double dt);
    
    // === Periodic boundary utilities ===
    void set_periodic_box(const dvec3& box_size);
    bool is_periodic() const { return periodic_enabled_; }
    dvec3 apply_periodic(const dvec3& pos) const;
    
    // === Debug ===
    const std::vector<PlaneBoundary>& planes() const { return planes_; }
    const std::vector<BoxBoundary>& boxes() const { return boxes_; }
    const std::vector<SphereBoundary>& spheres() const { return spheres_; }
    int active_boundary_count() const;

private:
    std::vector<PlaneBoundary> planes_;
    std::vector<BoxBoundary> boxes_;
    std::vector<SphereBoundary> spheres_;
    std::unique_ptr<SDF> sdf_;
    
    // Periodic
    bool periodic_enabled_ = false;
    dvec3 periodic_box_size_{20, 20, 20};
    
    // Apply single boundary type
    void apply_plane(Body& body, const PlaneBoundary& b, double dt);
    void apply_box(Body& body, const BoxBoundary& b, double dt);
    void apply_sphere(Body& body, const SphereBoundary& b, double dt);
    void apply_periodic_to_body(Body& body);
    void apply_thermal(Body& body, const PlaneBoundary& b);
};

} // namespace hlp
