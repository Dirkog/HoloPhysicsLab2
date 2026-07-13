#pragma once

// ============================================================
// HoloPhysics Lab 2 — Continuous Collision Detection (CCD)
//
// Предотвращает туннелирование быстрых тел сквозь
// статические/медленные объекты.
//
// Алгоритм: Conservative Advancement / Ray-Sphere TOI
// ============================================================

#include "../core/Types.h"
#include <vector>
#include <limits>

namespace hlp {

struct CCDConfig {
    bool enabled = true;
    double ccd_threshold = 2.0;      // Bodies moving faster than this get CCD
    double min_toi = 1e-6;           // Minimum time of impact
    int max_iterations = 8;           // Conservative advancement iterations
    bool use_ccd_for_hand = true;    // Enable CCD for virtual hand too
};

struct TOIResult {
    double toi = 1.0;                 // Time of impact (1.0 = no impact)
    dvec3 contact_point{0};
    dvec3 contact_normal{0};
    dvec3 contact_velocity{0};
    uint64_t body_a = UINT64_MAX;
    uint64_t body_b = UINT64_MAX;
    bool has_impact = false;
};

class CCDSolver {
public:
    CCDSolver();
    ~CCDSolver();

    // === Main CCD step ===
    // Sweep all fast bodies against all others
    // Returns modified velocities/positions for impacted bodies
    void solve(std::vector<Body>& bodies, double dt);
    
    // === Single body sweep ===
    TOIResult sweep_body(const Body& body, const dvec3& next_pos,
                          const Body* other, double dt);
    
    // === Ray-Sphere TOI ===
    // Compute time of impact between a moving sphere and a static sphere
    static double ray_sphere_toi(const dvec3& origin, const dvec3& dir,
                                  double radius, const dvec3& sphere_center,
                                  double sphere_radius);
    
    // === Ray-Box TOI ===
    static double ray_box_toi(const dvec3& origin, const dvec3& dir,
                               const dvec3& box_min, const dvec3& box_max);
    
    // === Swept Sphere-Sphere ===
    static TOIResult swept_sphere_sphere(const Body& a, const Body& b, double dt);
    
    // === Configuration ===
    void set_config(const CCDConfig& cfg) { config_ = cfg; }
    const CCDConfig& config() const { return config_; }
    CCDConfig& config() { return config_; }
    
    // Stats
    int last_ccd_checks() const { return last_checks_; }
    int last_ccd_hits() const { return last_hits_; }

private:
    CCDConfig config_;
    int last_checks_ = 0;
    int last_hits_ = 0;
    
    // Conservative advancement
    TOIResult conservative_advancement(const Body& a, const Body& b, double dt);
    
    // Get bounding sphere for body (at given position)
    static dvec3 get_bounding_center(const Body& b);
    static double get_bounding_radius(const Body& b);
};

} // namespace hlp
