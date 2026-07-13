// ============================================================
// HoloPhysics Lab 2 — Comprehensive Physics Tests
// Проверка всех физических подсистем
// ============================================================

#include <iostream>
#include <cmath>
#include <cassert>
#include <vector>
#include "../core/Types.h"
#include "../core/Solver.h"
#include "../physics/CollisionSystem.h"
#include "../physics/ConstraintSolver.h"
#include "../physics/RotationalDynamics.h"
#include "../physics/AdaptiveTimestep.h"
#include "../physics/BoundaryConditions.h"

using namespace hlp;

static int tests_passed = 0;
static int tests_failed = 0;

#define TEST(name, expr) do { \
    if (!(expr)) { \
        std::cerr << "❌ FAIL: " << name << std::endl; \
        tests_failed++; \
    } else { \
        std::cout << "✅ PASS: " << name << std::endl; \
        tests_passed++; \
    } \
} while(0)

#define TEST_NEAR(name, a, b, eps) TEST(name, std::abs((a) - (b)) < (eps))

// ==================== COLLISION TESTS ====================
void test_collision_sphere_sphere() {
    CollisionSystem cs;
    
    Body a(0), b(1);
    a.props.radius = 1.0;
    b.props.radius = 1.0;
    a.state.position = dvec3(0, 0, 0);
    b.state.position = dvec3(1.5, 0, 0);
    
    std::vector<Body> bodies = {a, b};
    SimulationParams params;
    std::vector<CollisionEvent> events;
    
    // Should not collide (distance 1.5, sum of radii = 2.0 -> separated)
    cs.detect_collisions(bodies, params, 0.016, events);
    TEST("Sphere-sphere: no collision when separated", events.empty());
    
    // Move closer
    bodies[1].state.position = dvec3(1.3, 0, 0);
    events.clear();
    cs.detect_collisions(bodies, params, 0.016, events);
    TEST("Sphere-sphere: collision when overlapping", !events.empty());
    
    if (!events.empty()) {
        TEST_NEAR("Sphere-sphere: collision point at midpoint", 
                  events[0].point.x, 0.65, 0.1);
        TEST("Sphere-sphere: normal points from A to B", 
             events[0].normal.x > 0);
    }
}

void test_collision_static_boundary() {
    CollisionSystem cs;
    cs.add_static_collider({Collider::PLANE, 0, 0, {0}, {0,1,0}, -5});
    
    Body b(0);
    b.props.radius = 1.0;
    b.state.position = dvec3(0, -6, 0);
    b.state.velocity = dvec3(0, -2, 0);
    
    std::vector<Body> bodies = {b};
    SimulationParams params;
    std::vector<CollisionEvent> events;
    
    cs.detect_collisions(bodies, params, 0.016, events);
    TEST("Static plane: collision detected", !events.empty());
    
    if (!events.empty()) {
        TEST("Static plane: normal points upward", events[0].normal.y > 0);
    }
}

void test_spatial_hash() {
    SpatialHash hash(2.0);
    
    std::vector<Body> bodies;
    for (int i = 0; i < 10; i++) {
        Body b(i);
        b.state.position = dvec3(i * 3.0, 0, 0);
        b.props.radius = 0.5;
        bodies.push_back(b);
    }
    
    hash.build(bodies);
    std::vector<CollisionPair> pairs;
    hash.get_potential_pairs(pairs);
    
    // Bodies are 3 units apart with radius 0.5, should not collide
    // but spatial hash might produce them due to cell boundaries
    TEST("Spatial hash: produces pairs", !pairs.empty());
}

// ==================== CONSTRAINT TESTS ====================
void test_distance_constraint() {
    ConstraintSolver solver;
    
    Body a(0), b(1);
    a.props.mass = 1.0;
    b.props.mass = 1.0;
    a.state.position = dvec3(0, 0, 0);
    b.state.position = dvec3(3, 0, 0);
    a.is_static(); // not static
    
    ConstraintParams c;
    c.type = ConstraintType::DISTANCE;
    c.body_a = 0;
    c.body_b = 1;
    c.distance.rest_length = 2.0;
    c.distance.stiffness = 0.8;
    
    solver.add_constraint(c);
    
    std::vector<Body> bodies = {a, b};
    solver.solve(bodies, 0.016, 5);
    
    double dist = glm::length(bodies[1].state.position - bodies[0].state.position);
    TEST_NEAR("Distance constraint: converges to rest length", dist, 2.0, 0.5);
}

void test_spring_constraint() {
    ConstraintSolver solver;
    
    Body a(0), b(1);
    a.props.mass = 1.0;
    b.props.mass = 1.0;
    a.state.position = dvec3(0, 0, 0);
    b.state.position = dvec3(2, 0, 0);
    
    ConstraintParams c;
    c.type = ConstraintType::SPRING;
    c.body_a = 0;
    c.body_b = 1;
    c.spring.rest_length = 1.0;
    c.spring.stiffness = 10.0;
    
    solver.add_constraint(c);
    
    std::vector<Body> bodies = {a, b};
    double initial_dist = glm::length(bodies[1].state.position - bodies[0].state.position);
    
    // Apply spring over many steps
    for (int i = 0; i < 100; i++) {
        solver.solve_constraint(bodies, 0, 0.016);
    }
    
    double final_dist = glm::length(bodies[1].state.position - bodies[0].state.position);
    TEST("Spring constraint: bodies move toward rest length", 
         std::abs(final_dist - 1.0) < std::abs(initial_dist - 1.0));
}

// ==================== INTEGRATOR TESTS ====================
void test_integrator_energy_conservation() {
    // Test that symplectic integrators conserve energy better than explicit
    
    Solver solver;
    
    // Simple harmonic oscillator
    auto derivs = [](const StateVector& s, StateVector& d, double t) {
        // dx/dt = v, dv/dt = -x (unit mass, unit stiffness)
        d[0] = s[1];   // dx = v
        d[1] = -s[0];  // dv = -x
    };
    
    // Run with different integrators and check energy drift
    struct TestCase {
        IntegratorType type;
        const char* name;
    };
    
    TestCase cases[] = {
        {IntegratorType::Euler, "Явный Эйлер"},
        {IntegratorType::SymplecticEuler, "Симплектический Эйлер"},
        {IntegratorType::Verlet, "Verlet"},
        {IntegratorType::RK4, "RK4"},
    };
    
    for (auto& tc : cases) {
        solver.set_integrator(tc.type);
        
        StateVector state = {1.0, 0.0}; // x=1, v=0
        double t = 0;
        
        // Initial energy = 0.5 * v^2 + 0.5 * x^2 = 0.5
        double e0 = 0.5 * state[0] * state[0] + 0.5 * state[1] * state[1];
        double max_drift = 0;
        
        // Simulate 1000 steps
        for (int i = 0; i < 1000; i++) {
            solver.step(state, derivs, t, 0.01);
            double e = 0.5 * state[0] * state[0] + 0.5 * state[1] * state[1];
            double drift = std::abs(e - e0) / e0;
            max_drift = std::max(max_drift, drift);
        }
        
        std::cout << "  Интегратор " << tc.name << ": макс. дрифт = " 
                  << max_drift << std::endl;
        
        // Symplectic methods should have bounded drift
        if (tc.type == IntegratorType::SymplecticEuler || 
            tc.type == IntegratorType::Verlet) {
            TEST((std::string("Symplectic: ") + tc.name + " дрифт мал").c_str(), 
                 max_drift < 0.1);
        }
    }
}

void test_rk4_accuracy() {
    Solver solver;
    solver.set_integrator(IntegratorType::RK4);
    
    // Solve dy/dx = y (solution: y = exp(x))
    auto derivs = [](const StateVector& s, StateVector& d, double t) {
        d[0] = s[0];
    };
    
    StateVector state = {1.0};
    double t = 0;
    
    for (int i = 0; i < 100; i++) {
        solver.step(state, derivs, t, 0.01);
    }
    
    double expected = std::exp(1.0);
    TEST_NEAR("RK4: exponential growth accurate", state[0], expected, 0.01);
}

// ==================== ROTATION TESTS ====================
void test_inertia_tensor() {
    auto I = InertiaTensor::sphere(1.0, 1.0);
    TEST_NEAR("Sphere inertia Ixx = 0.4 MR²", I.Ixx, 0.4, 1e-10);
    TEST_NEAR("Sphere inertia Iyy = 0.4 MR²", I.Iyy, 0.4, 1e-10);
    TEST_NEAR("Sphere inertia Izz = 0.4 MR²", I.Izz, 0.4, 1e-10);
    
    auto Ibox = InertiaTensor::box(1.0, 2.0, 2.0, 2.0);
    TEST_NEAR("Box Ixx correct", Ibox.Ixx, 2.0/3.0, 1e-10);
    
    auto inv = I.inverse();
    dvec3 v(1, 0, 0);
    dvec3 applied = I.apply(v);
    dvec3 restored = inv.apply(applied);
    TEST_NEAR("Inverse inertia: identity test", restored.x, 1.0, 1e-10);
}

void test_quaternion_normalization() {
    dquat q(1.0, 0.5, 0.3, 0.1);
    RotationalDynamics::normalize_quat(q);
    double len = q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z;
    TEST_NEAR("Quaternion normalization preserves unit length", len, 1.0, 1e-10);
}

void test_rotation_impulse() {
    Body b(0);
    b.props.mass = 1.0;
    b.state.position = dvec3(0, 0, 0);
    
    // Apply impulse at a point to produce rotation
    dvec3 impulse(0, 0, 10);
    dvec3 point(1, 0, 0);
    
    RotationalDynamics::apply_impulse_at_point(b, impulse, point);
    
    // Linear velocity
    TEST_NEAR("Impulse: linear velocity", b.state.velocity.z, 10.0, 1e-10);
    
    // Angular velocity from r × p = (1,0,0) × (0,0,10) = (0, -10, 0)
    TEST("Impulse: angular velocity produced", 
         std::abs(b.state.angular_velocity.y) > 0);
}

// ==================== BOUNDARY TESTS ====================
void test_reflective_boundary() {
    BoundarySystem bs;
    
    PlaneBoundary pb;
    pb.normal = dvec3(0, 1, 0);
    pb.offset = 0;
    pb.restitution = 1.0;
    bs.add_plane_boundary(pb);
    
    Body b(0);
    b.props.radius = 1.0;
    b.state.position = dvec3(0, -0.5, 0);  // Crossing boundary
    b.state.velocity = dvec3(0, -5, 0);
    
    std::vector<Body> bodies = {b};
    bs.apply(bodies, 0.016);
    
    TEST("Reflective: position corrected above boundary", 
         bodies[0].state.position.y > -1.0);
    TEST("Reflective: velocity reversed", 
         bodies[0].state.velocity.y > 0);
}

void test_periodic_boundary() {
    BoundarySystem bs;
    bs.set_periodic_box(dvec3(10, 10, 10));
    
    dvec3 pos(6, 0, 0);  // Outside box
    dvec3 wrapped = bs.apply_periodic(pos);
    TEST("Periodic: wraps correctly", std::abs(wrapped.x) <= 5.0);
    
    dvec3 pos2(-6, 0, 0);
    dvec3 wrapped2 = bs.apply_periodic(pos2);
    TEST("Periodic: wraps negative correctly", std::abs(wrapped2.x) <= 5.0);
}

void test_sdf_boundary() {
    auto sdf = std::make_unique<SDF>();
    sdf->add_sphere(dvec3(0, 0, 0), 5.0);
    
    BoundarySystem bs;
    bs.set_sdf(std::move(sdf));
    
    Body b(0);
    b.props.radius = 1.0;
    b.state.position = dvec3(8, 0, 0);  // Outside sphere
    b.state.velocity = dvec3(-1, 0, 0);
    
    // SDF distance at (8,0,0) with radius 5 should be 3
    TEST("SDF: outside returns positive distance", 
         bs.sdf()->distance(dvec3(8, 0, 0)) > 0);
    
    // Inside sphere
    TEST("SDF: inside returns negative distance",
         bs.sdf()->distance(dvec3(2, 0, 0)) < 0);
}

// ==================== ADAPTIVE TIMESTEP TESTS ====================
void test_adaptive_timestep_config() {
    AdaptiveTimestep at;
    
    TEST("Default min_dt", at.config().min_dt == 1e-6);
    TEST("Default target_dt", std::abs(at.config().target_dt - 1.0/60.0) < 1e-10);
    
    at.config().error_tolerance = 1e-3;
    TEST("Config change works", at.config().error_tolerance == 1e-3);
}

// ==================== MAIN ====================
int main() {
    std::cout << "\n=== HoloPhysics Lab 2 — Physics Test Suite ===\n" << std::endl;
    
    // Collision tests
    std::cout << "\n--- Collision System ---\n" << std::endl;
    test_collision_sphere_sphere();
    test_collision_static_boundary();
    test_spatial_hash();
    
    // Constraint tests
    std::cout << "\n--- Constraint Solver ---\n" << std::endl;
    test_distance_constraint();
    test_spring_constraint();
    
    // Integrator tests
    std::cout << "\n--- Numerical Integrators ---\n" << std::endl;
    test_integrator_energy_conservation();
    test_rk4_accuracy();
    
    // Rotation tests
    std::cout << "\n--- Rotational Dynamics ---\n" << std::endl;
    test_inertia_tensor();
    test_quaternion_normalization();
    test_rotation_impulse();
    
    // Boundary tests
    std::cout << "\n--- Boundary Conditions ---\n" << std::endl;
    test_reflective_boundary();
    test_periodic_boundary();
    test_sdf_boundary();
    
    // Adaptive timestep tests
    std::cout << "\n--- Adaptive Timestep ---\n" << std::endl;
    test_adaptive_timestep_config();
    
    // Summary
    std::cout << "\n=== Results ===" << std::endl;
    std::cout << "✅ Passed: " << tests_passed << std::endl;
    std::cout << "❌ Failed: " << tests_failed << std::endl;
    std::cout << "📊 Total:  " << (tests_passed + tests_failed) << std::endl;
    
    return tests_failed > 0 ? 1 : 0;
}
