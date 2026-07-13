// ============================================================
// HoloPhysics Lab 2 — Collision Detection Stress Tests
// ============================================================

#include "../physics/CollisionSystem.h"
#include <iostream>
#include <chrono>

using namespace hlp;

int main() {
    std::cout << "=== Collision Performance Test ===\n" << std::endl;
    
    CollisionSystem cs;
    cs.set_broad_phase(CollisionSystem::SPATIAL_HASH);
    
    // Create N bodies in random positions
    const int N = 100;
    std::vector<Body> bodies;
    
    srand(42);
    for (int i = 0; i < N; i++) {
        Body b(i);
        b.state.position = dvec3(
            (rand() % 2000 - 1000) * 0.01,
            (rand() % 2000 - 1000) * 0.01,
            (rand() % 2000 - 1000) * 0.01
        );
        b.props.radius = 0.1 + (rand() % 100) * 0.01;
        bodies.push_back(b);
    }
    
    SimulationParams params;
    std::vector<CollisionEvent> events;
    
    auto start = std::chrono::high_resolution_clock::now();
    
    for (int iter = 0; iter < 100; iter++) {
        events.clear();
        cs.detect_collisions(bodies, params, 1.0/60.0, events);
        
        // Move bodies slightly
        for (auto& b : bodies) {
            b.state.position += dvec3((rand() % 200 - 100) * 0.0005, 0, 0);
        }
    }
    
    auto end = std::chrono::high_resolution_clock::now();
    double ms = std::chrono::duration<double, std::milli>(end - start).count();
    
    std::cout << "Тело: " << N << std::endl;
    std::cout << "Коллизий: " << events.size() << std::endl;
    std::cout << "Время: " << ms / 100.0 << " мс/шаг" << std::endl;
    std::cout << "FPS: " << 1000.0 / (ms / 100.0) << std::endl;
    
    return 0;
}
