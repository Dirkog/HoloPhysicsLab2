// ============================================================
// HoloPhysics Lab 2 — Integrator Benchmark & Accuracy Suite
// ============================================================

#include "../core/Solver.h"
#include "../core/Types.h"
#include <iostream>
#include <cmath>
#include <vector>
#include <string>

using namespace hlp;

struct Result {
    const char* name;
    double final_energy;
    double max_drift;
    double final_pos;
    double runtime_ms;
};

int main() {
    std::cout << "=== Integrator Benchmark Suite ===\n" << std::endl;
    
    // === Test 1: Harmonic Oscillator (energy conservation) ===
    std::cout << "--- Test 1: Гармонический осциллятор ---" << std::endl;
    
    auto deriv_harmonic = [](const StateVector& s, StateVector& d, double t) {
        d[0] = s[1];  // dx = v
        d[1] = -s[0]; // dv = -x
    };
    
    IntegratorType types[] = {
        IntegratorType::Euler,
        IntegratorType::SymplecticEuler,
        IntegratorType::Verlet,
        IntegratorType::RK4,
        IntegratorType::RK5,
        IntegratorType::Ruth
    };
    
    std::vector<Result> results;
    
    for (auto type : types) {
        Solver solver;
        solver.set_integrator(type);
        
        StateVector state = {1.0, 0.0};
        double t = 0;
        double e0 = 0.5 * state[0] * state[0] + 0.5 * state[1] * state[1];
        double max_drift = 0;
        
        auto start = std::chrono::high_resolution_clock::now();
        
        for (int i = 0; i < 10000; i++) {
            solver.step(state, deriv_harmonic, t, 0.01);
            double e = 0.5 * state[0] * state[0] + 0.5 * state[1] * state[1];
            double drift = std::abs(e - e0) / e0;
            max_drift = std::max(max_drift, drift);
        }
        
        auto end = std::chrono::high_resolution_clock::now();
        double ms = std::chrono::duration<double, std::milli>(end - start).count();
        
        Result r;
        r.name = Solver::integrator_name(type);
        r.final_energy = 0.5 * state[0] * state[0] + 0.5 * state[1] * state[1];
        r.max_drift = max_drift;
        r.final_pos = state[0];
        r.runtime_ms = ms;
        results.push_back(r);
        
        printf("  %-35s | E=%.6f | drift=%.2e | x=%.4f | %.2f ms\n",
               r.name, r.final_energy, r.max_drift, r.final_pos, r.runtime_ms);
    }
    
    // === Test 2: Kepler orbit (gravitational two-body) ===
    std::cout << "\n--- Test 2: Орбита Кеплера (два тела) ---" << std::endl;
    
    auto deriv_kepler = [](const StateVector& s, StateVector& d, double t) {
        double G = 1.0, M = 100.0;
        double x = s[0], y = s[2]; // positions are at indices 0,2
        // Actually for 2 bodies: [x1,y1,z1, vx1,vy1,vz1, x2,y2,z2, vx2,vy2,vz2]
        // Simplified: test single particle in central field
        double r2 = s[0]*s[0] + s[2]*s[2];  // x^2 + z^2
        double r = std::sqrt(r2);
        if (r < 0.001) r = 0.001;
        double F = -G * M / r2;
        d[0] = s[1];           // dx/dt = vx
        d[1] = F * s[0] / r;   // dvx/dt = Fx/m
        d[2] = s[3];           // dz/dt = vz
        d[3] = F * s[2] / r;   // dvz/dt = Fz/m
    };
    
    for (auto type : {IntegratorType::Verlet, IntegratorType::RK4, IntegratorType::SymplecticEuler}) {
        Solver solver;
        solver.set_integrator(type);
        
        // Circular orbit at r=10, v = sqrt(GM/r) = sqrt(100/10) = sqrt(10) ≈ 3.16
        StateVector state = {10.0, 0.0, 0.0, 3.1623}; // x, vx, z, vz
        double t = 0;
        
        auto start = std::chrono::high_resolution_clock::now();
        
        double max_radius_drift = 0;
        double initial_r = 10.0;
        
        for (int i = 0; i < 10000; i++) {
            solver.step(state, deriv_kepler, t, 0.01);
            double r = std::sqrt(state[0]*state[0] + state[2]*state[2]);
            max_radius_drift = std::max(max_radius_drift, std::abs(r - initial_r));
        }
        
        auto end = std::chrono::high_resolution_clock::now();
        double ms = std::chrono::duration<double, std::milli>(end - start).count();
        
        printf("  %-35s | дрейф радиуса=%.2e | %.2f ms\n",
               Solver::integrator_name(type), max_radius_drift, ms);
    }
    
    // === Test 3: Pendulum (large angle) ===
    std::cout << "\n--- Test 3: Маятник (большой угол) ---" << std::endl;
    
    auto deriv_pendulum = [](const StateVector& s, StateVector& d, double t) {
        double g = 9.81, L = 1.0;
        d[0] = s[1];              // dθ/dt = ω
        d[1] = -(g / L) * std::sin(s[0]);  // dω/dt = -(g/L) sin(θ)
    };
    
    for (auto type : {IntegratorType::RK4, IntegratorType::Verlet}) {
        Solver solver;
        solver.set_integrator(type);
        
        StateVector state = {M_PI * 0.9, 0.0}; // θ=162°, ω=0
        double t = 0;
        
        // Energy: E = 0.5 * m * L^2 * ω^2 + m * g * L * (1 - cos(θ))
        // with m=1, L=1: E = 0.5 * ω^2 + g * (1 - cos(θ))
        double e0 = 9.81 * (1 - std::cos(state[0]));
        double max_drift = 0;
        
        for (int i = 0; i < 10000; i++) {
            solver.step(state, deriv_pendulum, t, 0.005);
            double e = 0.5 * state[1] * state[1] + 9.81 * (1 - std::cos(state[0]));
            max_drift = std::max(max_drift, std::abs(e - e0) / e0);
        }
        
        printf("  %-35s | дрейф энергии=%.2e | θ=%.2f°\n",
               Solver::integrator_name(type), max_drift, state[0] * 180.0 / M_PI);
    }
    
    std::cout << "\n=== Benchmarks Complete ===\n" << std::endl;
    return 0;
}
