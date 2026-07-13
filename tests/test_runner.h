#pragma once

// ============================================================
// HoloPhysics Lab 2 — Minimal Test Framework
// Используется, когда Catch2 недоступен
// ============================================================

#include <iostream>
#include <string>
#include <functional>
#include <vector>

struct TestCase {
    std::string name;
    std::function<bool()> test;
    std::string file;
    int line;
};

class TestRegistry {
public:
    static TestRegistry& instance() {
        static TestRegistry reg;
        return reg;
    }
    
    void add(const std::string& name, std::function<bool()> test,
             const std::string& file, int line) {
        tests_.push_back({name, test, file, line});
    }
    
    int run_all() {
        int passed = 0, failed = 0;
        
        std::cout << "\n=== HoloPhysics Lab 2 — Test Suite ===\n" << std::endl;
        
        for (auto& t : tests_) {
            try {
                if (t.test()) {
                    std::cout << "  ✅ " << t.name << std::endl;
                    passed++;
                } else {
                    std::cout << "  ❌ " << t.name << " (" << t.file << ":" << t.line << ")" << std::endl;
                    failed++;
                }
            } catch (const std::exception& e) {
                std::cout << "  💥 " << t.name << " — exception: " << e.what() << std::endl;
                failed++;
            } catch (...) {
                std::cout << "  💥 " << t.name << " — unknown exception" << std::endl;
                failed++;
            }
        }
        
        std::cout << "\n=== Results: " << passed << " passed, " 
                  << failed << " failed ===" << std::endl;
        return failed;
    }
    
private:
    std::vector<TestCase> tests_;
};

#define TEST_CASE(name) \
    bool test_func_##__LINE__(); \
    bool registered_##__LINE__ = []() { \
        TestRegistry::instance().add(name, test_func_##__LINE__, __FILE__, __LINE__); \
        return true; \
    }(); \
    bool test_func_##__LINE__()

#define CHECK(expr) do { \
    if (!(expr)) { \
        std::cerr << "    FAIL at " << __FILE__ << ":" << __LINE__ << ": " << #expr << std::endl; \
        return false; \
    } \
} while(0)

#define CHECK_NEAR(a, b, eps) do { \
    double va = (a), vb = (b); \
    if (std::abs(va - vb) > eps) { \
        std::cerr << "    FAIL at " << __FILE__ << ":" << __LINE__ << ": " \
                  << #a << " = " << va << " != " << #b << " = " << vb \
                  << " (eps=" << eps << ")" << std::endl; \
        return false; \
    } \
} while(0)
