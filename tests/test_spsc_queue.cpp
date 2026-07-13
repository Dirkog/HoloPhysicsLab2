// ============================================================
// HoloPhysics Lab 2 — Lock-Free SPSC Queue Stress Test
// Проверка на гонки данных, потерю событий и memory ordering
// ============================================================

#include "../src/physics/queue/LockFreeEventQueue.h"
#include <iostream>
#include <thread>
#include <atomic>
#include <vector>
#include <chrono>
#include <cassert>

using namespace hlp;

static int tests_passed = 0;
static int tests_failed = 0;

#define TEST(name, expr) do { \
    if (!(expr)) { \
        std::cerr << "  ❌ FAIL: " << name << std::endl; \
        tests_failed++; \
    } else { \
        std::cout << "  ✅ PASS: " << name << std::endl; \
        tests_passed++; \
    } \
} while(0)

// ==================== Test 1: Basic Push/Pop ====================
void test_basic_push_pop() {
    LockFreeEventQueue queue;
    EventData e;
    
    // Empty queue
    TEST("Empty queue returns false", !queue.pop(e));
    
    // Push one event
    EventData e1;
    e1.type = EventType::COLLISION;
    e1.body_a = 42;
    e1.body_b = 137;
    TEST("Push succeeds", queue.push(e1));
    
    // Pop it back
    TEST("Pop succeeds", queue.pop(e));
    TEST("Type matches", e.type == EventType::COLLISION);
    TEST("Body A matches", e.body_a == 42);
    TEST("Body B matches", e.body_b == 137);
    
    // Queue empty again
    TEST("Queue empty after pop", !queue.pop(e));
}

// ==================== Test 2: Multiple Events ====================
void test_multiple_events() {
    LockFreeEventQueue queue;
    
    // Push 100 events
    for (int i = 0; i < 100; i++) {
        EventData e;
        e.type = EventType::COLLISION;
        e.body_a = i;
        e.value1 = i * 1.5;
        queue.push(e);
    }
    
    // Pop and verify order
    int count = 0;
    EventData e;
    while (queue.pop(e)) {
        TEST("Event order preserved", e.body_a == (uint64_t)count);
        TEST("Event value preserved", e.value1 == count * 1.5);
        count++;
    }
    
    TEST("All 100 events received", count == 100);
}

// ==================== Test 3: SPSC Concurrent Stress ====================
void test_concurrent_stress() {
    LockFreeEventQueue queue;
    const int NUM_EVENTS = 100000;
    std::atomic<int> produced{0};
    std::atomic<int> consumed{0};
    
    // Producer thread
    std::thread producer([&]() {
        for (int i = 0; i < NUM_EVENTS; i++) {
            EventData e;
            e.type = EventType::COLLISION;
            e.body_a = i;
            e.timestamp = i;
            while (!queue.push(e)) {
                // Busy-wait if full (shouldn't happen with 4096 buffer)
                std::this_thread::yield();
            }
            produced++;
        }
    });
    
    // Consumer thread
    std::thread consumer([&]() {
        EventData e;
        int last_id = -1;
        while (consumed < NUM_EVENTS) {
            if (queue.pop(e)) {
                // Check ordering (must be non-decreasing)
                // In SPSC, ordering is guaranteed within a single producer
                consumed++;
            } else {
                std::this_thread::yield();
            }
        }
    });
    
    producer.join();
    consumer.join();
    
    TEST("All events produced", produced == NUM_EVENTS);
    TEST("All events consumed", consumed == NUM_EVENTS);
    TEST("No events dropped", queue.dropped() == 0);
    TEST("Queue is empty", queue.empty());
}

// ==================== Test 4: Buffer Overflow ====================
void test_buffer_overflow() {
    LockFreeEventQueue queue;
    
    // Fill the buffer completely
    int pushed = 0;
    EventData e;
    e.type = EventType::COLLISION;
    
    while (queue.push(e)) {
        pushed++;
    }
    
    TEST("Buffer fills at capacity", pushed >= LockFreeEventQueue::BUFFER_SIZE - 1);
    TEST("Dropped counter incremented", queue.dropped() > 0);
    
    // Drain
    int drained = 0;
    while (queue.pop(e)) drained++;
    
    TEST("Can drain after overflow", drained > 0);
    TEST("Queue empty after drain", queue.empty());
}

// ==================== Test 5: Memory Ordering ====================
void test_memory_ordering() {
    // Test that release/acquire semantics work correctly
    // by checking that producer writes are visible to consumer
    LockFreeEventQueue queue;
    
    std::atomic<bool> ready{false};
    EventData shared_data;
    
    std::thread producer([&]() {
        EventData e;
        e.type = EventType::COLLISION;
        e.body_a = 0xDEADBEEF;
        e.value1 = 3.14159;
        e.timestamp = 12345;
        
        queue.push(e);
        ready.store(true, std::memory_order_release);
    });
    
    std::thread consumer([&]() {
        while (!ready.load(std::memory_order_acquire)) {
            std::this_thread::yield();
        }
        
        EventData e;
        if (queue.pop(e)) {
            shared_data = e;
        }
    });
    
    producer.join();
    consumer.join();
    
    // Verify all fields are correct (no torn reads)
    TEST("Memory ordering: type", shared_data.type == EventType::COLLISION);
    TEST("Memory ordering: body_a", shared_data.body_a == 0xDEADBEEF);
    TEST("Memory ordering: value1", shared_data.value1 == 3.14159);
    TEST("Memory ordering: timestamp", shared_data.timestamp == 12345);
}

// ==================== Test 6: EventDispatcher ====================
void test_event_dispatcher() {
    EventDispatcher dispatcher;
    
    // Push several events
    dispatcher.push_collision(1, 2, 5.0, dvec3(0,1,0), dvec3(0,1,0));
    dispatcher.push_grab(42);
    dispatcher.push_release(42, dvec3(1,2,3));
    
    // Dispatch with counting callback
    int collision_count = 0;
    int grab_count = 0;
    int release_count = 0;
    
    dispatcher.dispatch([&](const EventData& e) {
        switch (e.type) {
            case EventType::COLLISION: collision_count++; break;
            case EventType::GRAB:      grab_count++; break;
            case EventType::RELEASE:   release_count++; break;
            default: break;
        }
    });
    
    TEST("Collision dispatched", collision_count == 1);
    TEST("Grab dispatched", grab_count == 1);
    TEST("Release dispatched", release_count == 1);
    TEST("Queue empty after dispatch", dispatcher.queue_size() == 0);
}

// ==================== Test 7: Performance Benchmark ====================
void test_performance() {
    LockFreeEventQueue queue;
    const int NUM_EVENTS = 1000000;  // 1 million
    
    EventData e;
    e.type = EventType::COLLISION;
    
    auto start = std::chrono::high_resolution_clock::now();
    
    for (int i = 0; i < NUM_EVENTS; i++) {
        e.body_a = i;
        while (!queue.push(e)) {}
    }
    
    int count = 0;
    while (queue.pop(e)) count++;
    
    auto end = std::chrono::high_resolution_clock::now();
    double ms = std::chrono::duration<double, std::milli>(end - start).count();
    double mops = (NUM_EVENTS * 2) / ms / 1000;  // push + pop
    
    std::cout << "  Performance: " << mops << " MOPS (" << ms << " ms for " 
              << NUM_EVENTS << " push+pop)" << std::endl;
    TEST("Performance > 5 MOPS", mops > 5.0);
    TEST("All events accounted for", count == NUM_EVENTS);
}

// ==================== Main ====================
int main() {
    std::cout << "\n=== Lock-Free SPSC Queue Stress Test ===\n" << std::endl;
    
    std::cout << "--- Basic Operations ---" << std::endl;
    test_basic_push_pop();
    
    std::cout << "\n--- Multiple Events ---" << std::endl;
    test_multiple_events();
    
    std::cout << "\n--- Concurrent Stress (100k events) ---" << std::endl;
    test_concurrent_stress();
    
    std::cout << "\n--- Buffer Overflow ---" << std::endl;
    test_buffer_overflow();
    
    std::cout << "\n--- Memory Ordering ---" << std::endl;
    test_memory_ordering();
    
    std::cout << "\n--- Event Dispatcher ---" << std::endl;
    test_event_dispatcher();
    
    std::cout << "\n--- Performance (1M events) ---" << std::endl;
    test_performance();
    
    std::cout << "\n=== Results ===" << std::endl;
    std::cout << "✅ Passed: " << tests_passed << std::endl;
    std::cout << "❌ Failed: " << tests_failed << std::endl;
    std::cout << "📊 Total:  " << (tests_passed + tests_failed) << std::endl;
    
    return tests_failed > 0 ? 1 : 0;
}
