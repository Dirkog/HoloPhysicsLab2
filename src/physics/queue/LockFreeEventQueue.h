#pragma once

// ============================================================
// HoloPhysics Lab 2 — Lock-Free Event Queue
//
// Решает: Lua-дедлок при многопоточности
//   Физический поток НЕ вызывает Lua напрямую.
//   Он кладёт событие в lock-free очередь.
//   Главный поток (Render) разгребает очередь и вызывает Lua.
//
// Алгоритм: Single-Producer Single-Consumer (SPSC) Ring Buffer
//   Размер: фиксированный (4096), без аллокаций
// ============================================================

#include "../../core/Types.h"
#include <atomic>
#include <array>
#include <functional>

namespace hlp {

// ==================== Event Types ====================

enum class EventType : uint8_t {
    COLLISION,
    GRAB,
    RELEASE,
    BODY_ADDED,
    BODY_REMOVED,
    CUSTOM
};

struct EventData {
    EventType type;
    uint64_t body_a = 0;
    uint64_t body_b = 0;
    double value1 = 0;  // impulse, force, etc.
    double value2 = 0;
    double value3 = 0;
    dvec3 point{0};
    dvec3 normal{0};
    uint64_t timestamp = 0;
    
    // Custom payload (max 64 bytes)
    uint8_t custom_data[48] = {0};
};

// ==================== SPSC Ring Buffer (Lock-Free) ====================

class LockFreeEventQueue {
public:
    static constexpr int BUFFER_SIZE = 4096;
    
    LockFreeEventQueue() = default;
    
    // === Producer (Physics Thread) — NEVER blocks ===
    bool push(const EventData& event) {
        int head = head_.load(std::memory_order_relaxed);
        int next = (head + 1) % BUFFER_SIZE;
        
        if (next == tail_.load(std::memory_order_acquire)) {
            // Buffer full — overwrite oldest
            dropped_++;
            return false;
        }
        
        buffer_[head] = event;
        head_.store(next, std::memory_order_release);
        return true;
    }
    
    // === Consumer (Main/Render Thread) ===
    bool pop(EventData& event) {
        int tail = tail_.load(std::memory_order_relaxed);
        if (tail == head_.load(std::memory_order_acquire)) {
            return false;  // Empty
        }
        
        event = buffer_[tail];
        tail_.store((tail + 1) % BUFFER_SIZE, std::memory_order_release);
        return true;
    }
    
    // Pop all events and call callback for each
    int drain(std::function<void(const EventData&)> callback) {
        int count = 0;
        EventData event;
        while (pop(event)) {
            callback(event);
            count++;
        }
        return count;
    }
    
    // === Stats ===
    int size() const {
        int head = head_.load(std::memory_order_acquire);
        int tail = tail_.load(std::memory_order_acquire);
        return (head - tail + BUFFER_SIZE) % BUFFER_SIZE;
    }
    
    int dropped() const { return dropped_.load(); }
    bool empty() const { return head_.load() == tail_.load(); }
    void reset() { 
        head_.store(0, std::memory_order_release);
        tail_.store(0, std::memory_order_release);
        dropped_.store(0);
    }

private:
    // Cache-line padded to prevent false sharing
    alignas(64) std::array<EventData, BUFFER_SIZE> buffer_;
    alignas(64) std::atomic<int> head_{0};  // Producer writes here
    alignas(64) std::atomic<int> tail_{0};  // Consumer reads here
    alignas(64) std::atomic<int> dropped_{0};
};

// ==================== Event Dispatcher ====================

class EventDispatcher {
public:
    EventDispatcher() = default;
    
    // === Producer API (physics thread) ===
    LockFreeEventQueue& queue() { return queue_; }
    
    // Convenience: push collision event
    void push_collision(uint64_t a, uint64_t b, double impulse,
                        const dvec3& point, const dvec3& normal) {
        EventData e;
        e.type = EventType::COLLISION;
        e.body_a = a;
        e.body_b = b;
        e.value1 = impulse;
        e.point = point;
        e.normal = normal;
        e.timestamp = get_timestamp();
        queue_.push(e);
    }
    
    void push_grab(uint64_t body_id) {
        EventData e;
        e.type = EventType::GRAB;
        e.body_a = body_id;
        e.timestamp = get_timestamp();
        queue_.push(e);
    }
    
    void push_release(uint64_t body_id, const dvec3& velocity) {
        EventData e;
        e.type = EventType::RELEASE;
        e.body_a = body_id;
        e.value1 = velocity.x;
        e.value2 = velocity.y;
        e.value3 = velocity.z;
        e.timestamp = get_timestamp();
        queue_.push(e);
    }
    
    // === Consumer API (main thread) ===
    int dispatch(std::function<void(const EventData&)> callback) {
        return queue_.drain(callback);
    }
    
    // === Stats ===
    int queue_size() const { return queue_.size(); }
    int dropped_events() const { return queue_.dropped(); }

private:
    LockFreeEventQueue queue_;
    
    static uint64_t get_timestamp() {
        return std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count();
    }
};

} // namespace hlp
