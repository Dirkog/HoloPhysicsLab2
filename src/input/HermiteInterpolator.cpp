#include "HermiteInterpolator.h"
#include <cstring>
#include <iostream>

namespace hlp {

// ==================== HandRingBuffer ====================

HandRingBuffer::HandRingBuffer() {
    // Initialize all slots as invalid
    for (int i = 0; i < NUM_SLOTS; i++) {
        slots_[i].valid = false;
    }
}

void HandRingBuffer::produce(const HandSnapshot& snapshot) {
    // === Producer: CV Thread ===
    // Lock-free write: find next slot, write, then publish
    
    int current_write = write_idx_.load(std::memory_order_relaxed);
    int next_slot = (current_write + 1) % NUM_SLOTS;
    
    // Write to the slot (producer owns it exclusively)
    slots_[next_slot] = snapshot;
    
    // Ensure all writes are visible before publishing
    std::atomic_thread_fence(std::memory_order_release);
    
    // Publish: update available index
    available_idx_.store(next_slot, std::memory_order_release);
    
    // Update write index
    write_idx_.store(next_slot, std::memory_order_relaxed);
    write_count_.fetch_add(1, std::memory_order_relaxed);
}

HandRingBuffer::SnapshotPair HandRingBuffer::consume() {
    // === Consumer: Physics/Render Thread ===
    SnapshotPair result;
    
    int avail = available_idx_.load(std::memory_order_acquire);
    if (avail < 0) return result;  // No data yet
    
    // Read current snapshot
    HandSnapshot current;
    std::memcpy(&current, &slots_[avail], sizeof(HandSnapshot));
    
    // Ensure we read after the acquire fence
    std::atomic_thread_fence(std::memory_order_acquire);
    
    result.current = current;
    result.has_two = has_last_;
    
    if (has_last_) {
        result.prev = last_consumed_;
        result.dt = (current.timestamp_us - last_consumed_.timestamp_us) / 1e6;
        if (result.dt < 1e-6) result.dt = 1.0 / 60.0;  // Safety default
    }
    
    last_consumed_ = current;
    has_last_ = true;
    
    read_idx_.store(avail, std::memory_order_relaxed);
    read_count_.fetch_add(1, std::memory_order_relaxed);
    
    return result;
}

// ==================== HermiteInterpolator ====================

HermiteInterpolator::InterpolatedState HermiteInterpolator::sample(double t_local) {
    InterpolatedState result;
    
    if (!buffer_) {
        result.valid = false;
        return result;
    }
    
    auto pair = buffer_->consume();
    if (!pair.has_two) {
        // Only one snapshot: use directly (no interpolation possible)
        result.position = pair.current.position();
        result.velocity = pair.current.velocity();
        result.valid = pair.current.valid;
        return result;
    }
    
    // Clamp t to [0, 1]
    double t = std::max(0.0, std::min(1.0, t_local / pair.dt));
    
    // Get positions and velocities
    dvec3 p0 = pair.prev.position();
    dvec3 p1 = pair.current.position();
    dvec3 m0 = pair.prev.velocity();    // Tangent at start = velocity at prev
    dvec3 m1 = pair.current.velocity(); // Tangent at end = velocity at current
    
    // Hermite interpolation
    result.position = hermite_vec(p0, m0, p1, m1, t);
    result.velocity = hermite_vel(p0, m0, p1, m1, t, pair.dt);
    result.valid = pair.current.valid && pair.prev.valid;
    
    return result;
}

dvec3 HermiteInterpolator::latest_position() const {
    if (!buffer_) return dvec3(0);
    
    auto pair = buffer_->consume();
    if (pair.has_two) {
        return pair.current.position();
    }
    return pair.current.position();
}

} // namespace hlp
