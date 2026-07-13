#pragma once

// ============================================================
// HoloPhysics Lab 2 — Tracy Profiler Integration
//
// Лёгкое профилирование с макросами ZoneScoped.
// Включается/отключается флагом ENABLE_TRACY.
//
// Использование:
//   void my_function() {
//       ZoneScoped;  // Автоматический замер
//       // ... код ...
//   }
// ============================================================

#include <cstdint>

// ==================== Configuration ====================
// Включить: cmake -DENABLE_TRACY=ON
// Выключено по умолчанию (zero overhead)

#ifdef ENABLE_TRACY
    #include <tracy/Tracy.hpp>
    #define PROFILE_FRAME() FrameMark
    #define PROFILE_ZONE(name) ZoneScopedN(name)
    #define PROFILE_ZONE_COLOR(name, color) ZoneScopedNC(name, color)
    #define PROFILE_LOCK(type, ptr) LockMark(type, ptr)
    #define PROFILE_ALLOC(ptr, size) TracyAlloc(ptr, size)
    #define PROFILE_FREE(ptr) TracyFree(ptr)
    #define PROFILE_PLOT(name, val) TracyPlot(name, val)
    #define PROFILE_MESSAGE(msg) TracyMessage(msg, strlen(msg))
#else
    // No-op stubs (zero overhead when disabled)
    #define PROFILE_FRAME()
    #define PROFILE_ZONE(name)
    #define PROFILE_ZONE_COLOR(name, color)
    #define PROFILE_LOCK(type, ptr)
    #define PROFILE_ALLOC(ptr, size)
    #define PROFILE_FREE(ptr)
    #define PROFILE_PLOT(name, val)
    #define PROFILE_MESSAGE(msg)
#endif

// ==================== Custom Zone Colors ====================
// Для визуальной группировки в Tracy

namespace hlp {
namespace profiler {

// Color constants (ABGR format)
inline constexpr uint32_t COLOR_PHYSICS     = 0xFF4488FF;  // Blue
inline constexpr uint32_t COLOR_RENDER      = 0xFF00FF88;  // Green
inline constexpr uint32_t COLOR_CV          = 0xFFFF8844;  // Orange
inline constexpr uint32_t COLOR_INPUT       = 0xFFFF44FF;  // Pink
inline constexpr uint32_t COLOR_INTERACTION = 0xFFFF4488;  // Red
inline constexpr uint32_t COLOR_UI          = 0xFF88FF44;  // Lime
inline constexpr uint32_t COLOR_AUDIO       = 0xFFFFCC44;  // Yellow
inline constexpr uint32_t COLOR_SCRIPTING   = 0xFF44FFCC;  // Cyan
inline constexpr uint32_t COLOR_XR          = 0xFF8844FF;  // Purple

// Named zones for common operations
#define ZONE_PHYSICS_STEP()     PROFILE_ZONE_COLOR("Physics::Step", hlp::profiler::COLOR_PHYSICS)
#define ZONE_COLLISION()        PROFILE_ZONE_COLOR("Collision::Detect", hlp::profiler::COLOR_PHYSICS)
#define ZONE_GJK()              PROFILE_ZONE_COLOR("GJK::Intersect", hlp::profiler::COLOR_PHYSICS)
#define ZONE_CCD()              PROFILE_ZONE_COLOR("CCD::Sweep", hlp::profiler::COLOR_PHYSICS)
#define ZONE_PBD()              PROFILE_ZONE_COLOR("PBD::Solve", hlp::profiler::COLOR_PHYSICS)
#define ZONE_VIRTUAL_COUPLING() PROFILE_ZONE_COLOR("VirtualCoupling::Update", hlp::profiler::COLOR_INTERACTION)

#define ZONE_RENDER()           PROFILE_ZONE_COLOR("Render::Frame", hlp::profiler::COLOR_RENDER)
#define ZONE_METABALL()         PROFILE_ZONE_COLOR("Metaball::Occlusion", hlp::profiler::COLOR_RENDER)
#define ZONE_SHADOW()           PROFILE_ZONE_COLOR("Shadow::Render", hlp::profiler::COLOR_RENDER)

#define ZONE_CAMERA_CAPTURE()   PROFILE_ZONE_COLOR("Camera::Capture", hlp::profiler::COLOR_CV)
#define ZONE_HAND_TRACK()       PROFILE_ZONE_COLOR("Hand::Track", hlp::profiler::COLOR_CV)

#define ZONE_GESTURE()          PROFILE_ZONE_COLOR("Gesture::Classify", hlp::profiler::COLOR_INPUT)
#define ZONE_KALMAN()           PROFILE_ZONE_COLOR("Kalman::Update", hlp::profiler::COLOR_INPUT)

#define ZONE_UI()               PROFILE_ZONE_COLOR("UI::Render", hlp::profiler::COLOR_UI)
#define ZONE_LUA_CALL()         PROFILE_ZONE_COLOR("Lua::Callback", hlp::profiler::COLOR_SCRIPTING)

// Plot values for Tracy graphs
#define PLOT_FPS(val)           PROFILE_PLOT("FPS", val)
#define PLOT_PHYSICS_MS(val)    PROFILE_PLOT("Physics (ms)", val)
#define PLOT_RENDER_MS(val)     PROFILE_PLOT("Render (ms)", val)
#define PLOT_BODY_COUNT(val)    PROFILE_PLOT("Body Count", val)
#define PLOT_ENERGY(val)        PROFILE_PLOT("Total Energy", val)
#define PLOT_ENERGY_DRIFT(val)  PROFILE_PLOT("Energy Drift", val)
#define PLOT_CAMERA_FPS(val)    PROFILE_PLOT("Camera FPS", val)

} // namespace profiler
} // namespace hlp
