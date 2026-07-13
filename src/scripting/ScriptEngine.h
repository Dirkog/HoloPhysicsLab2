#pragma once

// ============================================================
// HoloPhysics Lab 2 — Lua Scripting Engine
//
// Интеграция Lua (через sol2) для:
//   - Загрузки сцен и экспериментов
//   - Скриптов управления физикой
//   - Пользовательских force-функций
//   - Batch-симуляций (parameter sweep)
//   - Callback'ов на события (on_collision, on_grab)
// ============================================================

#include "../core/Types.h"
#include "../core/Engine.h"
#include "../core/Scene.h"
#include "../input/VirtualHand.h"
#include <string>
#include <vector>
#include <functional>
#include <memory>
#include <unordered_map>

// Forward declare Lua state
struct lua_State;

namespace hlp {

// ==================== Lua Script ====================

struct LuaScript {
    std::string name;
    std::string source;
    bool enabled = true;
    bool compiled = false;
    
    // Runtime
    int ref_func = 0;       // Lua registry reference for main function
    std::vector<int> hook_refs;  // Event hooks
};

// ==================== Script Event Types ====================

enum class ScriptEvent : uint8_t {
    ON_START,
    ON_UPDATE,
    ON_FIXED_UPDATE,
    ON_COLLISION,
    ON_GRAB,
    ON_RELEASE,
    ON_KEY_PRESS,
    ON_GESTURE,
    ON_SCENE_LOAD,
    COUNT
};

const char* script_event_name(ScriptEvent e);

// ==================== Script Binding ====================

struct ScriptBinding {
    // Expose physics to Lua
    struct PhysicsAPI {
        // Create/modify bodies
        static int create_sphere(lua_State* L);
        static int create_box(lua_State* L);
        static int create_spring(lua_State* L);
        static int remove_body(lua_State* L);
        static int clear_all(lua_State* L);
        
        // Get/set properties
        static int get_position(lua_State* L);
        static int set_position(lua_State* L);
        static int get_velocity(lua_State* L);
        static int set_velocity(lua_State* L);
        static int apply_force(lua_State* L);
        static int apply_impulse(lua_State* L);
        
        // Query
        static int get_body_count(lua_State* L);
        static int find_nearest(lua_State* L);
        static int raycast(lua_State* L);
        
        // Parameters
        static int get_gravity(lua_State* L);
        static int set_gravity(lua_State* L);
        static int set_damping(lua_State* L);
        
        // Presets
        static int load_preset(lua_State* L);
        static int save_scene(lua_State* L);
        static int load_scene(lua_State* L);
    };
    
    struct RenderAPI {
        static int set_background_color(lua_State* L);
        static int set_fog(lua_State* L);
        static int set_camera_target(lua_State* L);
    };
    
    struct InputAPI {
        static int is_key_pressed(lua_State* L);
        static int get_mouse_position(lua_State* L);
        static int get_hand_position(lua_State* L);
        static int get_hand_gesture(lua_State* L);
    };
    
    struct DebugAPI {
        static int print(lua_State* L);
        static int draw_line(lua_State* L);
        static int draw_text(lua_State* L);
        static int log(lua_State* L);
    };
};

// ==================== Script Engine ====================

class ScriptEngine {
public:
    ScriptEngine();
    ~ScriptEngine();

    // === Lifecycle ===
    bool init(Engine& engine);
    void shutdown();
    bool is_initialized() const { return initialized_; }

    // === Script Management ===
    int load_script(const std::string& name, const std::string& source);
    bool run_script(int script_id);
    bool run_script_file(const std::string& path);
    void unload_script(int script_id);
    void reload_all();
    
    // === Event System ===
    void trigger_event(ScriptEvent event, const void* data = nullptr);
    void register_hook(int script_id, ScriptEvent event, const std::string& func_name);
    
    // === Batch Simulation ===
    struct BatchResult {
        std::string param_name;
        std::vector<double> param_values;
        std::vector<double> results;  // Energy drift, momentum, etc.
    };
    
    BatchResult run_batch(const std::string& script, 
                          const std::string& param,
                          double min_val, double max_val, int steps);

    // === REPL / Interactive ===
    std::string evaluate(const std::string& code);
    
    // === Accessors ===
    Engine* engine() { return engine_; }
    lua_State* lua_state() { return L_; }
    
    // === Configuration ===
    void set_scripts_path(const std::string& path) { scripts_path_ = path; }

private:
    Engine* engine_ = nullptr;
    lua_State* L_ = nullptr;
    bool initialized_ = false;
    std::string scripts_path_ = "scripts/";
    
    // Loaded scripts
    std::vector<LuaScript> scripts_;
    int next_script_id_ = 1;
    
    // Internal Lua setup
    void create_lua_state();
    void register_bindings();
    void register_physics_api();
    void register_render_api();
    void register_input_api();
    void register_debug_api();
    
    // Script helpers
    static int lua_panic_handler(lua_State* L);
    static void print_error(lua_State* L, const char* context);
    
    // Run script content
    bool do_string(const std::string& code, const std::string& name);
};

// ==================== Example Lua Script ====================
/*
-- Пример скрипта: oscillating_gravity.lua
-- Меняет гравитацию по синусоиде

function on_start()
    print("Oscillating Gravity started!")
    initial_gravity = get_gravity()
end

function on_update(dt)
    local t = get_time()
    local g = initial_gravity * (1.0 + 0.5 * math.sin(t * 2.0))
    set_gravity(g)
end

function on_collision(body_a, body_b, impulse)
    if impulse > 10 then
        set_velocity(body_a, 0, 10, 0)
        print("Big collision: body " .. body_a .. " launched!")
    end
end
*/

} // namespace hlp
