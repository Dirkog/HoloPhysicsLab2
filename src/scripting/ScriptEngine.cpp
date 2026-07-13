#include "ScriptEngine.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <algorithm>
#include <cmath>

extern "C" {
    #include <lua.h>
    #include <lauxlib.h>
    #include <lualib.h>
}

namespace hlp {

const char* script_event_name(ScriptEvent e) {
    switch (e) {
        case ScriptEvent::ON_START:        return "on_start";
        case ScriptEvent::ON_UPDATE:       return "on_update";
        case ScriptEvent::ON_FIXED_UPDATE: return "on_fixed_update";
        case ScriptEvent::ON_COLLISION:    return "on_collision";
        case ScriptEvent::ON_GRAB:         return "on_grab";
        case ScriptEvent::ON_RELEASE:      return "on_release";
        case ScriptEvent::ON_KEY_PRESS:    return "on_key_press";
        case ScriptEvent::ON_GESTURE:      return "on_gesture";
        case ScriptEvent::ON_SCENE_LOAD:   return "on_scene_load";
        default:                           return "unknown";
    }
}

// ==================== ScriptEngine ====================

ScriptEngine::ScriptEngine() = default;
ScriptEngine::~ScriptEngine() { shutdown(); }

int ScriptEngine::lua_panic_handler(lua_State* L) {
    const char* msg = lua_tostring(L, -1);
    std::cerr << "[Lua] PANIC: " << (msg ? msg : "unknown error") << std::endl;
    return 0;
}

void ScriptEngine::print_error(lua_State* L, const char* context) {
    const char* msg = lua_tostring(L, -1);
    std::cerr << "[Lua] Error in " << context << ": " << (msg ? msg : "unknown") << std::endl;
    lua_pop(L, 1);
}

// ==================== Initialization ====================

bool ScriptEngine::init(Engine& engine) {
    engine_ = &engine;
    
    // Create Lua state
    L_ = luaL_newstate();
    if (!L_) {
        std::cerr << "[Script] Failed to create Lua state" << std::endl;
        return false;
    }
    
    lua_atpanic(L_, lua_panic_handler);
    luaL_openlibs(L_);
    
    // Register API
    register_bindings();
    
    initialized_ = true;
    std::cout << "[Script] Lua engine initialized" << std::endl;
    return true;
}

void ScriptEngine::shutdown() {
    if (L_) {
        lua_close(L_);
        L_ = nullptr;
    }
    scripts_.clear();
    initialized_ = false;
}

// ==================== Script Loading ====================

int ScriptEngine::load_script(const std::string& name, const std::string& source) {
    if (!initialized_) return -1;
    
    LuaScript script;
    script.name = name;
    script.source = source;
    script.compiled = false;
    
    // Compile the script
    std::string wrapped = 
        "local function __main__()\n" + source + "\nend\n"
        "local ok, err = pcall(__main__)\n"
        "if not ok then print('[Script] ' .. tostring(err)) end";
    
    if (luaL_loadstring(L_, wrapped.c_str()) != LUA_OK) {
        print_error(L_, ("compile: " + name).c_str());
        return -1;
    }
    
    // Run the wrapper to define functions
    if (lua_pcall(L_, 0, 0, 0) != LUA_OK) {
        print_error(L_, ("init: " + name).c_str());
        return -1;
    }
    
    script.compiled = true;
    int id = next_script_id_++;
    scripts_.push_back(script);
    
    std::cout << "[Script] Loaded: " << name << " (id=" << id << ")" << std::endl;
    return id;
}

bool ScriptEngine::run_script(int script_id) {
    if (!initialized_) return false;
    
    for (auto& s : scripts_) {
        if (!s.enabled) continue;
        
        // Call on_update()
        lua_getglobal(L_, "on_update");
        if (lua_isfunction(L_, -1)) {
            lua_pushnumber(L_, 1.0/60.0); // dt
            if (lua_pcall(L_, 1, 0, 0) != LUA_OK) {
                print_error(L_, ("runtime: " + s.name).c_str());
            }
        } else {
            lua_pop(L_, 1);
        }
    }
    return true;
}

bool ScriptEngine::run_script_file(const std::string& path) {
    std::ifstream file(path);
    if (!file.is_open()) {
        std::cerr << "[Script] Cannot open: " << path << std::endl;
        return false;
    }
    
    std::stringstream ss;
    ss << file.rdbuf();
    
    // Extract name from path
    auto pos = path.find_last_of("/\\");
    std::string name = (pos != std::string::npos) ? path.substr(pos + 1) : path;
    
    return load_script(name, ss.str()) >= 0;
}

void ScriptEngine::unload_script(int script_id) {
    scripts_.erase(
        std::remove_if(scripts_.begin(), scripts_.end(),
            [script_id](const LuaScript& s) { 
                return false; // Keep for now
            }),
        scripts_.end());
}

void ScriptEngine::reload_all() {
    // Re-run all scripts
    for (auto& s : scripts_) {
        if (luaL_loadstring(L_, s.source.c_str()) == LUA_OK) {
            lua_pcall(L_, 0, 0, 0);
        }
    }
}

// ==================== Event System ====================

void ScriptEngine::trigger_event(ScriptEvent event, const void* data) {
    if (!initialized_) return;
    
    const char* func_name = script_event_name(event);
    lua_getglobal(L_, func_name);
    
    if (!lua_isfunction(L_, -1)) {
        lua_pop(L_, 1);
        return;
    }
    
    // Push arguments based on event type
    switch (event) {
        case ScriptEvent::ON_COLLISION: {
            auto* cd = (const CollisionEvent*)data;
            if (cd) {
                lua_pushinteger(L_, cd->body_a);
                lua_pushinteger(L_, cd->body_b);
                lua_pushnumber(L_, cd->impulse_magnitude);
                if (lua_pcall(L_, 3, 0, 0) != LUA_OK)
                    print_error(L_, "on_collision");
            }
            break;
        }
        case ScriptEvent::ON_UPDATE: {
            lua_pushnumber(L_, 1.0/60.0);
            if (lua_pcall(L_, 1, 0, 0) != LUA_OK)
                print_error(L_, "on_update");
            break;
        }
        default: {
            if (lua_pcall(L_, 0, 0, 0) != LUA_OK)
                print_error(L_, func_name);
            break;
        }
    }
}

void ScriptEngine::register_hook(int script_id, ScriptEvent event, 
                                   const std::string& func_name) {
    // Hooks are global functions - just verify they exist
    lua_getglobal(L_, func_name.c_str());
    if (!lua_isfunction(L_, -1)) {
        std::cerr << "[Script] Hook not found: " << func_name << std::endl;
    }
    lua_pop(L_, 1);
}

// ==================== Batch Simulation ====================

ScriptEngine::BatchResult ScriptEngine::run_batch(
    const std::string& script,
    const std::string& param,
    double min_val, double max_val, int steps) {
    
    BatchResult result;
    result.param_name = param;
    
    load_script("batch_" + param, script);
    
    for (int i = 0; i < steps; i++) {
        double val = min_val + (max_val - min_val) * i / (steps - 1);
        
        // Set parameter via Lua
        std::string cmd = param + " = " + std::to_string(val);
        if (luaL_loadstring(L_, cmd.c_str()) == LUA_OK)
            lua_pcall(L_, 0, 0, 0);
        
        // Run simulation for N frames
        // (in production: run 1000 steps and measure energy drift)
        
        result.param_values.push_back(val);
        result.results.push_back(0.0); // Placeholder
    }
    
    return result;
}

// ==================== REPL ====================

std::string ScriptEngine::evaluate(const std::string& code) {
    if (!initialized_) return "Engine not initialized";
    
    if (luaL_loadstring(L_, code.c_str()) != LUA_OK) {
        const char* err = lua_tostring(L_, -1);
        std::string result = err ? err : "compile error";
        lua_pop(L_, 1);
        return "Error: " + result;
    }
    
    if (lua_pcall(L_, 0, 1, 0) != LUA_OK) {
        const char* err = lua_tostring(L_, -1);
        std::string result = err ? err : "runtime error";
        lua_pop(L_, 1);
        return "Error: " + result;
    }
    
    // Get return value
    const char* ret = lua_tostring(L_, -1);
    std::string result = ret ? ret : "(nil)";
    lua_pop(L_, 1);
    return result;
}

// ==================== Lua Bindings ====================

#define REGISTER_FUNC(name, func) \
    lua_pushcfunction(L_, func); \
    lua_setglobal(L_, name);

void ScriptEngine::register_bindings() {
    register_physics_api();
    register_render_api();
    register_input_api();
    register_debug_api();
}

void ScriptEngine::register_physics_api() {
    REGISTER_FUNC("create_sphere", ScriptBinding::PhysicsAPI::create_sphere);
    REGISTER_FUNC("create_box", ScriptBinding::PhysicsAPI::create_box);
    REGISTER_FUNC("create_spring", ScriptBinding::PhysicsAPI::create_spring);
    REGISTER_FUNC("remove_body", ScriptBinding::PhysicsAPI::remove_body);
    REGISTER_FUNC("clear_all", ScriptBinding::PhysicsAPI::clear_all);
    REGISTER_FUNC("get_position", ScriptBinding::PhysicsAPI::get_position);
    REGISTER_FUNC("set_position", ScriptBinding::PhysicsAPI::set_position);
    REGISTER_FUNC("get_velocity", ScriptBinding::PhysicsAPI::get_velocity);
    REGISTER_FUNC("set_velocity", ScriptBinding::PhysicsAPI::set_velocity);
    REGISTER_FUNC("apply_force", ScriptBinding::PhysicsAPI::apply_force);
    REGISTER_FUNC("apply_impulse", ScriptBinding::PhysicsAPI::apply_impulse);
    REGISTER_FUNC("get_body_count", ScriptBinding::PhysicsAPI::get_body_count);
    REGISTER_FUNC("get_gravity", ScriptBinding::PhysicsAPI::get_gravity);
    REGISTER_FUNC("set_gravity", ScriptBinding::PhysicsAPI::set_gravity);
    REGISTER_FUNC("load_preset", ScriptBinding::PhysicsAPI::load_preset);
}

void ScriptEngine::register_render_api() {
    REGISTER_FUNC("set_background_color", ScriptBinding::RenderAPI::set_background_color);
    REGISTER_FUNC("set_camera_target", ScriptBinding::RenderAPI::set_camera_target);
}

void ScriptEngine::register_input_api() {
    REGISTER_FUNC("is_key_pressed", ScriptBinding::InputAPI::is_key_pressed);
    REGISTER_FUNC("get_hand_gesture", ScriptBinding::InputAPI::get_hand_gesture);
}

void ScriptEngine::register_debug_api() {
    REGISTER_FUNC("print", ScriptBinding::DebugAPI::print);
    REGISTER_FUNC("log", ScriptBinding::DebugAPI::log);
    REGISTER_FUNC("draw_line", ScriptBinding::DebugAPI::draw_line);
    REGISTER_FUNC("draw_text", ScriptBinding::DebugAPI::draw_text);
}

// ==================== Physics API Implementation ====================

int ScriptBinding::PhysicsAPI::create_sphere(lua_State* L) {
    double x = luaL_optnumber(L, 1, 0);
    double y = luaL_optnumber(L, 2, 3);
    double z = luaL_optnumber(L, 3, 0);
    double mass = luaL_optnumber(L, 4, 1);
    double radius = luaL_optnumber(L, 5, 0.5);
    
    // Get engine from Lua registry
    lua_getfield(L, LUA_REGISTRYINDEX, "__engine");
    auto* engine = (Engine*)lua_touserdata(L, -1);
    lua_pop(L, 1);
    
    if (engine) {
        BodyState state;
        state.position = dvec3(x, y, z);
        BodyProperties props;
        props.mass = mass;
        props.radius = radius;
        VisualProperties visual;
        visual.color = glm::vec3(0, 1, 0.8f);
        
        auto& body = engine->scene().add_body(state, props, visual);
        lua_pushinteger(L, body.id);
        return 1;
    }
    
    lua_pushinteger(L, -1);
    return 1;
}

int ScriptBinding::PhysicsAPI::get_body_count(lua_State* L) {
    lua_getfield(L, LUA_REGISTRYINDEX, "__engine");
    auto* engine = (Engine*)lua_touserdata(L, -1);
    lua_pop(L, 1);
    lua_pushinteger(L, engine ? (int)engine->scene().bodies().size() : 0);
    return 1;
}

int ScriptBinding::PhysicsAPI::get_gravity(lua_State* L) {
    lua_getfield(L, LUA_REGISTRYINDEX, "__engine");
    auto* engine = (Engine*)lua_touserdata(L, -1);
    lua_pop(L, 1);
    lua_pushnumber(L, engine ? engine->scene().params().gravity : 9.81);
    return 1;
}

int ScriptBinding::PhysicsAPI::set_gravity(lua_State* L) {
    double g = luaL_checknumber(L, 1);
    lua_getfield(L, LUA_REGISTRYINDEX, "__engine");
    auto* engine = (Engine*)lua_touserdata(L, -1);
    lua_pop(L, 1);
    if (engine) engine->scene().params().gravity = g;
    return 0;
}

int ScriptBinding::PhysicsAPI::load_preset(lua_State* L) {
    const char* name = luaL_checkstring(L, 1);
    lua_getfield(L, LUA_REGISTRYINDEX, "__engine");
    auto* engine = (Engine*)lua_touserdata(L, -1);
    lua_pop(L, 1);
    if (engine) engine->scene().load_preset(name ? name : "sandbox_demo");
    return 0;
}

int ScriptBinding::DebugAPI::print(lua_State* L) {
    const char* msg = lua_tostring(L, 1);
    if (msg) std::cout << "[Lua] " << msg << std::endl;
    return 0;
}

// Stub implementations for remaining functions
int ScriptBinding::PhysicsAPI::create_box(lua_State* L) { return create_sphere(L); }
int ScriptBinding::PhysicsAPI::create_spring(lua_State* L) { lua_pushboolean(L, 0); return 1; }
int ScriptBinding::PhysicsAPI::remove_body(lua_State* L) { return 0; }
int ScriptBinding::PhysicsAPI::clear_all(lua_State* L) { return 0; }
int ScriptBinding::PhysicsAPI::get_position(lua_State* L) { return 0; }
int ScriptBinding::PhysicsAPI::set_position(lua_State* L) { return 0; }
int ScriptBinding::PhysicsAPI::set_velocity(lua_State* L) { return 0; }
int ScriptBinding::PhysicsAPI::get_velocity(lua_State* L) { return 0; }
int ScriptBinding::PhysicsAPI::apply_force(lua_State* L) { return 0; }
int ScriptBinding::PhysicsAPI::apply_impulse(lua_State* L) { return 0; }
int ScriptBinding::PhysicsAPI::set_damping(lua_State* L) { return 0; }
int ScriptBinding::PhysicsAPI::find_nearest(lua_State* L) { return 0; }
int ScriptBinding::PhysicsAPI::raycast(lua_State* L) { return 0; }
int ScriptBinding::PhysicsAPI::save_scene(lua_State* L) { return 0; }
int ScriptBinding::PhysicsAPI::load_scene(lua_State* L) { return 0; }
int ScriptBinding::RenderAPI::set_background_color(lua_State* L) { return 0; }
int ScriptBinding::RenderAPI::set_fog(lua_State* L) { return 0; }
int ScriptBinding::RenderAPI::set_camera_target(lua_State* L) { return 0; }
int ScriptBinding::InputAPI::is_key_pressed(lua_State* L) { lua_pushboolean(L, 0); return 1; }
int ScriptBinding::InputAPI::get_mouse_position(lua_State* L) { return 0; }
int ScriptBinding::InputAPI::get_hand_position(lua_State* L) { return 0; }
int ScriptBinding::InputAPI::get_hand_gesture(lua_State* L) { lua_pushstring(L, "unknown"); return 1; }
int ScriptBinding::DebugAPI::draw_line(lua_State* L) { return 0; }
int ScriptBinding::DebugAPI::draw_text(lua_State* L) { return 0; }
int ScriptBinding::DebugAPI::log(lua_State* L) {
    const char* msg = lua_tostring(L, 1);
    if (msg) std::cout << "[Lua Log] " << msg << std::endl;
    return 0;
}

} // namespace hlp
