#pragma once

// ============================================================
// HoloPhysics Lab 2 — Главное приложение
// Интеграция: Физика + CV + Жесты + AR + UI
// ============================================================

#include "core/Engine.h"
#include "core/Scene.h"
#include "core/Solver.h"
#include "physics/CollisionSystem.h"
#include "physics/ConstraintSolver.h"
#include "physics/RotationalDynamics.h"
#include "physics/AdaptiveTimestep.h"
#include "physics/BoundaryConditions.h"
#include "render/Renderer.h"
#include "render/ARRenderer.h"
#include "cv/CameraCapture.h"
#include "cv/HandTracker.h"
#include "input/VirtualHand.h"
#include "interaction/HandInteraction.h"
#include <memory>
#include <atomic>
#include <thread>

struct GLFWwindow;

namespace hlp {

class HoloPhysicsApp {
public:
    HoloPhysicsApp();
    ~HoloPhysicsApp();

    // === Lifecycle ===
    bool init(int width = 1600, int height = 1000);
    void shutdown();
    void run();
    void stop() { running_ = false; }

    // === Accessors ===
    Engine& engine() { return *engine_; }
    Renderer& renderer() { return *renderer_; }
    ARRenderer& ar_renderer() { return *ar_renderer_; }
    GestureController& gestures() { return *gesture_controller_; }
    HandInteractionSystem& interaction() { return *interaction_system_; }
    
    GLFWwindow* window() const { return window_; }
    
    // === Configuration ===
    bool ar_mode_enabled() const { return ar_mode_; }
    void set_ar_mode(bool enabled);

private:
    // Core systems
    std::unique_ptr<Engine> engine_;
    std::unique_ptr<Renderer> renderer_;
    std::unique_ptr<ConstraintSolver> constraint_solver_;
    std::unique_ptr<BoundarySystem> boundary_system_;
    std::unique_ptr<AdaptiveTimestep> adaptive_timestep_;

    // CV + Input
    std::unique_ptr<CameraCapture> camera_;
    std::unique_ptr<HandTracker> hand_tracker_;
    std::unique_ptr<GestureController> gesture_controller_;

    // Interaction
    std::unique_ptr<HandInteractionSystem> interaction_system_;

    // AR
    std::unique_ptr<ARRenderer> ar_renderer_;

    // Window
    GLFWwindow* window_ = nullptr;
    bool running_ = false;
    bool ar_mode_ = false;
    
    // Timing
    double last_time_ = 0;
    double sim_time_ = 0;
    int frame_count_ = 0;
    float fps_ = 60.0f;

    // UI state
    bool show_ui_ = true;
    int selected_preset_ = 0;
    
    // Plot data
    static constexpr int PLOT_N = 300;
    int plot_idx_ = 0;
    float plot_time_[PLOT_N] = {0};
    float plot_ke_[PLOT_N] = {0};
    float plot_pe_[PLOT_N] = {0};
    float plot_te_[PLOT_N] = {0};
    float plot_mom_[PLOT_N] = {0};
    float plot_drift_[PLOT_N] = {0};

    // === Internal methods ===
    bool init_glfw();
    void init_ui_systems();
    void init_ar_systems();
    
    void main_loop();
    void process_input();
    void update_physics(double dt);
    void update_hand_interaction(double dt);
    void render_frame();
    void render_ui();
    void collect_plots();

    // UI panels
    void render_menu_bar();
    void render_experiments_panel();
    void render_params_panel();
    void render_controls_panel();
    void render_bodies_panel();
    void render_plots_panel();
    void render_ar_panel();
    void render_metrics_panel();

    // GLFW callbacks (static)
    static void glfw_key_cb(GLFWwindow*, int, int, int, int);
    static void glfw_mouse_cb(GLFWwindow*, int, int, int);
    static void glfw_cursor_cb(GLFWwindow*, double, double);
    static void glfw_scroll_cb(GLFWwindow*, double, double);
    static void glfw_drop_cb(GLFWwindow*, int, const char**);
    static void glfw_resize_cb(GLFWwindow*, int, int);
};

} // namespace hlp
