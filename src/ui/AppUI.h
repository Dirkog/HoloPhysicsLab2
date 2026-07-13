#pragma once

// ============================================================
// HoloPhysics Lab 2 — Scientific User Interface
// Строгий научный интерфейс в стиле MATLAB/Origin
// ============================================================

#include "../core/Engine.h"
#include "../render/Renderer.h"
#include <string>
#include <vector>
#include <array>

// Forward declarations
struct ImGuiContext;
struct ImPlotContext;

namespace hlp {

class AppUI {
public:
    AppUI();
    ~AppUI();

    bool init(GLFWwindow* window);
    void shutdown();
    void new_frame();
    void render();

    // Getters for engine access
    Engine& engine() { return *engine_; }
    Renderer& renderer() { return *renderer_; }

    // State
    bool is_running() const { return running_; }
    void request_shutdown() { running_ = false; }
    bool should_show_about() const { return show_about_; }
    void clear_about() { show_about_ = false; }

private:
    // Core systems
    std::unique_ptr<Engine> engine_;
    std::unique_ptr<Renderer> renderer_;
    
    // Window
    GLFWwindow* window_ = nullptr;
    bool running_ = true;
    
    // UI state
    bool show_controls_ = true;
    bool show_experiments_ = true;
    bool show_bodies_ = true;
    bool show_params_ = true;
    bool show_plots_ = true;
    bool show_scene_io_ = true;
    bool show_about_ = false;
    bool show_help_ = false;
    bool show_metrics_ = true;
    
    // Selected body
    uint64_t selected_body_id_ = UINT64_MAX;
    Body* selected_body_ = nullptr;
    
    // Plot data
    static constexpr int PLOT_BUFFER = 500;
    std::vector<float> plot_time_;
    std::vector<float> plot_ke_;
    std::vector<float> plot_pe_;
    std::vector<float> plot_te_;
    std::vector<float> plot_mom_;
    std::vector<float> plot_drift_;
    
    // Drag & drop / import
    std::string import_file_path_;
    
    // Layout tracking
    bool first_frame_ = true;

    // UI panels
    void render_menu_bar();
    void render_experiments_panel();
    void render_controls_panel();
    void render_params_panel();
    void render_body_list_panel();
    void render_plots_panel();
    void render_scene_io_panel();
    void render_metrics_panel();
    void render_about_dialog();
    
    // Helpers
    void render_body_properties(Body& body);
    void render_force_buttons();
    void render_preset_selector();
    void render_integrator_selector();
    void render_save_load_ui();
    
    // Plot data collection
    void collect_plot_data();
    void clear_plot_data();
    
    // Presets list
    int selected_preset_ = 0;
    
    // Save slot
    int save_slot_ = 1;
    
    // Integrator selection
    int selected_integrator_ = static_cast<int>(IntegratorType::SymplecticEuler);
};

} // namespace hlp
