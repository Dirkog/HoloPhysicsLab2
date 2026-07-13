#include "HoloPhysicsApp.h"
#include <iostream>
#include <chrono>
#include <thread>

// GLFW
#define GLFW_INCLUDE_NONE
#include <GLFW/glfw3.h>
#include <glad/gl.h>

// ImGui
#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>
#include <implot.h>

// GLM
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

namespace hlp {

// ==================== Global pointer for callbacks ====================
static HoloPhysicsApp* g_app = nullptr;

HoloPhysicsApp::HoloPhysicsApp() = default;
HoloPhysicsApp::~HoloPhysicsApp() { shutdown(); }

// ==================== INITIALIZATION ====================

bool HoloPhysicsApp::init(int width, int height) {
    if (!init_glfw()) return false;
    
    // Core physics
    engine_ = std::make_unique<Engine>();
    renderer_ = std::make_unique<Renderer>();
    renderer_->init(width, height);
    
    constraint_solver_ = std::make_unique<ConstraintSolver>();
    boundary_system_ = std::make_unique<BoundarySystem>();
    adaptive_timestep_ = std::make_unique<AdaptiveTimestep>();
    
    // UI
    init_ui_systems();
    
    // Load default scene
    engine_->scene().load_preset("sandbox_demo");
    
    // AR systems (optional)
    init_ar_systems();
    
    // Interaction (connects hands → physics)
    interaction_system_ = std::make_unique<HandInteractionSystem>(*engine_, *constraint_solver_);
    
    // Timing
    last_time_ = glfwGetTime();
    g_app = this;
    
    std::cout << "[App] HoloPhysics Lab 2 initialized" << std::endl;
    if (ar_mode_) {
        std::cout << "[App] AR mode ACTIVE - use webcam for hand tracking" << std::endl;
    } else {
        std::cout << "[App] Desktop mode - use mouse/keyboard" << std::endl;
    }
    
    return true;
}

void HoloPhysicsApp::shutdown() {
    if (camera_) camera_->stop();
    if (hand_tracker_) hand_tracker_->shutdown();
    
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImPlot::DestroyContext();
    ImGui::DestroyContext();
    
    renderer_->shutdown();
    glfwDestroyWindow(window_);
    glfwTerminate();
}

bool HoloPhysicsApp::init_glfw() {
    glfwSetErrorCallback([](int error, const char* desc) {
        std::cerr << "GLFW Error " << error << ": " << desc << std::endl;
    });
    
    if (!glfwInit()) return false;
    
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 6);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_SAMPLES, 4);
    
    window_ = glfwCreateWindow(1600, 1000, 
        "HoloPhysics Lab 2 — Научная песочница [AR режим: вкл/Simulation]", 
        nullptr, nullptr);
    if (!window_) return false;
    
    glfwMakeContextCurrent(window_);
    glfwSwapInterval(1);
    
    if (!gladLoadGL(glfwGetProcAddress)) return false;
    
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_MULTISAMPLE);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    
    // Callbacks
    glfwSetKeyCallback(window_, glfw_key_cb);
    glfwSetMouseButtonCallback(window_, glfw_mouse_cb);
    glfwSetCursorPosCallback(window_, glfw_cursor_cb);
    glfwSetScrollCallback(window_, glfw_scroll_cb);
    glfwSetDropCallback(window_, glfw_drop_cb);
    glfwSetFramebufferSizeCallback(window_, glfw_resize_cb);
    
    return true;
}

void HoloPhysicsApp::init_ui_systems() {
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImPlot::CreateContext();
    
    ImGuiIO& io = ImGui::GetIO();
    io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;
    io.ConfigFlags |= ImGuiConfigFlags_ViewportsEnable;
    
    // Scientific dark theme
    ImGui::StyleColorsDark();
    auto& style = ImGui::GetStyle();
    style.WindowRounding = 4.0f;
    style.FrameRounding = 3.0f;
    style.GrabRounding = 3.0f;
    style.WindowBorderSize = 0.0f;
    
    ImVec4* c = style.Colors;
    c[ImGuiCol_WindowBg]       = ImVec4(0.08f, 0.08f, 0.10f, 1.00f);
    c[ImGuiCol_MenuBarBg]      = ImVec4(0.06f, 0.06f, 0.08f, 1.00f);
    c[ImGuiCol_FrameBg]        = ImVec4(0.12f, 0.12f, 0.15f, 1.00f);
    c[ImGuiCol_FrameBgHovered] = ImVec4(0.18f, 0.18f, 0.22f, 1.00f);
    c[ImGuiCol_FrameBgActive]  = ImVec4(0.22f, 0.22f, 0.27f, 1.00f);
    c[ImGuiCol_SliderGrab]     = ImVec4(0.26f, 0.59f, 0.98f, 1.00f);
    c[ImGuiCol_Button]         = ImVec4(0.15f, 0.15f, 0.19f, 1.00f);
    c[ImGuiCol_ButtonHovered]  = ImVec4(0.22f, 0.22f, 0.28f, 1.00f);
    c[ImGuiCol_ButtonActive]   = ImVec4(0.28f, 0.28f, 0.35f, 1.00f);
    
    ImGui_ImplGlfw_InitForOpenGL(window_, true);
    ImGui_ImplOpenGL3_Init("#version 460");
}

void HoloPhysicsApp::init_ar_systems() {
    // Camera
    camera_ = std::make_unique<CameraCapture>();
    if (camera_->open(0, 640, 480, 60)) {
        ar_mode_ = true;
        camera_->start();
        
        // Hand tracker (simulation mode for now, MediaPipe if available)
        hand_tracker_ = std::make_unique<HandTracker>();
        hand_tracker_->init(HandTrackerMode::SIMULATION);  // Will try MediaPipe, fallback to sim
        
        // Gesture controller
        gesture_controller_ = std::make_unique<GestureController>();
        
        // AR Renderer
        ar_renderer_ = std::make_unique<ARRenderer>(*renderer_);
        ar_renderer_->init();
        
        std::cout << "[AR] Camera initialized, hand tracking active" << std::endl;
    } else {
        ar_mode_ = false;
        std::cout << "[AR] No camera found, running in desktop mode" << std::endl;
    }
}

void HoloPhysicsApp::set_ar_mode(bool enabled) {
    if (enabled && !ar_mode_ && camera_) {
        ar_mode_ = true;
        if (camera_->is_open()) camera_->start();
    } else if (!enabled && ar_mode_) {
        ar_mode_ = false;
        camera_->stop();
    }
}

// ==================== MAIN LOOP ====================

void HoloPhysicsApp::run() {
    running_ = true;
    
    while (!glfwWindowShouldClose(window_) && running_) {
        double now = glfwGetTime();
        double dt = now - last_time_;
        last_time_ = now;
        frame_count_++;
        if (dt > 0) fps_ = (float)(1.0 / dt);
        
        // Process input (hands or mouse)
        process_input();
        
        // Update physics
        update_physics(dt);
        
        // Update hand interaction
        update_hand_interaction(dt);
        
        // Render
        render_frame();
        render_ui();
        
        ImGui::Render();
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
        
        if (ImGui::GetIO().ConfigFlags & ImGuiConfigFlags_ViewportsEnable) {
            GLFWwindow* backup = glfwGetCurrentContext();
            ImGui::UpdatePlatformWindows();
            ImGui::RenderPlatformWindowsDefault();
            glfwMakeContextCurrent(backup);
        }
        
        glfwSwapBuffers(window_);
        glfwPollEvents();
    }
}

void HoloPhysicsApp::process_input() {
    // === AR / Hand Input ===
    if (ar_mode_ && camera_ && hand_tracker_ && gesture_controller_) {
        // Get latest camera frame (non-blocking)
        CameraFrame frame;
        if (camera_->read_latest(frame)) {
            // Update hand tracker
            hand_tracker_->process_frame(frame);
            
            // Update AR texture
            ar_renderer_->update_background_texture(frame);
            
            // Update gesture controller (converts to 3D)
            gesture_controller_->update(*hand_tracker_, camera_->intrinsics(), 0.016);
        }
    }
}

void HoloPhysicsApp::update_physics(double dt) {
    // Physics substeps
    engine_->step_physics();
    
    // Boundary conditions
    if (boundary_system_->active_boundary_count() > 0) {
        boundary_system_->apply(engine_->scene().bodies(), 0.016);
    }
    
    // Collect plot data
    collect_plots();
}

void HoloPhysicsApp::update_hand_interaction(double dt) {
    if (ar_mode_ && gesture_controller_ && interaction_system_) {
        interaction_system_->update(*gesture_controller_, dt);
        
        // If grabbing, apply constraint forces
        if (interaction_system_->is_grabbing()) {
            constraint_solver_->solve(engine_->scene().bodies(), dt, 5);
        }
    }
}

// ==================== RENDER ====================

void HoloPhysicsApp::render_frame() {
    // === AR Background ===
    if (ar_mode_ && ar_renderer_ && ar_renderer_->background_visible()) {
        // Render camera feed as background first
        ar_renderer_->render_background();
        
        // Then render physics scene on top (with transparency)
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    } else {
        // Standard desktop: clear to dark color
        renderer_->begin_frame();
    }
    
    // === Render Physics Scene ===
    renderer_->render_scene(
        engine_->scene().bodies(),
        engine_->scene().params(),
        engine_->collision_events()
    );
    
    // === AR Overlays (Hand skeleton, interaction points) ===
    if (ar_mode_ && ar_renderer_ && gesture_controller_) {
        // Right hand
        if (gesture_controller_->right_hand().is_active()) {
            ar_renderer_->render_hand_skeleton(gesture_controller_->right_hand(), true);
        }
        // Left hand
        if (gesture_controller_->left_hand().is_active()) {
            ar_renderer_->render_hand_skeleton(gesture_controller_->left_hand(), false);
        }
        
        // Interaction debug
        ar_renderer_->render_debug_info(interaction_system_->debug());
    }
    
    if (!ar_mode_) renderer_->end_frame();
}

// ==================== COLLECT PLOTS ====================

void HoloPhysicsApp::collect_plots() {
    auto& scene = engine_->scene();
    int i = plot_idx_ % PLOT_N;
    plot_time_[i] = (float)scene.time();
    plot_ke_[i] = (float)scene.kinetic_energy();
    plot_pe_[i] = (float)scene.potential_energy();
    plot_te_[i] = (float)scene.total_energy();
    plot_mom_[i] = (float)scene.linear_momentum();
    plot_drift_[i] = (float)scene.energy_drift() * 100.0f;
    plot_idx_++;
}

// ==================== UI ====================

void HoloPhysicsApp::render_ui() {
    // Docking
    ImGui::DockSpaceOverViewport(ImGui::GetMainViewport());
    
    render_menu_bar();
    render_experiments_panel();
    render_params_panel();
    render_controls_panel();
    render_bodies_panel();
    render_plots_panel();
    if (ar_mode_) render_ar_panel();
    render_metrics_panel();
}

void HoloPhysicsApp::render_menu_bar() {
    if (ImGui::BeginMainMenuBar()) {
        if (ImGui::BeginMenu("Файл")) {
            if (ImGui::MenuItem("Сохранить сцену", "Ctrl+S")) {
                engine_->scene().save_to_file("scene_save.json");
            }
            if (ImGui::MenuItem("Загрузить сцену", "Ctrl+O")) {
                engine_->scene().load_from_file("scene_save.json");
            }
            ImGui::Separator();
            if (ImGui::MenuItem("Экспорт JSON")) {
                auto json = engine_->scene().serialize_to_json();
                FILE* f = fopen("export_scene.json", "w");
                if (f) { fputs(json.c_str(), f); fclose(f); }
            }
            ImGui::Separator();
            if (ImGui::MenuItem("Выход", "Alt+F4")) {
                stop();
            }
            ImGui::EndMenu();
        }
        
        if (ImGui::BeginMenu("Вид")) {
            ImGui::MenuItem("AR Панель", nullptr, ar_mode_ ? &show_ui_ : &show_ui_);
            ImGui::EndMenu();
        }
        
        // === Status bar in menu ===
        ImGui::SameLine(ImGui::GetWindowWidth() - 400);
        auto& stats = engine_->stats();
        ImGui::TextColored(ImVec4(0.6f, 0.6f, 0.7f, 1.0f),
            "FPS: %.0f | E: %.2f | p: %.2f | Дрейф: %.4f%% | Тел: %zu%s",
            fps_, stats.total_energy, stats.momentum,
            stats.energy_drift * 100.0, stats.body_count,
            ar_mode_ ? " | 🤚 AR" : " | 🖱 Desk");
        
        ImGui::EndMainMenuBar();
    }
}

void HoloPhysicsApp::render_experiments_panel() {
    ImGui::Begin("Эксперименты", &show_ui_);
    
    auto presets = Scene::available_presets();
    const char* preview = presets[selected_preset_].name.c_str();
    if (ImGui::BeginCombo("Пресет", preview)) {
        for (size_t i = 0; i < presets.size(); ++i) {
            if (ImGui::Selectable(presets[i].name.c_str(), i == (size_t)selected_preset_)) {
                selected_preset_ = i;
            }
        }
        ImGui::EndCombo();
    }
    
    if (ImGui::Button("Загрузить", ImVec2(-1, 0))) {
        engine_->scene().load_preset(presets[selected_preset_].id);
        plot_idx_ = 0;
        std::cout << "[App] Loaded preset: " << presets[selected_preset_].name << std::endl;
    }
    
    ImGui::Separator();
    ImGui::Text("Текущий: %s", engine_->scene().experiment_id().c_str());
    ImGui::Text("Время: %.3f с", engine_->scene().time());
    ImGui::Text("Интегратор: %s", Solver::integrator_name(engine_->scene().integrator()));
    
    // AR Status
    if (ar_mode_) {
        ImGui::Separator();
        ImGui::TextColored(ImVec4(0, 1, 0.5f, 1), "🤚 AR режим: АКТИВЕН");
        if (gesture_controller_) {
            auto g = gesture_controller_->right_hand().gesture();
            ImGui::Text("Жест: %s", gesture_name(g));
            ImGui::Text("Щипок: %.2f", gesture_controller_->right_hand().pinch_strength());
        }
    }
    
    ImGui::End();
}

void HoloPhysicsApp::render_params_panel() {
    ImGui::Begin("Параметры физики", &show_ui_);
    auto& params = engine_->scene().params();
    
    if (ImGui::CollapsingHeader("Базовые", ImGuiTreeNodeFlags_DefaultOpen)) {
        ImGui::DragDouble("Гравитация", &params.gravity, 0.1, -20, 50);
        ImGui::DragDouble("Затухание", &params.damping, 0.001, 0, 0.5);
        ImGui::DragDouble("Упругость", &params.restitution, 0.01, 0, 1);
        ImGui::DragDouble("Скорость ×", &params.time_scale, 0.05, 0.05, 5);
    }
    
    if (ImGui::CollapsingHeader("Среда")) {
        ImGui::Checkbox("Ветер", &params.enable_wind);
        if (params.enable_wind) ImGui::DragDouble3("Направление", &params.wind.x, 0.1, -20, 20);
        ImGui::Checkbox("Сопротивление", &params.enable_drag);
        if (params.enable_drag) ImGui::DragDouble("Плотность", &params.air_density, 0.01, 0, 10);
    }
    
    if (ImGui::CollapsingHeader("Поля")) {
        ImGui::Checkbox("Магнитное", &params.enable_magnet);
        if (params.enable_magnet) ImGui::DragDouble("B (Тл)", &params.magnetic_field, 0.1, 0, 10);
        ImGui::Checkbox("Центр. гравитация", &params.enable_central_g);
        if (params.enable_central_g) ImGui::DragDouble("M", &params.central_mass, 1, 0, 1000);
    }
    
    if (ImGui::CollapsingHeader("Нелинейные")) {
        ImGui::Checkbox("Турбулентность", &params.enable_turbulence);
        if (params.enable_turbulence) ImGui::DragDouble("Ампл.", &params.turbulence, 0.1, 0, 5);
        ImGui::Checkbox("Вихрь", &params.enable_vortex);
        if (params.enable_vortex) ImGui::DragDouble("Сила", &params.vortex_strength, 0.1, 0, 10);
        ImGui::Checkbox("Кориолис", &params.enable_coriolis);
        if (params.enable_coriolis) ImGui::DragDouble("ω", &params.coriolis, 0.01, 0, 5);
    }
    
    if (ImGui::CollapsingHeader("Взаимодействия")) {
        ImGui::Checkbox("Кулон", &params.enable_coulomb);
        if (params.enable_coulomb) ImGui::DragDouble("K", &params.coulomb_k, 0.1, 0, 10);
        ImGui::Checkbox("Когезия", &params.enable_cohesion);
        if (params.enable_cohesion) ImGui::DragDouble("Сила", &params.cohesion, 0.01, 0, 5);
    }
    
    ImGui::End();
}

void HoloPhysicsApp::render_controls_panel() {
    ImGui::Begin("Управление", &show_ui_);
    
    if (ImGui::Button(engine_->paused() ? "▶ Пуск" : "⏸ Пауза", ImVec2(-1, 0))) {
        engine_->set_paused(!engine_->paused());
    }
    
    ImGui::Separator();
    ImGui::Text("Импульсы:");
    auto apply_impulse = [&](double x, double y, double z) {
        if (interaction_system_->highlighted_body() != UINT64_MAX) {
            engine_->apply_impulse(interaction_system_->highlighted_body(), dvec3(x, y, z));
        }
    };
    
    if (ImGui::Button("↑", ImVec2(-1, 30))) apply_impulse(0, 5, 0);
    
    if (ImGui::Button("💥 Взрыв", ImVec2(-1, 0))) {
        engine_->explode(dvec3(0, 3, 0), 6, 15);
    }
    
    if (ImGui::Button("❄ Стоп", ImVec2(-1, 0))) engine_->freeze_all();
    if (ImGui::Button("🎲 Случайные тела", ImVec2(-1, 0))) engine_->spawn_random_bodies(5);
    
    ImGui::Separator();
    
    // AR Controls
    if (ar_mode_) {
        ImGui::TextColored(ImVec4(0, 1, 0.5f, 1), "🤚 Управление руками:");
        ImGui::BulletText("Щипок (Pinch) — захватить тело");
        ImGui::BulletText("Кулак (Fist) — оттолкнуть всё");
        ImGui::BulletText("Указание (Point) — выбрать");
        ImGui::BulletText("Ладонь (Open) — силовое поле");
    }
    
    ImGui::End();
}

void HoloPhysicsApp::render_bodies_panel() {
    ImGui::Begin("Список тел", &show_ui_);
    
    const auto& bodies = engine_->scene().bodies();
    ImGui::Text("Всего тел: %zu", bodies.size());
    ImGui::Separator();
    
    ImGui::BeginChild("list", ImVec2(0, -60), true);
    for (const auto& b : bodies) {
        bool highlighted = (b.id == interaction_system_->highlighted_body());
        bool grabbed = (b.id == interaction_system_->grabbed_body());
        bool selected = highlighted || grabbed;
        
        ImGui::PushID((int)b.id);
        ImVec4 col(b.visual.color.r, b.visual.color.g, b.visual.color.b, 1);
        ImGui::ColorButton("c", col, ImGuiColorEditFlags_NoTooltip, ImVec2(12, 12));
        ImGui::SameLine();
        
        char label[64];
        snprintf(label, 64, "#%llu %s | m=%.1f v=%.1f%s",
                 (unsigned long long)b.id,
                 b.is_static() ? "⚓" : "○",
                 b.props.mass, b.speed(),
                 grabbed ? " ✊" : "");
        
        if (ImGui::Selectable(label, selected)) {
            // Select via click (also works with AR pointing)
        }
        ImGui::PopID();
    }
    ImGui::EndChild();
    
    if (ImGui::Button("Удалить выбранное", ImVec2(-1, 0))) {
        auto id = interaction_system_->highlighted_body();
        if (id != UINT64_MAX) {
            engine_->scene().remove_body(id);
        }
    }
    
    ImGui::End();
}

void HoloPhysicsApp::render_plots_panel() {
    ImGui::Begin("Графики", &show_ui_);
    
    int count = std::min(plot_idx_, PLOT_N);
    
    if (ImPlot::BeginPlot("Энергия", ImVec2(-1, 200))) {
        ImPlot::SetupAxes("Время (с)", "Энергия (Дж)");
        ImPlot::PlotLine("E_k", plot_time_, plot_ke_, count);
        ImPlot::PlotLine("E_p", plot_time_, plot_pe_, count);
        ImPlot::PlotLine("E_total", plot_time_, plot_te_, count);
        ImPlot::EndPlot();
    }
    
    if (ImPlot::BeginPlot("Импульс", ImVec2(-1, 100))) {
        ImPlot::SetupAxes("Время (с)", "p");
        ImPlot::PlotLine("|p|", plot_time_, plot_mom_, count);
        ImPlot::EndPlot();
    }
    
    if (ImPlot::BeginPlot("Дрейф энергии", ImVec2(-1, 80))) {
        ImPlot::SetupAxes("Время (с)", "ΔE/E (%)");
        ImPlot::PlotLine("Дрейф", plot_time_, plot_drift_, count);
        ImPlot::EndPlot();
    }
    
    ImGui::End();
}

void HoloPhysicsApp::render_ar_panel() {
    ImGui::Begin("AR Панель", nullptr);
    
    ImGui::TextColored(ImVec4(0, 1, 0.5f, 1), "🤚 Управление жестами");
    ImGui::Separator();
    
    if (gesture_controller_ && gesture_controller_->is_calibrated()) {
        ImGui::TextColored(ImVec4(0, 1, 0, 1), "✅ Откалибровано");
    } else {
        ImGui::TextColored(ImVec4(1, 1, 0, 1), "⏳ Калибровка...");
    }
    
    ImGui::Separator();
    
    auto& right = gesture_controller_->right_hand();
    if (right.is_active()) {
        ImGui::Text("Правая рука:");
        ImGui::Text("  Жест: %s", gesture_name(right.gesture()));
        ImGui::Text("  Щипок: %.2f", right.pinch_strength());
        ImGui::Text("  Позиция: (%.2f, %.2f, %.2f)",
                    right.palm_center().x, right.palm_center().y, right.palm_center().z);
        
        if (right.interaction().is_grabbing) {
            ImGui::TextColored(ImVec4(1, 0.5f, 0, 1), "✊ Захват тела #%llu",
                (unsigned long long)right.interaction().grabbed_body_id);
        }
    } else {
        ImGui::Text("Правая рука: не обнаружена");
    }
    
    ImGui::Separator();
    
    // Interaction stats
    auto& interaction = interaction_system_->debug();
    if (!interaction.affected_bodies.empty()) {
        ImGui::Text("Затронуто тел: %zu", interaction.affected_bodies.size());
    }
    
    ImGui::Separator();
    
    // AR Settings
    ImGui::Text("Настройки AR:");
    if (ar_renderer_) {
        bool bg = ar_renderer_->background_visible();
        if (ImGui::Checkbox("Фон камеры", &bg)) ar_renderer_->set_background_visible(bg);
        
        bool mirror = ar_renderer_->mirror_mode();
        if (ImGui::Checkbox("Зеркальный режим", &mirror)) ar_renderer_->set_mirror_mode(mirror);
    }
    
    if (ImGui::Button("🔄 Перекалибровать", ImVec2(-1, 0))) {
        if (gesture_controller_) gesture_controller_->calibrate();
    }
    
    ImGui::End();
}

void HoloPhysicsApp::render_metrics_panel() {
    ImGui::Begin("Метрики", &show_ui_);
    
    auto& stats = engine_->stats();
    auto& scene = engine_->scene();
    
    ImGui::Text("Произв-сть");
    ImGui::Separator();
    ImGui::Text("FPS: %.1f", fps_);
    ImGui::Text("Шагов: %d", stats.step_count);
    ImGui::Separator();
    
    ImGui::Text("Энергия");
    ImGui::Separator();
    ImGui::Text("E_k: %.6f Дж", scene.kinetic_energy());
    ImGui::Text("E_p: %.6f Дж", scene.potential_energy());
    ImGui::Text("Дрейф: %.8f%%", stats.energy_drift * 100.0);
    ImGui::Separator();
    
    ImGui::Text("Метод");
    if (ImGui::BeginCombo("Интегратор", Solver::integrator_name(scene.integrator()))) {
        for (int i = 0; i < Solver::integrator_count(); ++i) {
            auto t = static_cast<IntegratorType>(i);
            if (ImGui::Selectable(Solver::integrator_name(t), i == (int)scene.integrator())) {
                scene.set_integrator(t);
            }
        }
        ImGui::EndCombo();
    }
    
    ImGui::SliderInt("Субшаги", &engine_->substeps(), 1, 8);
    
    if (ImGui::Button("Сбросить энергию", ImVec2(-1, 0))) {
        scene.reset_energy_tracking();
    }
    
    ImGui::End();
}

// ==================== CALLBACKS ====================

void HoloPhysicsApp::glfw_key_cb(GLFWwindow*, int key, int, int action, int mods) {
    if (!g_app || action != GLFW_PRESS) return;
    
    if (key == GLFW_KEY_SPACE) g_app->engine_->set_paused(!g_app->engine_->paused());
    if (key == GLFW_KEY_R && !(mods & GLFW_MOD_CONTROL)) {
        g_app->engine_->scene().load_preset(g_app->engine_->scene().experiment_id());
        g_app->plot_idx_ = 0;
    }
    if (key == GLFW_KEY_F) g_app->engine_->set_gravity_well(!g_app->engine_->gravity_well_active());
    if (key == GLFW_KEY_X) g_app->engine_->explode(dvec3(0, 3, 0), 6, 15);
    if (key == GLFW_KEY_DELETE) {
        auto id = g_app->interaction_system_->highlighted_body();
        if (id != UINT64_MAX) g_app->engine_->scene().remove_body(id);
    }
    if (key == GLFW_KEY_ESCAPE) g_app->stop();
    
    // Presets 1-9
    if (key >= GLFW_KEY_1 && key <= GLFW_KEY_9) {
        int idx = key - GLFW_KEY_1;
        auto presets = Scene::available_presets();
        if (idx < (int)presets.size()) {
            g_app->engine_->scene().load_preset(presets[idx].id);
            g_app->plot_idx_ = 0;
        }
    }
}

void HoloPhysicsApp::glfw_mouse_cb(GLFWwindow*, int button, int action, int) {
    if (!g_app || action != GLFW_PRESS) return;
    
    double x, y;
    glfwGetCursorPos(g_app->window_, &x, &y);
    
    int w, h;
    glfwGetFramebufferSize(g_app->window_, &w, &h);
    
    // Only pick if not over ImGui
    if (!ImGui::GetIO().WantCaptureMouse) {
        // Ray pick
    }
}

void HoloPhysicsApp::glfw_cursor_cb(GLFWwindow*, double x, double y) {
    // Called by GLFW, camera orbit handled here
    static bool dragging = false;
    static double last_x = 0, last_y = 0;
    
    if (glfwGetMouseButton(g_app->window_, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS) {
        if (!dragging) { last_x = x; last_y = y; dragging = true; }
        double dx = x - last_x, dy = y - last_y;
        g_app->renderer_->orbit_camera((float)dx * 0.005f, (float)dy * 0.005f);
        last_x = x; last_y = y;
    } else {
        dragging = false;
    }
}

void HoloPhysicsApp::glfw_scroll_cb(GLFWwindow*, double, double dy) {
    if (g_app) g_app->renderer_->zoom_camera((float)dy * 0.1f);
}

void HoloPhysicsApp::glfw_drop_cb(GLFWwindow*, int count, const char** paths) {
    if (!g_app) return;
    for (int i = 0; i < count; ++i) {
        std::string path = paths[i];
        if (path.size() > 5 && path.substr(path.size() - 5) == ".json") {
            g_app->engine_->scene().load_from_file(path);
        }
    }
}

void HoloPhysicsApp::glfw_resize_cb(GLFWwindow*, int w, int h) {
    if (g_app) g_app->renderer_->resize(w, h);
}

} // namespace hlp

// ==================== ENTRY POINT ====================

int main() {
    hlp::HoloPhysicsApp app;
    
    if (!app.init()) {
        std::cerr << "Failed to initialize HoloPhysics App" << std::endl;
        return -1;
    }
    
    std::cout << "\n==============================================" << std::endl;
    std::cout << "  HoloPhysics Lab 2 — Научная песочница" << std::endl;
    std::cout << "  C++20 / OpenGL 4.6 / ImGui / Hand Tracking" << std::endl;
    std::cout << "==============================================\n" << std::endl;
    std::cout << "Управление:" << std::endl;
    std::cout << "  Пробел    — Пауза" << std::endl;
    std::cout << "  R         — Сброс" << std::endl;
    std::cout << "  F         — Тоггл притяжения" << std::endl;
    std::cout << "  X         — Взрыв" << std::endl;
    std::cout << "  1-9       — Пресеты" << std::endl;
    std::cout << "  Delete    — Удалить выбранное" << std::endl;
    std::cout << "  Escape    — Выход" << std::endl;
    std::cout << "  Средняя   — Вращение камеры" << std::endl;
    std::cout << "  Scroll    — Zoom" << std::endl;
    
    if (app.ar_mode_enabled()) {
        std::cout << "\n  🤚 AR РЕЖИМ — используйте веб-камеру:" << std::endl;
        std::cout << "    Щипок      — Захватить тело" << std::endl;
        std::cout << "    Кулак      — Оттолкнуть всё" << std::endl;
        std::cout << "    Указание   — Выбрать тело" << std::endl;
        std::cout << "    Ладонь     — Силовое поле" << std::endl;
    }
    
    app.run();
    return 0;
}
