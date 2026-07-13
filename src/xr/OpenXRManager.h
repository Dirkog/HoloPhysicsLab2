#pragma once

// ============================================================
// HoloPhysics Lab 2 — OpenXR Integration Layer
//
// Замена GLFW + CameraCapture на промышленный стандарт OpenXR
// Поддержка: Quest 3, Apple Vision Pro, HoloLens 2, Pico 4
//
// Возможности:
//   - 6DOF трекинг головы (HMD pose)
//   - Нативный passthrough (без хромакея)
//   - Аппаратный трекинг рук (без MediaPipe)
//   - Контроллеры с тактильной отдачей
// ============================================================

#include <openxr/openxr.h>
#include <openxr/openxr_platform.h>
#include "../core/Types.h"
#include "../input/VirtualHand.h"
#include <vector>
#include <array>
#include <memory>
#include <functional>

namespace hlp {

// ==================== OpenXR Configuration ====================

struct XRConfig {
    bool enable_passthrough = true;      // Camera passthrough (HoloLens/Quest)
    bool enable_hand_tracking = true;    // Native hand tracking
    float render_scale = 1.0f;           // Resolution multiplier
    int32_t preferred_view_width = 0;    // 0 = auto
    int32_t preferred_view_height = 0;
    std::string application_name = "HoloPhysics Lab 2";
    uint32_t application_version = 3;
};

// ==================== XR Hand Tracking ====================

struct XRHandJoint {
    XrHandJointEXT joint;
    XrSpaceLocationEXT location;
    dvec3 position{0};
    dquat orientation{1, 0, 0, 0};
    float radius = 0.02f;
    bool tracked = false;
};

struct XRHandData {
    std::array<XRHandJoint, XR_HAND_JOINT_COUNT_EXT> joints;
    XrHandTrackingAimStateFB aim_state;
    bool is_active = false;

    // Convert OpenXR joints to VirtualHand
    VirtualHand to_virtual_hand(bool is_right) const;
};

struct XRDeviceState {
    // HMD
    XrSpaceLocationFlags hmd_tracking = 0;
    dvec3 hmd_position{0};
    dquat hmd_orientation{1, 0, 0, 0};
    glm::mat4 hmd_pose{1.0f};

    // Hands
    XRHandData hands[2]; // 0=right, 1=left

    // Controllers
    XrBool32 controller_active[2] = {false, false};
    XrVector3f controller_pos[2];
    XrQuaternionf controller_rot[2];
};

// ==================== OpenXR Manager ====================

class OpenXRManager {
public:
    OpenXRManager();
    ~OpenXRManager();

    // === Lifecycle ===
    bool init(const XRConfig& config);
    void shutdown();
    bool begin_session();
    void end_session();
    bool is_running() const { return session_running_; }

    // === Per-frame update ===
    struct FrameResult {
        XRDeviceState state;
        std::array<XrCompositionLayerProjectionView, 2> views;
        uint32_t view_count = 0;
        XrTime predicted_display_time = 0;
        bool should_render = false;
    };
    
    FrameResult begin_frame();
    void end_frame(const std::array<XrCompositionLayerProjectionView, 2>& views,
                   uint32_t view_count);

    // === Rendering resources ===
    struct SwapchainInfo {
        XrSwapchain handle = XR_NULL_HANDLE;
        int32_t width = 0, height = 0;
        std::vector<XrSwapchainImageOpenGLKHR> images;
    };
    
    const std::vector<SwapchainInfo>& swapchains() const { return swapchains_; }
    XrSession session() const { return session_; }
    XrInstance instance() const { return instance_; }
    
    // === Passthrough (FB passthrough extension) ===
    void enable_passthrough();
    void disable_passthrough();
    bool passthrough_enabled() const { return passthrough_enabled_; }

    // === Hand tracking ===
    void update_hand_tracking(XrTime time, XRDeviceState& state);
    XrHandTrackerEXT hand_tracker(int hand) const { return hand_trackers_[hand]; }

    // === Haptic feedback ===
    void trigger_haptic(int hand, float amplitude = 0.5f, float duration = 0.1f);

    // === Calibration ===
    XRConfig& config() { return config_; }
    const XRConfig& config() const { return config_; }

private:
    XRConfig config_;
    
    // Core OpenXR objects
    XrInstance instance_ = XR_NULL_HANDLE;
    XrSystemId system_id_ = XR_NULL_SYSTEM_ID;
    XrSession session_ = XR_NULL_HANDLE;
    XrSpace local_space_ = XR_NULL_HANDLE;
    XrSpace stage_space_ = XR_NULL_HANDLE;
    XrFormFactor form_factor_ = XR_FORM_FACTOR_HEAD_MOUNTED_DISPLAY;
    XrViewConfigurationType view_config_ = XR_VIEW_CONFIGURATION_TYPE_PRIMARY_STEREO;
    
    // Graphics binding
    XrGraphicsBindingOpenGLWin32KHR graphics_binding_{};
    XrSessionCreateInfo session_create_info_{};
    
    // Swapchains
    std::vector<SwapchainInfo> swapchains_;
    XrSwapchain depth_swapchain_ = XR_NULL_HANDLE;
    
    // Reference spaces
    XrReferenceSpaceCreateInfo local_space_info_{};
    XrReferenceSpaceCreateInfo stage_space_info_{};
    
    // Frame state
    XrFrameState frame_state_{};
    XrFrameWaitInfo frame_wait_info_{};
    XrFrameBeginInfo frame_begin_info_{};
    
    // Session state
    bool session_running_ = false;
    bool application_running_ = true;
    bool passthrough_enabled_ = false;
    
    // Hand tracking
    XrHandTrackerEXT hand_trackers_[2] = {XR_NULL_HANDLE, XR_NULL_HANDLE};
    XrHandTrackingAimStateFB aim_states_[2];
    bool hand_tracking_initialized_ = false;
    
    // Extension function pointers
    PFN_xrCreateHandTrackerEXT pfn_xrCreateHandTrackerEXT_ = nullptr;
    PFN_xrDestroyHandTrackerEXT pfn_xrDestroyHandTrackerEXT_ = nullptr;
    PFN_xrLocateHandJointsEXT pfn_xrLocateHandJointsEXT_ = nullptr;
    PFN_xrCreateHandMeshSpaceMSFT pfn_xrCreateHandMeshSpaceMSFT_ = nullptr;
    PFN_xrApplyHapticFeedbackFB pfn_xrApplyHapticFeedbackFB_ = nullptr;
    
    // Passthrough extensions
    PFN_xrCreatePassthroughFB pfn_xrCreatePassthroughFB_ = nullptr;
    PFN_xrDestroyPassthroughFB pfn_xrDestroyPassthroughFB_ = nullptr;
    PFN_xrPassthroughStartFB pfn_xrPassthroughStartFB_ = nullptr;
    PFN_xrPassthroughPauseFB pfn_xrPassthroughPauseFB_ = nullptr;
    XrPassthroughFB passthrough_ = XR_NULL_HANDLE;

    // Internal init
    bool create_instance();
    bool get_system();
    bool create_session();
    bool create_swapchains();
    bool init_hand_tracking();
    void load_extension_functions();
    
    // Debug
    static void print_xr_extensions(const std::vector<XrExtensionProperties>& exts);
    static void print_xr_layers(const std::vector<XrApiLayerProperties>& layers);
};

} // namespace hlp
