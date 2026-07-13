#include "OpenXRManager.h"
#include <iostream>
#include <cstring>
#include <algorithm>

namespace hlp {

OpenXRManager::OpenXRManager() = default;
OpenXRManager::~OpenXRManager() { shutdown(); }

// ==================== Initialization ====================

void OpenXRManager::print_xr_extensions(const std::vector<XrExtensionProperties>& exts) {
    std::cout << "[OpenXR] Available extensions (" << exts.size() << "):" << std::endl;
    for (const auto& e : exts) {
        std::cout << "  " << e.extensionName << " (v" << e.extensionVersion << ")" << std::endl;
    }
}

bool OpenXRManager::create_instance() {
    // Check runtime available
    XrResult result = xrInitializeLoaderKHR(nullptr);
    if (XR_FAILED(result)) {
        std::cerr << "[OpenXR] Runtime not available, falling back to Desktop mode" << std::endl;
        return false;
    }

    // Enable required extensions
    std::vector<const char*> extensions = {
        XR_KHR_OPENGL_ENABLE_EXTENSION_NAME,
        XR_EXT_HAND_TRACKING_EXTENSION_NAME,
        XR_FB_HAND_TRACKING_AIM_EXTENSION_NAME,
        XR_FB_PASSTHROUGH_EXTENSION_NAME,
        XR_FB_HAPTICS_EXTENSION_NAME,
    };

    XrInstanceCreateInfo create_info{XR_TYPE_INSTANCE_CREATE_INFO};
    create_info.enabledExtensionCount = (uint32_t)extensions.size();
    create_info.enabledExtensionNames = extensions.data();

    strcpy(create_info.applicationInfo.applicationName, config_.application_name.c_str());
    create_info.applicationInfo.applicationVersion = config_.application_version;
    create_info.applicationInfo.apiVersion = XR_CURRENT_API_VERSION;

    result = xrCreateInstance(&create_info, &instance_);
    if (XR_FAILED(result)) {
        std::cerr << "[OpenXR] Failed to create instance: " << result << std::endl;
        return false;
    }

    // Query available extensions
    uint32_t ext_count = 0;
    xrEnumerateInstanceExtensionProperties(nullptr, 0, &ext_count, nullptr);
    std::vector<XrExtensionProperties> available_exts(ext_count, {XR_TYPE_EXTENSION_PROPERTIES});
    xrEnumerateInstanceExtensionProperties(nullptr, ext_count, &ext_count, available_exts.data());
    print_xr_extensions(available_exts);

    std::cout << "[OpenXR] Instance created successfully" << std::endl;
    return true;
}

bool OpenXRManager::get_system() {
    XrSystemGetInfo system_info{XR_TYPE_SYSTEM_GET_INFO};
    system_info.formFactor = form_factor_;

    XrResult result = xrGetSystem(instance_, &system_info, &system_id_);
    if (XR_FAILED(result)) {
        std::cerr << "[OpenXR] No HMD found, falling back to Desktop mode" << std::endl;
        return false;
    }

    // Get system properties
    XrSystemProperties sys_props{XR_TYPE_SYSTEM_PROPERTIES};
    xrGetSystemProperties(instance_, system_id_, &sys_props);
    std::cout << "[OpenXR] System: " << sys_props.systemName
              << " vendor=" << sys_props.vendorId << std::endl;

    return true;
}

bool OpenXRManager::init(const XRConfig& config) {
    config_ = config;
    
    if (!create_instance()) return false;
    if (!get_system()) { xrDestroyInstance(instance_); return false; }
    if (!create_session()) return false;
    if (!create_swapchains()) return false;
    
    load_extension_functions();
    
    if (config_.enable_hand_tracking) {
        init_hand_tracking();
    }

    if (config_.enable_passthrough) {
        enable_passthrough();
    }

    std::cout << "[OpenXR] Initialized successfully" << std::endl;
    return true;
}

void OpenXRManager::shutdown() {
    if (session_ != XR_NULL_HANDLE) {
        end_session();
        xrDestroySession(session_);
        session_ = XR_NULL_HANDLE;
    }
    
    for (auto& sc : swapchains_) {
        if (sc.handle != XR_NULL_HANDLE)
            xrDestroySwapchain(sc.handle);
    }
    swapchains_.clear();
    
    if (depth_swapchain_ != XR_NULL_HANDLE) {
        xrDestroySwapchain(depth_swapchain_);
        depth_swapchain_ = XR_NULL_HANDLE;
    }
    
    if (local_space_ != XR_NULL_HANDLE) {
        xrDestroySpace(local_space_);
        local_space_ = XR_NULL_HANDLE;
    }
    
    if (passthrough_ != XR_NULL_HANDLE) {
        pfn_xrDestroyPassthroughFB_(passthrough_);
        passthrough_ = XR_NULL_HANDLE;
    }
    
    if (instance_ != XR_NULL_HANDLE) {
        xrDestroyInstance(instance_);
        instance_ = XR_NULL_HANDLE;
    }
}

bool OpenXRManager::create_session() {
    // Graphics binding
    graphics_binding_ = {XR_TYPE_GRAPHICS_BINDING_OPENGL_WIN32_KHR};
    // In production: set hDC and hGLRC from the OpenGL context
    
    session_create_info_ = {XR_TYPE_SESSION_CREATE_INFO};
    session_create_info_.systemId = system_id_;
    session_create_info_.next = &graphics_binding_;

    XrResult result = xrCreateSession(instance_, &session_create_info_, &session_);
    if (XR_FAILED(result)) {
        std::cerr << "[OpenXR] Failed to create session: " << result << std::endl;
        return false;
    }

    // Create reference spaces
    local_space_info_ = {XR_TYPE_REFERENCE_SPACE_CREATE_INFO};
    local_space_info_.referenceSpaceType = XR_REFERENCE_SPACE_TYPE_LOCAL;
    local_space_info_.poseInReferenceSpace = {{0,0,0,1}, {0,0,0}};
    xrCreateReferenceSpace(session_, &local_space_info_, &local_space_);

    stage_space_info_ = {XR_TYPE_REFERENCE_SPACE_CREATE_INFO};
    stage_space_info_.referenceSpaceType = XR_REFERENCE_SPACE_TYPE_STAGE;
    xrCreateReferenceSpace(session_, &stage_space_info_, &stage_space_);

    return true;
}

bool OpenXRManager::create_swapchains() {
    // Get view configuration views
    uint32_t view_count = 0;
    xrEnumerateViewConfigurationViews(instance_, system_id_, view_config_, 0, &view_count, nullptr);
    std::vector<XrViewConfigurationView> views(view_count, {XR_TYPE_VIEW_CONFIGURATION_VIEW});
    xrEnumerateViewConfigurationViews(instance_, system_id_, view_config_, view_count, &view_count, views.data());

    swapchains_.resize(view_count);
    
    for (uint32_t i = 0; i < view_count; i++) {
        // Color swapchain
        XrSwapchainCreateInfo swapchain_info{XR_TYPE_SWAPCHAIN_CREATE_INFO};
        swapchain_info.usageFlags = XR_SWAPCHAIN_USAGE_COLOR_ATTACHMENT_BIT;
        swapchain_info.format = GL_SRGB8_ALPHA8;
        swapchain_info.sampleCount = 1;
        swapchain_info.width = views[i].recommendedImageRectWidth;
        swapchain_info.height = views[i].recommendedImageRectHeight;
        swapchain_info.faceCount = 1;
        swapchain_info.arraySize = 1;
        swapchain_info.mipCount = 1;

        XrSwapchain swapchain;
        XrResult result = xrCreateSwapchain(session_, &swapchain_info, &swapchain);
        if (XR_FAILED(result)) {
            std::cerr << "[OpenXR] Failed to create color swapchain " << i << std::endl;
            return false;
        }

        // Get swapchain images
        uint32_t image_count = 0;
        xrEnumerateSwapchainImages(swapchain, 0, &image_count, nullptr);
        std::vector<XrSwapchainImageOpenGLKHR> images(image_count, {XR_TYPE_SWAPCHAIN_IMAGE_OPENGL_KHR});
        xrEnumerateSwapchainImages(swapchain, image_count, &image_count, 
                                    (XrSwapchainImageBaseHeader*)images.data());

        swapchains_[i] = {swapchain, 
                          views[i].recommendedImageRectWidth,
                          views[i].recommendedImageRectHeight,
                          std::move(images)};
    }
    
    return true;
}

// ==================== Extension Loading ====================

void OpenXRManager::load_extension_functions() {
    // Hand tracking
    xrGetInstanceProcAddr(instance_, "xrCreateHandTrackerEXT",
                          (PFN_xrVoidFunction*)&pfn_xrCreateHandTrackerEXT_);
    xrGetInstanceProcAddr(instance_, "xrDestroyHandTrackerEXT",
                          (PFN_xrVoidFunction*)&pfn_xrDestroyHandTrackerEXT_);
    xrGetInstanceProcAddr(instance_, "xrLocateHandJointsEXT",
                          (PFN_xrVoidFunction*)&pfn_xrLocateHandJointsEXT_);

    // Haptics
    xrGetInstanceProcAddr(instance_, "xrApplyHapticFeedbackFB",
                          (PFN_xrVoidFunction*)&pfn_xrApplyHapticFeedbackFB_);

    // Passthrough
    xrGetInstanceProcAddr(instance_, "xrCreatePassthroughFB",
                          (PFN_xrVoidFunction*)&pfn_xrCreatePassthroughFB_);
    xrGetInstanceProcAddr(instance_, "xrDestroyPassthroughFB",
                          (PFN_xrVoidFunction*)&pfn_xrDestroyPassthroughFB_);
    xrGetInstanceProcAddr(instance_, "xrPassthroughStartFB",
                          (PFN_xrVoidFunction*)&pfn_xrPassthroughStartFB_);
    xrGetInstanceProcAddr(instance_, "xrPassthroughPauseFB",
                          (PFN_xrVoidFunction*)&pfn_xrPassthroughPauseFB_);
}

// ==================== Hand Tracking ====================

bool OpenXRManager::init_hand_tracking() {
    if (!pfn_xrCreateHandTrackerEXT_) {
        std::cerr << "[OpenXR] Hand tracking extension not available" << std::endl;
        return false;
    }

    for (int i = 0; i < 2; i++) {
        XrHandTrackerCreateInfoEXT create_info{XR_TYPE_HAND_TRACKER_CREATE_INFO_EXT};
        create_info.hand = (i == 0) ? XR_HAND_LEFT_EXT : XR_HAND_RIGHT_EXT;
        create_info.handJointSet = XR_HAND_JOINT_SET_DEFAULT_EXT;
        
        XrResult result = pfn_xrCreateHandTrackerEXT_(session_, &create_info, &hand_trackers_[i]);
        if (XR_SUCCEEDED(result)) {
            std::cout << "[OpenXR] Hand tracker " << (i == 0 ? "left" : "right") << " created" << std::endl;
        } else {
            std::cerr << "[OpenXR] Failed to create hand tracker " << i << ": " << result << std::endl;
        }
    }
    
    hand_tracking_initialized_ = true;
    return true;
}

void OpenXRManager::update_hand_tracking(XrTime time, XRDeviceState& state) {
    if (!hand_tracking_initialized_) return;

    for (int h = 0; h < 2; h++) {
        if (hand_trackers_[h] == XR_NULL_HANDLE) continue;

        auto& hand_data = state.hands[h];
        auto& joints = hand_data.joints;

        // Locate joints
        XrHandJointsLocateInfoEXT locate_info{XR_TYPE_HAND_JOINTS_LOCATE_INFO_EXT};
        locate_info.baseSpace = local_space_;
        locate_info.time = time;

        XrHandJointLocationsEXT locations{XR_TYPE_HAND_JOINT_LOCATIONS_EXT};
        XrHandJointLocationEXT joint_locations[XR_HAND_JOINT_COUNT_EXT];
        locations.jointCount = XR_HAND_JOINT_COUNT_EXT;
        locations.jointLocations = joint_locations;

        XrResult result = pfn_xrLocateHandJointsEXT_(hand_trackers_[h], &locate_info, &locations);
        
        if (XR_SUCCEEDED(result) && locations.isActive) {
            hand_data.is_active = true;
            
            for (uint32_t j = 0; j < XR_HAND_JOINT_COUNT_EXT; j++) {
                auto& jl = joint_locations[j];
                auto& joint = joints[j];
                
                joint.tracked = (jl.locationFlags & XR_SPACE_LOCATION_ORIENTATION_VALID_BIT) != 0;
                
                if (joint.tracked) {
                    joint.position = dvec3(jl.pose.position.x, jl.pose.position.y, jl.pose.position.z);
                    joint.orientation = dquat(jl.pose.orientation.w, jl.pose.orientation.x,
                                              jl.pose.orientation.y, jl.pose.orientation.z);
                    joint.radius = jl.radius;
                }
            }
        } else {
            hand_data.is_active = false;
        }
    }
}

// ==================== Frame Loop ====================

OpenXRManager::FrameResult OpenXRManager::begin_frame() {
    FrameResult result;

    // Wait for frame
    frame_wait_info_ = {XR_TYPE_FRAME_WAIT_INFO};
    XrResult wait_result = xrWaitFrame(session_, &frame_wait_info_, &frame_state_);
    if (XR_FAILED(wait_result) || !frame_state_.shouldRender) {
        return result; // Skip frame
    }

    // Begin frame
    frame_begin_info_ = {XR_TYPE_FRAME_BEGIN_INFO};
    xrBeginFrame(session_, &frame_begin_info_);

    // Get view configurations
    uint32_t view_count = 0;
    XrViewLocateInfo view_locate_info{XR_TYPE_VIEW_LOCATE_INFO};
    view_locate_info.viewConfigurationType = view_config_;
    view_locate_info.displayTime = frame_state_.predictedDisplayTime;
    view_locate_info.space = local_space_;

    XrViewState view_state{XR_TYPE_VIEW_STATE};
    std::array<XrView, 2> views;
    views[0] = {XR_TYPE_VIEW};
    views[1] = {XR_TYPE_VIEW};

    xrLocateViews(session_, &view_locate_info, &view_state, (uint32_t)views.size(), &view_count, views.data());

    // Acquire swapchain images
    for (uint32_t i = 0; i < view_count && i < (uint32_t)swapchains_.size(); i++) {
        XrSwapchainImageAcquireInfo acquire_info{XR_TYPE_SWAPCHAIN_IMAGE_ACQUIRE_INFO};
        uint32_t image_index = 0;
        xrAcquireSwapchainImage(swapchains_[i].handle, &acquire_info, &image_index);

        XrSwapchainImageWaitInfo wait_info{XR_TYPE_SWAPCHAIN_IMAGE_WAIT_INFO};
        wait_info.timeout = XR_INFINITE_DURATION;
        xrWaitSwapchainImage(swapchains_[i].handle, &wait_info);

        result.views[i] = {XR_TYPE_COMPOSITION_LAYER_PROJECTION_VIEW};
        result.views[i].pose = views[i].pose;
        result.views[i].fov = views[i].fov;
        result.views[i].subImage.swapchain = swapchains_[i].handle;
        result.views[i].subImage.imageRect.offset.x = 0;
        result.views[i].subImage.imageRect.offset.y = 0;
        result.views[i].subImage.imageRect.extent.width = swapchains_[i].width;
        result.views[i].subImage.imageRect.extent.height = swapchains_[i].height;
    }

    // Update HMD pose
    XrSpaceLocation hmd_location{XR_TYPE_SPACE_LOCATION};
    xrLocateSpace(local_space_, local_space_, frame_state_.predictedDisplayTime, &hmd_location);
    
    if (hmd_location.locationFlags & XR_SPACE_LOCATION_POSITION_VALID_BIT) {
        result.state.hmd_position = dvec3(
            hmd_location.pose.position.x,
            hmd_location.pose.position.y,
            hmd_location.pose.position.z
        );
        result.state.hmd_orientation = dquat(
            hmd_location.pose.orientation.w,
            hmd_location.pose.orientation.x,
            hmd_location.pose.orientation.y,
            hmd_location.pose.orientation.z
        );
    }

    // Update hand tracking
    update_hand_tracking(frame_state_.predictedDisplayTime, result.state);

    result.view_count = view_count;
    result.predicted_display_time = frame_state_.predictedDisplayTime;
    result.should_render = true;

    return result;
}

void OpenXRManager::end_frame(const std::array<XrCompositionLayerProjectionView, 2>& views,
                               uint32_t view_count) {
    // Release swapchain images
    for (uint32_t i = 0; i < view_count && i < (uint32_t)swapchains_.size(); i++) {
        XrSwapchainImageReleaseInfo release_info{XR_TYPE_SWAPCHAIN_IMAGE_RELEASE_INFO};
        xrReleaseSwapchainImage(swapchains_[i].handle, &release_info);
    }

    // Compose layers
    XrCompositionLayerProjection layer{XR_TYPE_COMPOSITION_LAYER_PROJECTION};
    layer.space = local_space_;
    layer.viewCount = view_count;
    layer.views = views.data();

    XrCompositionLayerBaseHeader* headers[] = {
        (XrCompositionLayerBaseHeader*)&layer
    };

    XrFrameEndInfo end_info{XR_TYPE_FRAME_END_INFO};
    end_info.displayTime = frame_state_.predictedDisplayTime;
    end_info.environmentBlendMode = passthrough_enabled_ ? 
        XR_ENVIRONMENT_BLEND_MODE_ADDITIVE : XR_ENVIRONMENT_BLEND_MODE_OPAQUE;
    end_info.layerCount = 1;
    end_info.layers = headers;

    xrEndFrame(session_, &end_info);
}

// ==================== Passthrough ====================

void OpenXRManager::enable_passthrough() {
    if (!pfn_xrCreatePassthroughFB_) return;
    if (passthrough_ != XR_NULL_HANDLE) return;

    XrPassthroughCreateInfoFB pt_info{XR_TYPE_PASSTHROUGH_CREATE_INFO_FB};
    pt_info.passthroughFlags = XR_PASSTHROUGH_IS_RUNNING_AT_CREATION_BIT_FB;
    pt_info.purpose = XR_PASSTHROUGH_LAYER_PURPOSE_RECONSTRUCTION_FB;

    XrResult result = pfn_xrCreatePassthroughFB_(session_, &pt_info, &passthrough_);
    if (XR_SUCCEEDED(result)) {
        passthrough_enabled_ = true;
        std::cout << "[OpenXR] Passthrough enabled" << std::endl;
    } else {
        std::cerr << "[OpenXR] Failed to enable passthrough: " << result << std::endl;
    }
}

void OpenXRManager::disable_passthrough() {
    if (passthrough_ != XR_NULL_HANDLE && pfn_xrPassthroughPauseFB_) {
        pfn_xrPassthroughPauseFB_(passthrough_);
        passthrough_enabled_ = false;
    }
}

// ==================== Haptics ====================

void OpenXRManager::trigger_haptic(int hand, float amplitude, float duration) {
    if (!pfn_xrApplyHapticFeedbackFB_) return;

    XrHapticActionInfoFB action_info{XR_TYPE_HAPTIC_ACTION_INFO_FB};
    action_info.hapticAction = nullptr; // In production: get from interaction profile

    XrHapticAmplitudeEnvelopeVibrationFB vibration{XR_TYPE_HAPTIC_AMPLITUDE_ENVELOPE_VIBRATION_FB};
    vibration.duration = (int64_t)(duration * 1e9); // Nanoseconds
    vibration.amplitude = amplitude;

    XrHapticPcmBufferFB pcm{XR_TYPE_HAPTIC_PCM_BUFFER_FB};
    pcm.buffer = nullptr; // Buffer not required for amplitude envelope
    
    pfn_xrApplyHapticFeedbackFB_(hand_trackers_[hand], &action_info, 
                                 (XrHapticBaseHeader*)&vibration);
}

bool OpenXRManager::begin_session() {
    XrSessionBeginInfo begin_info{XR_TYPE_SESSION_BEGIN_INFO};
    begin_info.primaryViewConfigurationType = view_config_;
    
    XrResult result = xrBeginSession(session_, &begin_info);
    if (XR_SUCCEEDED(result)) {
        session_running_ = true;
        std::cout << "[OpenXR] Session begun" << std::endl;
        return true;
    }
    return false;
}

void OpenXRManager::end_session() {
    if (session_running_) {
        xrEndSession(session_);
        session_running_ = false;
    }
}

} // namespace hlp
