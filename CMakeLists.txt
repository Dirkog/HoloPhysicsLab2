cmake_minimum_required(VERSION 3.20)

project(HoloPhysicsLab2
    VERSION 3.6.0
    DESCRIPTION "HoloPhysics Lab 2 — Научная песочница с Hand Tracking (16,923 строк, 75 файлов)"
    LANGUAGES CXX
)

# ==================== C++ Standard ====================
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# ==================== Options ====================
option(USE_VCPKG "Use vcpkg for dependencies" ON)
option(BUILD_TESTS "Build tests" ON)
option(USE_SANITIZERS "Enable address sanitizer" OFF)
option(BUILD_AR_MODULE "Build AR/Hand Tracking module" ON)
option(USE_OPENCV "Use OpenCV for camera capture" ON)
option(ENABLE_TRACY "Enable Tracy Profiler" OFF)

if(ENABLE_TRACY)
    add_definitions(-DENABLE_TRACY)
    find_package(Tracy CONFIG REQUIRED)
    target_link_libraries(${PROJECT_NAME} PRIVATE Tracy::TracyClient)
    message(STATUS "Tracy Profiler ENABLED")
endif()

# ==================== Dependencies ====================
if(USE_VCPKG)
    find_package(glfw3 CONFIG REQUIRED)
    find_package(glad CONFIG REQUIRED)
    find_package(glm CONFIG REQUIRED)
    find_package(imgui CONFIG REQUIRED)
    find_package(implot CONFIG REQUIRED)
    find_package(nlohmann_json CONFIG REQUIRED)
    
    if(BUILD_AR_MODULE AND USE_OPENCV)
        find_package(OpenCV QUIET COMPONENTS core imgproc videoio)
    endif()
else()
    find_package(glfw3 QUIET)
    if(NOT glfw3_FOUND)
        message(FATAL_ERROR "GLFW not found. Install: vcpkg install glfw3")
    endif()
    find_package(glad QUIET)
    if(NOT glad_FOUND)
        message(FATAL_ERROR "GLAD not found. Install: vcpkg install glad")
    endif()
    find_package(glm QUIET)
    if(NOT glm_FOUND)
        message(FATAL_ERROR "GLM not found. Install: vcpkg install glm")
    endif()
    find_package(imgui QUIET)
    if(NOT imgui_FOUND)
        message(FATAL_ERROR "ImGui not found. Install: vcpkg install imgui")
    endif()
    find_package(implot QUIET)
    if(NOT implot_FOUND)
        message(FATAL_ERROR "ImPlot not found. Install: vcpkg install implot")
    endif()
    find_package(nlohmann_json QUIET)
    if(NOT nlohmann_json_FOUND)
        message(FATAL_ERROR "nlohmann_json not found. Install: vcpkg install nlohmann-json")
    endif()
endif()

# ==================== Source files ====================

# Core physics engine (REQUIRED)
set(CORE_SOURCES
    src/core/Types.cpp
    src/core/Solver.cpp
    src/core/Scene.cpp
    src/core/Engine.cpp
)

# Physics modules (REQUIRED)
set(PHYSICS_SOURCES
    src/physics/CollisionSystem.cpp
    src/physics/ConstraintSolver.cpp
    src/physics/RotationalDynamics.cpp
    src/physics/AdaptiveTimestep.cpp
    src/physics/BoundaryConditions.cpp
)

# Rendering (REQUIRED)
set(RENDER_SOURCES
    src/render/Renderer.cpp
)

# AR Module (OPTIONAL - only if OpenCV is found)

# XR Module
set(XR_SOURCES
    src/xr/OpenXRManager.cpp
)

# ECS Module
set(ECS_SOURCES
    src/ecs/Entity.cpp
)

# Audio Module
set(AUDIO_SOURCES
    src/audio/AudioEngine.cpp
)

# Scripting Module
set(SCRIPT_SOURCES
    src/scripting/ScriptEngine.cpp
)


# ECS Systems
set(ECS_SYSTEMS
    src/ecs/systems/PhysicsSystems.cpp
)

# Event Queue
set(QUEUE_SOURCES
    src/physics/queue/LockFreeEventQueue.h
)

# Hand Sources
set(HAND_SOURCES
    src/input/sources/IHandSource.cpp
)

set(AR_SOURCES
    src/render/ARRenderer.cpp
    src/render/OcclusionRenderer.cpp
    src/render/MetaballOcclusion.cpp
    src/render/OptimizedMetaball.cpp
    src/render/CameraRelativeRenderer.cpp
    src/render/ShadowCatcher.cpp
    src/cv/CameraCapture.cpp
    src/cv/HandTracker.cpp
    src/cv/DepthCalibrator.cpp
    src/input/VirtualHand.cpp
    src/input/HermiteInterpolator.cpp
    src/input/PredictiveKalman.cpp
    src/interaction/HandInteraction.cpp
    src/physics/VirtualCoupling.cpp
    src/physics/CCD.cpp
    src/physics/MahalanobisGater.cpp
)

# UI (REQUIRED but simplified)
set(UI_SOURCES src/ui/AppUI.cpp)

# App integration (REQUIRED)
set(APP_SOURCES
    src/HoloPhysicsApp.cpp
)

# Headers
set(HEADERS
    src/HoloPhysicsApp.h
    src/core/Types.h
    src/core/Solver.h
    src/core/Scene.h
    src/core/Engine.h
    src/physics/CollisionSystem.h
    src/physics/ConstraintSolver.h
    src/physics/RotationalDynamics.h
    src/physics/AdaptiveTimestep.h
    src/physics/BoundaryConditions.h
    src/render/Renderer.h
    src/render/ARRenderer.h
    src/render/OcclusionRenderer.h
    src/render/MetaballOcclusion.h
    src/render/OptimizedMetaball.h
    src/render/CameraRelativeRenderer.h
    src/render/ShadowCatcher.h
    src/input/HermiteInterpolator.h
    src/physics/CCD.h
    src/physics/MahalanobisGater.h
    src/xr/OpenXRManager.h
    src/ecs/Entity.h
    src/ecs/core/Registry.h
    src/ecs/systems/PhysicsSystems.h
    src/input/sources/IHandSource.h
    src/physics/queue/LockFreeEventQueue.h
    src/audio/AudioEngine.h
    src/scripting/ScriptEngine.h
    src/cv/CameraCapture.h
    src/cv/HandTracker.h
    src/cv/DepthCalibrator.h
    src/input/VirtualHand.h
    src/input/PredictiveKalman.h
    src/interaction/HandInteraction.h
    src/physics/VirtualCoupling.h
    src/ui/AppUI.h
)

# ==================== Main executable ====================
add_executable(${PROJECT_NAME}
    src/main.cpp
    ${APP_SOURCES}
    ${CORE_SOURCES}
    ${PHYSICS_SOURCES}
    ${RENDER_SOURCES}
    ${AR_SOURCES}
    ${XR_SOURCES}
    ${ECS_SOURCES}
    ${ECS_SYSTEMS}
    ${QUEUE_SOURCES}
    ${AUDIO_SOURCES}
    ${SCRIPT_SOURCES}
    ${HAND_SOURCES}
    ${UI_SOURCES}
    ${HEADERS}
    ${SHADERS}
)

target_include_directories(${PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)

target_link_libraries(${PROJECT_NAME} PRIVATE
    glfw
    glad::glad
    glm::glm
    imgui::imgui
    implot::implot
    nlohmann_json::nlohmann_json
    openxr_loader
    lua5.4
    miniaudio
)

# Add OpenCV if available
if(BUILD_AR_MODULE AND OpenCV_FOUND)
    target_link_libraries(${PROJECT_NAME} PRIVATE ${OpenCV_LIBS})
    target_compile_definitions(${PROJECT_NAME} PRIVATE HOLO_HAVE_OPENCV)
    message(STATUS "OpenCV found — AR/Hand Tracking module ENABLED")
else()
    message(STATUS "OpenCV not found — AR/Hand Tracking DISABLED (simulation only)")
endif()

# ==================== Compiler options ====================
if(MSVC)
    target_compile_options(${PROJECT_NAME} PRIVATE
        /W4 /utf-8 /Oi /Oy- /arch:AVX2 /fp:precise /EHsc
    )
    target_compile_options(${PROJECT_NAME} PRIVATE
        $<$<CONFIG:Release>:/O2 /GL /GS- /Ob2>
    )
    target_link_options(${PROJECT_NAME} PRIVATE
        $<$<CONFIG:Release>:/LTCG /INCREMENTAL:NO>
    )
else()
    target_compile_options(${PROJECT_NAME} PRIVATE
        -Wall -Wextra -Wpedantic -O2 -march=native
    )
endif()

if(USE_SANITIZERS AND NOT MSVC)
    target_compile_options(${PROJECT_NAME} PRIVATE -fsanitize=address)
    target_link_options(${PROJECT_NAME} PRIVATE -fsanitize=address)
endif()

# ==================== Install ====================
install(TARGETS ${PROJECT_NAME} RUNTIME DESTINATION bin)

# ==================== Tests ====================
if(BUILD_TESTS)
    enable_testing()
    
    add_library(physics_core STATIC
        ${CORE_SOURCES}
        ${PHYSICS_SOURCES}
    )
    target_include_directories(physics_core PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/src)
    target_link_libraries(physics_core PUBLIC glm::glm nlohmann_json::nlohmann_json)
    
    add_executable(physics_tests
        tests/test_physics.cpp
        tests/test_collisions.cpp
        tests/test_integrators.cpp
    )
    target_link_libraries(physics_tests PRIVATE physics_core)
    target_include_directories(physics_tests PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/tests)
    add_test(NAME PhysicsTests COMMAND physics_tests)
    
    # SPSC Queue Stress Test
    add_executable(spsc_queue_test
        tests/test_spsc_queue.cpp
    )
    target_include_directories(spsc_queue_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
    target_link_libraries(spsc_queue_test PRIVATE)
    add_test(NAME SPSCQueueTest COMMAND spsc_queue_test)
endif()

# ==================== vcpkg ====================
if(USE_VCPKG AND NOT DEFINED CMAKE_TOOLCHAIN_FILE)
    set(CMAKE_TOOLCHAIN_FILE 
        "${CMAKE_CURRENT_SOURCE_DIR}/vcpkg/scripts/buildsystems/vcpkg.cmake"
        CACHE STRING "vcpkg toolchain file"
    )
endif()

# ==================== Post-build ====================
message(STATUS "============================================")
message(STATUS "HoloPhysics Lab 2 - Build Configuration")
message(STATUS "C++ Standard: ${CMAKE_CXX_STANDARD}")
message(STATUS "Core: ${CORE_SOURCES}")
message(STATUS "Physics: ${PHYSICS_SOURCES}")
message(STATUS "AR/Hand: ${AR_SOURCES}")
message(STATUS "Tests: ${BUILD_TESTS}")
message(STATUS "============================================")
message(STATUS "")
message(STATUS "Build:")
message(STATUS "  mkdir build && cd build")
message(STATUS "  cmake .. -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTS=ON")
message(STATUS "  cmake --build . --config Release")
message(STATUS "  ctest --output-on-failure")
message(STATUS "")

add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_directory
        "${CMAKE_SOURCE_DIR}/assets" 
        "$<TARGET_FILE_DIR:${PROJECT_NAME}>/assets"
    COMMENT "Copying assets..."
)
