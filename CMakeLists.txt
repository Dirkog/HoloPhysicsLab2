cmake_minimum_required(VERSION 3.20)

project(HoloPhysicsLab2
    VERSION 3.6.0
    DESCRIPTION "HoloPhysics Lab 2 — Enterprise AR Physics Sandbox (17,293 строк)"
    LANGUAGES CXX
)

# ==================== C++ Standard ====================
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# ==================== Options ====================
option(BUILD_TESTS "Build tests" ON)
option(USE_OPENCV "Use OpenCV for camera capture" OFF)
option(ENABLE_TRACY "Enable Tracy Profiler" OFF)

# ==================== Dependencies (vcpkg or system) ====================
find_package(glfw3 CONFIG REQUIRED)
find_package(glad CONFIG REQUIRED)
find_package(glm CONFIG REQUIRED)
find_package(imgui CONFIG REQUIRED)
find_package(implot CONFIG REQUIRED)
find_package(nlohmann_json CONFIG REQUIRED)

# Optional modules
find_package(OpenXR CONFIG QUIET)
find_package(Lua CONFIG QUIET)
find_package(miniaudio CONFIG QUIET)
if(USE_OPENCV)
    find_package(OpenCV QUIET COMPONENTS core imgproc videoio)
endif()

# ==================== Main executable ====================
list(APPEND ALL_SOURCES
    src/main.cpp
    src/HoloPhysicsApp.cpp
    src/core/Types.cpp
    src/core/Solver.cpp
    src/core/Scene.cpp
    src/core/Engine.cpp
    src/physics/CollisionSystem.cpp
    src/physics/ConstraintSolver.cpp
    src/physics/RotationalDynamics.cpp
    src/physics/AdaptiveTimestep.cpp
    src/physics/BoundaryConditions.cpp
    src/physics/VirtualCoupling.cpp
    src/physics/CCD.cpp
    src/physics/MahalanobisGater.cpp
    src/render/Renderer.cpp
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
    src/input/sources/IHandSource.cpp
    src/interaction/HandInteraction.cpp
    src/xr/OpenXRManager.cpp
    src/ecs/Entity.cpp
    src/ecs/systems/PhysicsSystems.cpp
    src/audio/AudioEngine.cpp
    src/scripting/ScriptEngine.cpp
    src/ui/AppUI.cpp
)

add_executable(${PROJECT_NAME} ${ALL_SOURCES})

target_include_directories(${PROJECT_NAME} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)

# Core libraries
target_link_libraries(${PROJECT_NAME} PRIVATE
    glfw glad::glad glm::glm
    imgui::imgui implot::implot
    nlohmann_json::nlohmann_json
)

# Optional: OpenXR
if(OpenXR_FOUND)
    target_link_libraries(${PROJECT_NAME} PRIVATE OpenXR::openxr_loader)
    target_compile_definitions(${PROJECT_NAME} PRIVATE HOLO_HAVE_OPENXR)
    message(STATUS "✅ OpenXR module ENABLED")
endif()

# Optional: Lua scripting
if(Lua_FOUND)
    target_link_libraries(${PROJECT_NAME} PRIVATE Lua::Lua)
    target_compile_definitions(${PROJECT_NAME} PRIVATE HOLO_HAVE_LUA)
    message(STATUS "✅ Lua scripting ENABLED")
endif()

# Optional: Audio
if(miniaudio_FOUND)
    target_link_libraries(${PROJECT_NAME} PRIVATE miniaudio::miniaudio)
    target_compile_definitions(${PROJECT_NAME} PRIVATE HOLO_HAVE_AUDIO)
    message(STATUS "✅ Audio module ENABLED")
else()
    # miniaudio is header-only, include directly
    target_compile_definitions(${PROJECT_NAME} PRIVATE MINIAUDIO_IMPLEMENTATION)
    message(STATUS "ℹ️  miniaudio: using header-only fallback")
endif()

# Optional: OpenCV
if(USE_OPENCV AND OpenCV_FOUND)
    target_link_libraries(${PROJECT_NAME} PRIVATE ${OpenCV_LIBS})
    target_compile_definitions(${PROJECT_NAME} PRIVATE HOLO_HAVE_OPENCV)
    message(STATUS "✅ OpenCV module ENABLED")
else()
    message(STATUS "ℹ️  OpenCV: DISABLED (hand simulation mode)")
endif()

# ==================== Tests ====================
if(BUILD_TESTS)
    enable_testing()

    # Common test dependencies
    set(TEST_DEPS glm::glm nlohmann_json::nlohmann_json)

    add_executable(physics_tests
        tests/test_physics.cpp
        tests/test_collisions.cpp
        tests/test_integrators.cpp
    )
    target_include_directories(physics_tests PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src
        ${CMAKE_CURRENT_SOURCE_DIR}/tests
    )
    target_link_libraries(physics_tests PRIVATE ${TEST_DEPS})
    add_test(NAME PhysicsTests COMMAND physics_tests)

    add_executable(spsc_queue_test tests/test_spsc_queue.cpp)
    target_include_directories(spsc_queue_test PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}/src
    )
    target_link_libraries(spsc_queue_test PRIVATE ${TEST_DEPS})
    add_test(NAME SPSCQueueTest COMMAND spsc_queue_test)
endif()

# ==================== Compiler flags ====================
if(MSVC)
    target_compile_options(${PROJECT_NAME} PRIVATE /W4 /utf-8 /Oi /arch:AVX2 /fp:precise /EHsc)
    target_compile_options(${PROJECT_NAME} PRIVATE $<$<CONFIG:Release>:/O2 /GL /GS- /Ob2>)
    target_link_options(${PROJECT_NAME} PRIVATE $<$<CONFIG:Release>:/LTCG /INCREMENTAL:NO>)

    if(BUILD_TESTS)
        target_compile_options(physics_tests PRIVATE /W4 /utf-8 /EHsc)
        target_compile_options(spsc_queue_test PRIVATE /W4 /utf-8 /EHsc)
    endif()
else()
    target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Wextra -O2 -march=native)
endif()

# ==================== vcpkg manifest fallback ====================
if(NOT DEFINED CMAKE_TOOLCHAIN_FILE)
    if(DEFINED ENV{VCPKG_ROOT})
        set(CMAKE_TOOLCHAIN_FILE "$ENV{VCPKG_ROOT}/scripts/buildsystems/vcpkg.cmake"
            CACHE STRING "vcpkg toolchain file")
        message(STATUS "ℹ️  Using vcpkg from VCPKG_ROOT: $ENV{VCPKG_ROOT}")
    else()
        message(STATUS "ℹ️  No vcpkg toolchain set — assuming system packages")
    endif()
endif()

message(STATUS "============================================")
message(STATUS "HoloPhysics Lab 2 v3.6")
message(STATUS "Build: cmake -B build -DBUILD_TESTS=ON")
message(STATUS "       cmake --build build --config Release --parallel")
message(STATUS "       cd build && ctest")
message(STATUS "============================================")
