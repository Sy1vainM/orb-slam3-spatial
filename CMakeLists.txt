cmake_minimum_required(VERSION 3.15)
project(orb_slam3_spatial LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

add_compile_options(-Wall -O3)
if(CMAKE_BUILD_TYPE STREQUAL "Release")
    add_compile_options(-march=native)
endif()
add_definitions(-DCOMPILEDWITHC11)

# ---------------------------------------------------------------------------
# Locate ORB-SLAM3 submodule (must be patched before this CMake runs — see
# scripts/apply-patches.sh). We do NOT pull the submodule's CMakeLists via
# add_subdirectory because that would also build every example binary we are
# replacing. Instead we consume its pre-built library and headers.
# ---------------------------------------------------------------------------
set(ORB_SLAM3_DIR ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/ORB_SLAM3)

if(NOT EXISTS ${ORB_SLAM3_DIR}/CMakeLists.txt)
    message(FATAL_ERROR
        "thirdparty/ORB_SLAM3 is not initialized. Run:\n"
        "  git submodule update --init --recursive\n"
        "  scripts/apply-patches.sh\n"
        "  scripts/build-orb-slam3.sh")
endif()

# Shared library produced by the submodule's own cmake run
find_library(ORB_SLAM3_LIB ORB_SLAM3
    PATHS ${ORB_SLAM3_DIR}/lib
    NO_DEFAULT_PATH)
if(NOT ORB_SLAM3_LIB)
    message(FATAL_ERROR
        "libORB_SLAM3 not found at ${ORB_SLAM3_DIR}/lib. "
        "Run scripts/build-orb-slam3.sh first.")
endif()
message(STATUS "Found libORB_SLAM3: ${ORB_SLAM3_LIB}")

# ---------------------------------------------------------------------------
# External dependencies (same set ORB-SLAM3 itself requires)
# ---------------------------------------------------------------------------
find_package(OpenCV 4 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Pangolin REQUIRED)
find_package(Boost REQUIRED COMPONENTS serialization)
find_package(OpenSSL REQUIRED)

message(STATUS "OpenCV ${OpenCV_VERSION}")

# ---------------------------------------------------------------------------
# Wrapper library: SocketPublisher (the UDS server previously embedded in
# ORB-SLAM3::System). Lives outside the submodule; links against ORB_SLAM3
# headers only.
# ---------------------------------------------------------------------------
add_library(spatial_publisher STATIC
    src/socket_publisher.cc
)
target_include_directories(spatial_publisher PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${ORB_SLAM3_DIR}
    ${ORB_SLAM3_DIR}/include
    ${ORB_SLAM3_DIR}/include/CameraModels
    ${ORB_SLAM3_DIR}/Thirdparty/Sophus
    ${OpenCV_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
)
target_link_libraries(spatial_publisher PUBLIC
    ${ORB_SLAM3_LIB}
    ${OpenCV_LIBS}
    Boost::serialization
)

# ---------------------------------------------------------------------------
# Executable: mono_tum (replaces ORB_SLAM3/Examples/Monocular/mono_tum)
# ---------------------------------------------------------------------------
add_executable(mono_tum src/main_tum.cc)
target_include_directories(mono_tum PRIVATE
    ${ORB_SLAM3_DIR}/include
    ${ORB_SLAM3_DIR}/include/CameraModels
    ${ORB_SLAM3_DIR}/Thirdparty/Sophus
    ${OpenCV_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    ${Pangolin_INCLUDE_DIRS}
)
target_link_libraries(mono_tum PRIVATE
    spatial_publisher
    ${ORB_SLAM3_LIB}
    ${OpenCV_LIBS}
    ${Pangolin_LIBRARIES}
    Boost::serialization
    OpenSSL::Crypto
)

install(TARGETS mono_tum RUNTIME DESTINATION Examples/Monocular)
install(FILES
    config/TUM1.yaml config/TUM2.yaml config/TUM3.yaml
    config/TUM1_tuned.yaml config/Generic720.yaml
    DESTINATION Examples/Monocular)
install(FILES Vocabulary/ORBvoc.txt.bin DESTINATION Vocabulary)
