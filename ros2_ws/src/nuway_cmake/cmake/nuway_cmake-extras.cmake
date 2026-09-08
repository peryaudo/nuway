# Loaded by find_package(nuway_cmake); makes nuway_target_defaults() available.

# The build defaults of ros2_ws/colcon_defaults.yaml, re-applied: a
# `colcon build --cmake-args ...` on the command line *replaces* that whole
# list (verified: the CI tidy and sanitizer builds got an empty build type
# and no compile database), so anything a `-DNUWAY_*` build must keep is set
# here unless the caller chose otherwise. The generator cannot be chosen from
# CMake; such a build falls back to Make (correct, slower).
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE
        RelWithDebInfo
        CACHE STRING
        "Build type (nuway default)"
        FORCE
    )
endif()
# CMake pre-creates this cache entry empty, so test the value, not DEFINED.
if("${CMAKE_EXPORT_COMPILE_COMMANDS}" STREQUAL "")
    set(CMAKE_EXPORT_COMPILE_COMMANDS
        ON
        CACHE BOOL
        "Write compile_commands.json (nuway default)"
        FORCE
    )
endif()
if(NOT CMAKE_CXX_COMPILER_LAUNCHER)
    find_program(NUWAY_CCACHE_EXE ccache)
    if(NUWAY_CCACHE_EXE)
        set(CMAKE_CXX_COMPILER_LAUNCHER "${NUWAY_CCACHE_EXE}")
    endif()
endif()
if(NOT CMAKE_EXE_LINKER_FLAGS MATCHES "-fuse-ld=")
    find_program(NUWAY_MOLD_EXE mold)
    if(NUWAY_MOLD_EXE)
        string(APPEND CMAKE_EXE_LINKER_FLAGS " -fuse-ld=mold")
        string(APPEND CMAKE_SHARED_LINKER_FLAGS " -fuse-ld=mold")
        string(APPEND CMAKE_MODULE_LINKER_FLAGS " -fuse-ld=mold")
    endif()
endif()

include("${nuway_cmake_DIR}/nuway_target_defaults.cmake")
