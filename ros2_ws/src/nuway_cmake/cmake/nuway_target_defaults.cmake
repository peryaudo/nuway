# nuway_target_defaults(<target>)
#
# Applies the project-wide compiler configuration to one target
# (docs/03_style_and_conventions.md §6.2). Packages call this on every library
# and executable instead of setting flags themselves.
#
# Cache options (pass with -DNAME=VALUE through `colcon build --cmake-args`):
#   NUWAY_WERROR      ON|OFF   warnings are errors (default ON)
#   NUWAY_CLANG_TIDY  ON|OFF   run clang-tidy as part of the compile (default OFF)
#   NUWAY_SANITIZE    none|address,undefined|thread   (default none)
#   NUWAY_LTO         ON|OFF   interprocedural optimization (default OFF)

option(NUWAY_WERROR "Treat warnings as errors" ON)
option(NUWAY_CLANG_TIDY "Run clang-tidy during the build" OFF)
option(NUWAY_LTO "Enable link-time optimization" OFF)
set(NUWAY_SANITIZE
    "none"
    CACHE STRING
    "Sanitizers: none | address,undefined | thread"
)

set(_nuway_warning_flags
    -Wall
    -Wextra
    -Wpedantic
    -Wshadow
    -Wnon-virtual-dtor
    -Wold-style-cast
    -Woverloaded-virtual
    -Wnull-dereference
    -Wdouble-promotion
    -Wformat=2
)

if(NUWAY_CLANG_TIDY)
    # Prefer the uv-managed wheel (pinned in uv.lock) over whatever apt ships.
    if(DEFINED ENV{VIRTUAL_ENV} AND EXISTS "$ENV{VIRTUAL_ENV}/bin/clang-tidy")
        set(NUWAY_CLANG_TIDY_EXE "$ENV{VIRTUAL_ENV}/bin/clang-tidy")
    else()
        find_program(NUWAY_CLANG_TIDY_EXE NAMES clang-tidy)
    endif()
    if(NOT NUWAY_CLANG_TIDY_EXE)
        message(
            FATAL_ERROR
            "NUWAY_CLANG_TIDY=ON but no clang-tidy found (source setup_env.sh)"
        )
    endif()
endif()

function(nuway_target_defaults target)
    set_target_properties(
        ${target}
        PROPERTIES CXX_STANDARD 17 CXX_STANDARD_REQUIRED ON CXX_EXTENSIONS OFF
    )
    target_compile_options(${target} PRIVATE ${_nuway_warning_flags})
    if(NUWAY_WERROR)
        target_compile_options(${target} PRIVATE -Werror)
    endif()
    if(NOT NUWAY_SANITIZE STREQUAL "none")
        target_compile_options(
            ${target}
            PRIVATE -fsanitize=${NUWAY_SANITIZE} -fno-omit-frame-pointer
        )
        target_link_options(${target} PRIVATE -fsanitize=${NUWAY_SANITIZE})
    endif()
    if(NUWAY_LTO)
        set_target_properties(
            ${target}
            PROPERTIES INTERPROCEDURAL_OPTIMIZATION ON
        )
    endif()
    if(NUWAY_CLANG_TIDY)
        set_target_properties(
            ${target}
            PROPERTIES CXX_CLANG_TIDY "${NUWAY_CLANG_TIDY_EXE}"
        )
    endif()
endfunction()
