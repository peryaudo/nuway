# Style and coding conventions

**Status:** planning document, v1 (2026-09-05)
**Audience:** Claude Code and human contributors. Read after `02_interfaces.md`. Applies to every milestone.

---

## 1. Summary

- **C++ follows the [Google C++ Style Guide](https://google.github.io/styleguide/cppguide.html).** It is the authoritative reference. This document only records project-specific decisions, the few deliberate deviations (§3), and how the style is enforced.
- **Formatting is not a matter of taste.** `clang-format` with the repo `.clang-format` (`BasedOnStyle: Google`) is the single source of truth. Code that is not clang-format-clean does not merge.
- **`clang-tidy` is mandatory.** Every C++ package builds with the repo `.clang-tidy`; warnings are errors in CI. Naming rules from the Google guide are machine-checked through `readability-identifier-naming`.
- **Python follows PEP 8 and PEP 257**, enforced by `ruff` (formatter and linter) and `mypy` with the repo `pyproject.toml` (§9). Same rule as C++: code that is not `ruff format`-clean, `ruff check`-clean and `mypy`-clean does not merge.
- **One toolchain, pinned, reproducible.** Python environments and every developer tool (ruff, mypy, pytest, pre-commit, clang-format, clang-tidy, gersemi) are managed by `uv` from the root `pyproject.toml` + `uv.lock`. C++ builds use colcon with Ninja, ccache and mold configured once in `ros2_ws/colcon_defaults.yaml`, and a shared `nuway_cmake` package for warnings, sanitizers and clang-tidy (§6). No `pip`, no `conda`, no hand-installed LLVM, no `make`.
- **Run the tools before every commit** (§7.4). Claude Code must run them after every change, in the same way it runs the build and the tests.

Precedence when rules conflict: formatter output (`clang-format`, `ruff format`) > linter diagnostics (`clang-tidy`, `ruff check`, `mypy`) > this document > the upstream style guide (Google C++ Style Guide, PEP 8/257) > ROS 2 community conventions. If a tool and this document disagree, fix the tool configuration *and* this document in the same commit, never suppress the tool locally.

---

## 2. Google C++ style: the rules that matter most here

This is not a restatement of the guide. It is the subset that a contributor coming from typical ROS 2 code (which uses a different style) is most likely to get wrong.

### 2.1 Naming

| Entity | Rule | Example |
|--------|------|---------|
| Namespaces | `lower_snake_case`, one per ROS package, same name as the package | `namespace nuway_control {` |
| Types (class, struct, enum, alias, template param) | `CamelCase` | `class LongitudinalMap`, `struct GridSpec`, `using PointList = std::vector<Point>` |
| Functions and methods | `CamelCase`, verbs | `Sample()`, `ToFrenet()`, `PublishDiag()` |
| Cheap accessors / mutators | `lower_snake_case`, named after the member | `resolution()`, `set_resolution()` |
| Local variables, parameters | `lower_snake_case` | `lateral_error`, `dt` |
| Struct data members (plain aggregates) | `lower_snake_case`, no suffix | `spec.resolution` |
| Class data members | `lower_snake_case_` with trailing underscore | `lookahead_m_`, `pub_cmd_` |
| Constants (`constexpr`, `const` at namespace/class scope) | `kCamelCase` | `constexpr double kMaxSteerRad = 1.22;` |
| Enumerators | `kCamelCase` (never `UPPER_CASE`) | `enum class LaneType { kDriving, kShoulder };` |
| Macros | `UPPER_SNAKE_CASE`; avoid macros entirely | `NUWAY_COMMON_GEOMETRY_HPP_` (include guards only) |
| Files | `lower_snake_case`; `.hpp` headers, `.cpp` sources | `pure_pursuit_pid_node.cpp`, `frenet.hpp` |
| Test files | `<unit>_test.cpp` next to the unit under `test/` | `test/frenet_test.cpp` |

Extra project rules:

- Node classes end in `Node`: `PurePursuitPidNode`, `MapServerNode`. The library class the node wraps does not: `PurePursuit`, `LaneGraph`.
- ROS parameter names are `lower_snake_case` and identical to the YAML keys in `configs/` (see `02_interfaces.md`). The C++ member holding a parameter has the same name with a trailing underscore: parameter `lookahead_m` → member `lookahead_m_`.
- Units in names when the unit is not obvious from the type: `_m`, `_mps`, `_rad`, `_s`, `_hz`. `double speed_mps`, not `double speed`.
- Topic and frame strings are `constexpr` constants (`kTopicEgoOdom`, `kFrameBaseLink`) declared in one header per package, never string literals scattered in code.
- Enumerators for message field values mirror the `nuway_msgs` constant names in `CamelCase` form (message `SPEED_LIMIT` ↔ enumerator `kSpeedLimit`).

### 2.2 Headers

- Every header has a `#define` guard of the form `NUWAY_<PACKAGE>_<PATH>_<FILE>_HPP_`, e.g. `NUWAY_COMMON_FRENET_HPP_`. `#pragma once` is not used (Google guide, and `clang-tidy` `llvm-header-guard` is configured to check the pattern).
- Headers are self-contained: include what you use, nothing more. No transitive-include reliance.
- Include order (enforced by `clang-format`):
  1. the header matching this `.cpp` file,
  2. C system headers (`<sys/...>`, `<cmath>` is C++),
  3. C++ standard library,
  4. third-party libraries (`<Eigen/...>`, `<gtsam/...>`, `<rclcpp/...>`, `<nuway_msgs/...>`),
  5. headers from other `nuway_*` packages,
  6. headers from the same package.
  Each group separated by a blank line, alphabetically sorted within the group.
- Include style: `"nuway_common/frenet.hpp"` with quotes for headers in the same package; angle brackets for everything installed by another package (ROS, Eigen, other `nuway_*` packages, generated `nuway_msgs`).
- Forward-declare instead of including when the header only needs a pointer or reference to the type.
- No `using namespace` anywhere, not even in `.cpp` files. `using std::chrono_literals::operator""ms;`-style targeted using-declarations are fine inside function scope. Namespace aliases (`namespace nm = nuway_msgs::msg;`) are allowed at file scope in `.cpp` files.

### 2.3 Classes and structs

- `struct` only for passive data aggregates with public members and no invariants (`GridSpec`, `TrajectoryPoint`). Anything with behavior or invariants is a `class` with private members.
- Declaration order inside a class: `public:`, `protected:`, `private:`; within each section: types and aliases, constants, factory functions, constructors/destructor, methods, then data members last.
- Mark single-argument constructors `explicit`. Mark overriding methods `override` (never `virtual` + `override`). Mark classes not meant to be derived from `final`.
- Prefer composition to inheritance. Inheritance is for the interfaces we define (e.g. a `Planner` interface with `virtual ~Planner() = default;`) and for `rclcpp::Node`.
- Copy/move: follow the rule of zero. If a class manages a resource, delete or explicitly define all five special members.
- No implicit conversions; no operator overloading except for value types where it is obvious (`SE2 operator*`, `operator==` on message-like structs).

### 2.4 Functions

- Small. A function that does not fit on one screen is a candidate for splitting.
- Inputs first, then outputs. Inputs are values or `const T&`. Outputs are return values, `std::optional<T>`, or a struct; avoid output parameters. Never non-const reference parameters for outputs, use a pointer (`T*`) if an in/out parameter is unavoidable so the mutation is visible at the call site.
- Return `std::optional<T>` for "may not produce a value" (e.g. `std::optional<FrenetPoint> ToFrenet(...)` when projection fails). Return a `bool` plus `T*` only when the payload is huge and must be filled in place.
- Default arguments only on non-virtual functions and only when the default is obvious. Prefer overloads when in doubt.
- Trailing return types only for lambdas where required.

### 2.5 Ownership and memory

- `std::unique_ptr` for owned polymorphic objects. `std::shared_ptr` only where `rclcpp` forces it (nodes, publishers, subscriptions, received messages). Never `new`/`delete` in project code; `std::make_unique` / `std::make_shared`.
- Raw pointers are non-owning and may be null unless documented otherwise. Prefer references when null is impossible.
- Pass `const std::shared_ptr<const Msg>&` (or `Msg::ConstSharedPtr`) in subscription callbacks; do not copy messages containing point clouds or grids.
- Eigen fixed-size vectorizable types (`Vector2d`, `Matrix4d`, …) inside `std::vector` need `Eigen::aligned_allocator`; use the aliases in `nuway_common/geometry.hpp` instead of spelling it out.

### 2.6 Language features

- C++17 exactly. No compiler extensions, no `-std=gnu++17`. `ament_cmake` packages set `CMAKE_CXX_STANDARD 17` and `CMAKE_CXX_EXTENSIONS OFF`.
- **No exceptions in project code.** Library classes never `throw`. Failures are reported through return values (`std::optional`, `bool`, or a small result struct with an `enum class` error code). Third-party code (`rclcpp` parameter APIs, GTSAM, OpenDRIVE parsing) may throw; catch at the node boundary (constructor or callback), log with `RCLCPP_ERROR`, and report through `/nuway/diag/<node>`. Never let an exception escape a callback.
- No RTTI-dependent logic (`dynamic_cast`, `typeid`). Use virtual functions or `std::variant`.
- `auto` only when the type is obvious from the right-hand side (`auto pub = create_publisher<...>()`, iterators, `make_unique`). Spell out numeric and Eigen types.
- Use `constexpr` for compile-time constants, `const` for everything that does not change, including local variables and member functions.
- Casts: `static_cast<>` only. No C-style casts. `reinterpret_cast` only in `carla_conv.hpp` or shared-memory code, with a comment.
- Integer types: `int` for counts and indices unless interop requires otherwise; `size_t` only where the standard library hands it to you; `int64_t`/`uint32_t` fixed widths for wire formats and message fields. Never unsigned for "non-negative" semantics.
- `enum class` always. Plain `enum` never.
- Lambdas: capture explicitly (`[this]`, `[&x]`), never `[=]` or `[&]` in callbacks stored past the current scope.
- No `goto`, no `#define` constants, no variable-length arrays, no `std::bind` (use lambdas).
- Braces on every `if`/`for`/`while` body, even single-line. (Deviation from Google, which permits omission; we forbid it. Enforced by `readability-braces-around-statements`.)

### 2.7 Comments

- `//` comments only. No `/* */` blocks except for the license/file header if one is ever added.
- Every header starts with a one-paragraph comment stating what the file provides and which milestone introduced it. Every non-trivial class and every public function has a comment describing *what* it does and its contract (units, frames, preconditions, failure behavior), not *how*.
- Doxygen markup is not used. Plain sentences, full stops.
- `TODO(username): text` or `TODO(M4): text` for work deferred to a milestone. No bare `TODO`.
- Comment frames and units at every boundary: `// Pose of base_link in map frame, ROS convention (x forward, y left).`

### 2.8 Formatting

Fully delegated to `clang-format`. For orientation, the Google base style means:

- 2-space indentation, no tabs. 80-column limit.
- Attach braces (`class Foo {`, `if (x) {`). `else` on the same line as the closing brace.
- Pointer/reference binds to the type: `const Foo& foo`, `Foo* foo`.
- Namespace contents are not indented; closing braces are commented: `}  // namespace nuway_control`.
- Two spaces before a trailing `//` comment.
- Access specifiers indented one space (` public:`).

Never hand-format. Never add `// clang-format off` except around large literal tables (lookup tables, test fixtures) and always paired with `// clang-format on`.

---

## 3. Deliberate deviations from the Google C++ Style Guide

Kept intentionally short. Anything not listed here follows Google.

| Topic | Google | nuway | Why |
|-------|--------|-------|-----|
| File extensions | `.h` / `.cc` | `.hpp` / `.cpp` | ROS 2 / `ament_cmake` ecosystem convention; `02_interfaces.md` and the directory structure already use it. |
| Include guard macro | project-path based | `NUWAY_<PKG>_<FILE>_HPP_` | Package name instead of full repo path so guards do not change when `ros2_ws/src/` moves. |
| Braces on single-statement bodies | optional | mandatory | Avoids a class of merge and diff bugs; enforced by clang-tidy. |
| `std::shared_ptr` | discouraged | allowed where `rclcpp` requires it | No alternative in ROS 2. |
| Abseil | recommended in places | not used | Keep the dependency set to what `00_overview.md` §4 lists. Use `std::optional`, `std::string_view`, `std::variant`. |
| Streams | discouraged | allowed only in tests and CLI tools | Never in nodes or libraries; use `RCLCPP_*` logging with `%`-style formatting or `fmt` if it is added later. |

---

## 4. ROS 2-specific conventions

- **Nodes are thin.** A node file (`*_node.cpp`) contains one class deriving from `rclcpp::Node`, its `main()`, and nothing else. It declares parameters, creates publishers/subscriptions/timers, converts messages to plain structs, and calls a library class that has **no `rclcpp` dependency** and is unit-tested with gtest.
- Parameters are declared in the constructor via `nuway_common/params.hpp` helpers with an explicit default and a one-line description. Read once at startup; dynamic reconfigure only where a milestone doc asks for it.
- One publisher/subscription member per topic, named `pub_<what>_` / `sub_<what>_`, e.g. `pub_cmd_`, `sub_ego_odom_`. Timers `timer_<what>_`.
- Callbacks are private methods named `On<Message>()` (`OnEgoOdom(...)`) or `OnTimer()`. They must not block; anything above a few milliseconds goes into the library class and is timed with `nuway_common/diag.hpp`'s scoped timer.
- QoS is set explicitly at every publisher/subscription using the profiles listed in `02_interfaces.md`. No implicit defaults.
- Logging: `RCLCPP_INFO` once at startup with the resolved parameters; `RCLCPP_WARN_THROTTLE` for recurring conditions; `RCLCPP_ERROR` for conditions that also flip the `NodeDiag` status. No `std::cout` / `printf` in nodes or libraries.
- Frame handling: every function that takes or returns a pose documents the frame in its comment (§2.7). Conversions between CARLA and ROS conventions only in `carla_conv.hpp` and `nuway_carla_bridge` (see `01_directory_structure.md` rules).
- No environment variable reads in nodes. Everything comes from parameters.

---

## 5. Tests

- Framework: gtest via `ament_cmake_gtest`. Pure library tests must link only the library, never `rclcpp`.
- Location: `<package>/test/<unit>_test.cpp`, one test file per header under test.
- Test names: `TEST(FrenetTest, RoundTripOnArcWithinTolerance)`; fixture classes end in `Test`. The test name reads as a sentence describing the expected behavior.
- Numerical tests state tolerances explicitly (`EXPECT_NEAR(x, y, 1e-6)`) and the tolerance is justified in a comment when it is not obvious.
- Tests are subject to the same `clang-format` and `clang-tidy` rules as production code, with the naming exceptions gtest requires (configured in `.clang-tidy`).

---

## 6. Toolchain

Modern, pinned, identical on every developer machine and in CI. This section is the contract; `setup_env.sh` implements it. Anything not listed here is not installed.

### 6.1 Python: `uv`

- [`uv`](https://docs.astral.sh/uv/) is the only Python environment and dependency tool. `pip`, `venv`, `virtualenv`, `conda`, `poetry`, `pipx` and `requirements*.txt` are not used. Install uv once with the official installer (`curl -LsSf https://astral.sh/uv/install.sh | sh`); its version is pinned in `.github/workflows/*.yml` via `astral-sh/setup-uv` and recorded in the table in §6.4.
- **Layout.** The repository root `pyproject.toml` is a uv workspace root and holds all tool configuration (§7.3). `ml/` is the single workspace member and the only installable package (`nuway-ml`, build backend `hatchling`). `tools/` and `tests/` are not packages; they run through `uv run` with `ml/` importable. `uv.lock` is committed and is the source of truth for every version. `.python-version` pins `3.10`.
- **Dependency groups** in the root `pyproject.toml`: `dev` (ruff, mypy, pytest, pytest-cov, pre-commit, `clang-format`, `clang-tidy`, `gersemi`), `carla` (the CARLA 0.9.16 client wheel), `train` (wandb/tensorboard, hydra-core). `uv sync` installs `dev` by default; `uv sync --group carla --group train` for a full workstation. Runtime dependencies of the model code (`torch`, `numpy`, `webdataset`, …) live in `ml/pyproject.toml` `[project.dependencies]`.
- **PyTorch** is pinned to a CUDA 12.x wheel index through `[[tool.uv.index]]` (`explicit = true`) plus `[tool.uv.sources]`, so `uv sync` never pulls the CPU-only wheel by accident. The exact `cu12x` index is decided in M0 against the workstation driver and recorded in §6.4.
- **ROS 2 interop.** ROS Humble's `rclpy`, `rosidl_runtime_py` and the generated `nuway_msgs` bindings live in Ubuntu 22.04's system Python 3.10 and cannot be installed from PyPI. The venv is therefore created from the system interpreter with system site packages visible: `uv venv --python /usr/bin/python3.10 --system-site-packages`, then `uv sync` populates it. `[tool.uv] python-preference = "only-system"` prevents uv from downloading its own interpreter, which would break the ABI match with ROS. `setup_env.sh` does this in the right order (source ROS, create venv, sync, source `ros2_ws/install/setup.bash`).
- **Commands.** `uv sync` to create or update the environment; `uv run <cmd>` for every tool and script (`uv run pytest ml`, `uv run ruff check .`, `uv run tools/eval/run_routes.py`); `uv add <pkg>` / `uv add --group dev <pkg>` to add dependencies, which updates `uv.lock` in the same commit; `uv lock --upgrade-package <pkg>` for controlled upgrades. Never `pip install`, never edit `.venv` by hand, never `uv pip` outside of debugging.
- **Launch files and rclpy nodes** are executed by `ros2 launch` / `ros2 run` from a shell where the venv is activated (`setup_env.sh` does `source .venv/bin/activate`), so they see both ROS and the locked dependencies. Nodes must not assume a global Python; anything not in `uv.lock` or the ROS distribution is unavailable.
- `pre-commit` is installed from the `dev` group and run as `uv run pre-commit install` once, then automatically on commit. Its hooks call the tools through `uv run` so pre-commit, the editor, `tools/lint/*.sh` and CI use the exact same binaries.

### 6.2 C++: compilers, build, dependencies

- **Compiler.** GCC 11.4 (Ubuntu 22.04 default, ROS Humble ABI) is the reference compiler for CI and release builds. Clang 17+ is supported for local builds (`CC=clang-17 CXX=clang++-17`) and is what clang-tidy models; code must compile warning-free on both. C++17, no extensions (§2.6).
- **Build system.** `colcon` over `ament_cmake`, CMake ≥ 3.22, **Ninja** generator, **ccache** compiler launcher, **mold** linker. These are configured once in `ros2_ws/colcon_defaults.yaml`, exported as `COLCON_DEFAULTS_FILE` by `setup_env.sh`, so a bare `colcon build` is already correct:

  ```yaml
  # ros2_ws/colcon_defaults.yaml
  build:
    symlink-install: true
    cmake-args:
      - -G Ninja
      - -DCMAKE_BUILD_TYPE=RelWithDebInfo
      - -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
      - -DCMAKE_CXX_COMPILER_LAUNCHER=ccache
      - -DCMAKE_EXE_LINKER_FLAGS=-fuse-ld=mold
      - -DCMAKE_SHARED_LINKER_FLAGS=-fuse-ld=mold
    event-handlers: [console_cohesion+, status-]
  test:
    event-handlers: [console_direct+]
    return-code-on-test-failure: true
  ```

  `ninja-build`, `ccache` and `mold` come from apt and are listed in `setup_env.sh`'s prerequisite check. Use `make` nowhere.
- **`nuway_cmake` package.** A tiny `ament_cmake` package that every `nuway_*` C++ package depends on (`<buildtool_depend>nuway_cmake</buildtool_depend>`). It provides `nuway_target_defaults(<target>)`, which sets: `CXX_STANDARD 17`, `CXX_EXTENSIONS OFF`; warnings `-Wall -Wextra -Wpedantic -Wshadow -Wnon-virtual-dtor -Wold-style-cast -Woverloaded-virtual -Wnull-dereference -Wdouble-promotion -Wformat=2 -Werror` (`-Werror` disabled with `-DNUWAY_WERROR=OFF` for third-party debugging only); third-party include directories marked `SYSTEM`; and the options `NUWAY_CLANG_TIDY=ON|OFF` (sets `CMAKE_CXX_CLANG_TIDY` to the uv-managed wheel binary described below), `NUWAY_SANITIZE=<none|address,undefined|thread>` and `NUWAY_LTO=ON|OFF`. Packages do not set compiler flags themselves. `CMakeLists.txt` files are formatted by `gersemi` (pinned in the `dev` group, enforced in pre-commit).
- **Dependencies.** Resolution order: (1) ROS/Ubuntu apt via `rosdep` (`rclcpp`, Eigen, PCL, gtest, tf2, …); (2) an `<lib>_vendor` `ament_cmake` package under `ros2_ws/src/` for anything not in Humble's apt (GTSAM 4.2, OSQP, osqp-eigen, nanoflann), following the ROS vendor-package convention: CMake `FetchContent` pinned to a release **tag and commit hash**, built once as part of the workspace, exported with `ament_export_targets`. No git submodules, no system-wide `make install`, no `find_package` of something that rosdep cannot install. Every dependency is declared in `package.xml` so `rosdep install --from-paths ros2_ws/src -yi` fully prepares a fresh machine.
- **clang-format and clang-tidy** are installed from their PyPI wheels (`clang-format`, `clang-tidy`, LLVM ≥ 17) pinned in the `dev` dependency group and invoked as `uv run clang-format` / `uv run clang-tidy`. This gives byte-identical formatting and diagnostics in the editor, pre-commit, `tools/lint/` and CI regardless of what apt provides (Ubuntu 22.04 ships LLVM 14, which is not used). `nuway_cmake` resolves `CMAKE_CXX_CLANG_TIDY` to `.venv/bin/clang-tidy` for the same reason.
- **Compilation database and clangd.** colcon writes one `compile_commands.json` per package under `ros2_ws/build/<pkg>/`. `tools/lint/merge_compile_commands.py` merges them into `ros2_ws/build/compile_commands.json` (run by `tools/lint/tidy_cpp.sh` and by `setup_env.sh` after a build). The root `.clangd` points at it:

  ```yaml
  # .clangd
  CompileFlags:
    CompilationDatabase: ros2_ws/build
  Diagnostics:
    ClangTidy:
      # Reuse .clang-tidy; clangd surfaces the same checks in the editor.
      FastCheckFilter: Loose
  ```

- **Sanitizers.** CI runs the pure-library gtest suites a second time with `-DNUWAY_SANITIZE=address,undefined`. Any sanitizer report is a failure. Locally: `colcon build --cmake-args -DNUWAY_SANITIZE=address,undefined --build-base build_asan --install-base install_asan`.

### 6.3 Environment bootstrap

`setup_env.sh` (idempotent, safe to `source` in every shell) does, in order:

1. Verify prerequisites: Ubuntu 22.04, `/opt/ros/humble`, `uv`, `ninja`, `ccache`, `mold`, `cmake ≥ 3.22`, `gcc-11`; print the install command for anything missing and stop.
2. `source /opt/ros/humble/setup.bash`; `export COLCON_DEFAULTS_FILE=$REPO/ros2_ws/colcon_defaults.yaml`; `export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`.
3. Create `.venv` if absent (`uv venv --python /usr/bin/python3.10 --system-site-packages`), then `uv sync` (with `--group carla --group train` when `NUWAY_FULL=1`).
4. `source .venv/bin/activate`; `source ros2_ws/install/setup.bash` if the workspace has been built.
5. `uv run pre-commit install` if the git hook is missing.

CI uses the same script inside the `ros:humble` container so there is one definition of "the environment".

### 6.4 Pinned versions

Decided in M0 and kept current here whenever `uv.lock` or the workflows change a pin. Until then the column shows the minimum.

| Tool | Pin | Where pinned |
|------|-----|--------------|
| Ubuntu / ROS 2 | 22.04 / Humble | `00_overview.md` §4, CI image `ros:humble` |
| Python | 3.10 (system) | `.python-version`, `[project] requires-python` |
| uv | ≥ 0.5, exact in CI | `astral-sh/setup-uv@vN` with `version:` |
| GCC | 11.4 | apt (distro default) |
| CMake / Ninja / ccache / mold | ≥ 3.22 / ≥ 1.10 / ≥ 4.5 / ≥ 1.0 | apt (distro default) |
| clang-format, clang-tidy (PyPI wheels) | ≥ 17, exact in lock | `pyproject.toml` `dev` group, `uv.lock` |
| ruff, mypy, pytest, pre-commit, gersemi | exact in lock | `pyproject.toml` `dev` group, `uv.lock` |
| torch | ≥ 2.4, `cu12x` index | `ml/pyproject.toml`, `[[tool.uv.index]]` |
| GTSAM / OSQP / osqp-eigen / nanoflann | 4.2.0 / tag / tag / tag, by commit hash | `ros2_ws/src/*_vendor/CMakeLists.txt` |

---

## 7. Style tooling specification

The following files will be added as part of M0 (see the M0 task list). Their intended content is recorded here so that the style is defined before the first line of C++ or Python lands. Once the files exist, they are the source of truth and this section is only an explanation of them. Every tool named below is the `uv`-managed binary from §6; `clang-format` means `uv run clang-format`.

### 7.1 `.clang-format`

```yaml
# Google C++ style, as enforced across nuway. Do not add per-directory overrides.
BasedOnStyle: Google
Standard: c++17
ColumnLimit: 80
DerivePointerAlignment: false
PointerAlignment: Left
IncludeBlocks: Regroup
IncludeCategories:
  # 1. matching header handled by IncludeIsMainRegex (priority 0)
  # 2. C system headers
  - Regex: '^<(sys/|unistd\.h|fcntl\.h|pthread\.h|errno\.h|signal\.h)'
    Priority: 1
  # 3. C++ standard library (no dot, no slash)
  - Regex: '^<[a-z_0-9]+>$'
    Priority: 2
  # 5. other nuway packages
  - Regex: '^<nuway_[a-z_]+/'
    Priority: 4
  # 4. third-party and ROS (everything else in angle brackets)
  - Regex: '^<.*'
    Priority: 3
  # 6. same package (quoted)
  - Regex: '^".*'
    Priority: 5
IncludeIsMainRegex: '(_test)?$'
SortIncludes: CaseSensitive
```

Everything not listed inherits from the Google base style (2-space indent, attached braces, 80 columns, etc.). Formatting checks apply to `*.hpp`, `*.cpp` under `ros2_ws/src/nuway_*` and `tests/`. Vendored packages (`carla_msgs`) are excluded.

### 7.2 `.clang-tidy`

```yaml
# Google C++ style enforcement plus bug-finding checks. Warnings are errors in CI.
Checks: >
  -*,
  bugprone-*,
  -bugprone-easily-swappable-parameters,
  cppcoreguidelines-init-variables,
  cppcoreguidelines-pro-type-cstyle-cast,
  cppcoreguidelines-pro-type-static-cast-downcast,
  cppcoreguidelines-special-member-functions,
  google-*,
  llvm-header-guard,
  llvm-namespace-comment,
  misc-*,
  -misc-non-private-member-variables-in-classes,
  -misc-include-cleaner,
  modernize-*,
  -modernize-use-trailing-return-type,
  -modernize-avoid-c-arrays,
  performance-*,
  readability-*,
  -readability-magic-numbers,
  -readability-else-after-return,
  -readability-function-cognitive-complexity
WarningsAsErrors: '*'
HeaderFilterRegex: '.*/ros2_ws/src/nuway_[a-z_]+/.*'
FormatStyle: file
CheckOptions:
  # Google naming, machine-checked.
  readability-identifier-naming.NamespaceCase: lower_case
  readability-identifier-naming.ClassCase: CamelCase
  readability-identifier-naming.StructCase: CamelCase
  readability-identifier-naming.EnumCase: CamelCase
  readability-identifier-naming.TypeAliasCase: CamelCase
  readability-identifier-naming.TypedefCase: CamelCase
  readability-identifier-naming.TemplateParameterCase: CamelCase
  readability-identifier-naming.FunctionCase: CamelCase
  readability-identifier-naming.MethodCase: CamelCase
  readability-identifier-naming.VariableCase: lower_case
  readability-identifier-naming.ParameterCase: lower_case
  readability-identifier-naming.MemberCase: lower_case
  readability-identifier-naming.PrivateMemberSuffix: '_'
  readability-identifier-naming.ProtectedMemberSuffix: '_'
  readability-identifier-naming.PublicMemberCase: lower_case
  readability-identifier-naming.ConstexprVariableCase: CamelCase
  readability-identifier-naming.ConstexprVariablePrefix: 'k'
  readability-identifier-naming.GlobalConstantCase: CamelCase
  readability-identifier-naming.GlobalConstantPrefix: 'k'
  readability-identifier-naming.StaticConstantCase: CamelCase
  readability-identifier-naming.StaticConstantPrefix: 'k'
  readability-identifier-naming.ClassConstantCase: CamelCase
  readability-identifier-naming.ClassConstantPrefix: 'k'
  readability-identifier-naming.EnumConstantCase: CamelCase
  readability-identifier-naming.EnumConstantPrefix: 'k'
  readability-identifier-naming.MacroDefinitionCase: UPPER_CASE
  # Cheap accessors are allowed to be snake_case (Google "Function Names").
  readability-identifier-naming.MethodIgnoredRegexp: '^(get|set)_[a-z_0-9]+$|^[a-z_0-9]+$'
  # gtest macros generate identifiers we do not control.
  readability-identifier-naming.FunctionIgnoredRegexp: '^(TEST|TEST_F|TEST_P|TYPED_TEST).*'
  llvm-header-guard.HeaderFileExtensions: 'hpp'
  google-readability-braces-around-statements.ShortStatementLines: 0
  readability-braces-around-statements.ShortStatementLines: 0
  readability-implicit-bool-conversion.AllowIntegerConditions: false
  readability-implicit-bool-conversion.AllowPointerConditions: false
  performance-unnecessary-value-param.AllowedTypes: 'SharedPtr;ConstSharedPtr'
```

Notes:

- `llvm-header-guard` in stock form derives the expected guard from the file path. The `NUWAY_<PKG>_<FILE>_HPP_` pattern is enforced by a small script in `tools/lint/` alongside clang-tidy; the check remains enabled to catch missing guards.
- `readability-magic-numbers` is off because planning and control code is full of justified tunables; those must still be named constants when reused (§2.1).
- Checks may only be disabled repo-wide in `.clang-tidy` with a comment explaining why. Inline `// NOLINT(check-name)` requires a trailing justification: `// NOLINT(bugprone-narrowing-conversions): CARLA API takes float`. Bare `// NOLINT` is rejected in review.
- Generated code (`nuway_msgs` headers, pybind stubs) and vendored packages are excluded by `HeaderFilterRegex` and by not being under `nuway_*` source dirs.

### 7.3 `pyproject.toml` (repository root)

The uv workspace root (§6.1) and the single place for Python tool configuration. Applies to every `.py` file in the repo: `ml/`, `tools/`, the rclpy packages and launch files under `ros2_ws/src/nuway_*`, and `tests/`. `ml/pyproject.toml` (the installable package) holds only package metadata and runtime dependencies; it must not contain `[tool.ruff]` or `[tool.mypy]` sections.

```toml
[project]
name = "nuway"
version = "0.0.0"
requires-python = "==3.10.*"
dependencies = ["nuway-ml"]

[dependency-groups]
dev = [
  "ruff", "mypy", "pytest", "pytest-cov", "pre-commit",
  "clang-format", "clang-tidy",   # LLVM >= 17 wheels; exact pins in uv.lock
  "gersemi",
]
carla = ["carla==0.9.16"]
train = ["hydra-core", "wandb", "tensorboard"]

[tool.uv]
package = false                    # the root is not installable
python-preference = "only-system"  # must be Humble's /usr/bin/python3.10

[tool.uv.workspace]
members = ["ml"]

[tool.uv.sources]
nuway-ml = { workspace = true }
torch = { index = "pytorch-cu12x" }

[[tool.uv.index]]
name = "pytorch-cu12x"             # exact cu12x chosen in M0 (see §6.4)
url = "https://download.pytorch.org/whl/cu124"
explicit = true

[tool.pytest.ini_options]
testpaths = ["ml/tests", "tools", "tests"]
addopts = "--strict-markers"
markers = ["slow: minutes-long", "carla: needs a running CARLA server", "gpu: needs CUDA"]

[tool.ruff]
target-version = "py310"
line-length = 88
extend-exclude = ["ros2_ws/build", "ros2_ws/install", "ros2_ws/log", "ros2_ws/src/carla_msgs", "data"]

[tool.ruff.format]
quote-style = "double"
docstring-code-format = true

[tool.ruff.lint]
select = [
  "E", "W",    # pycodestyle
  "F",         # pyflakes
  "I",         # isort
  "N",         # pep8-naming
  "UP",        # pyupgrade (3.10 idioms)
  "B",         # bugbear
  "D",         # pydocstyle
  "ANN",       # type annotations required
  "ARG",       # unused arguments
  "C4",        # comprehensions
  "SIM",       # simplify
  "RET",       # return consistency
  "PL",        # pylint subset
  "RUF",       # ruff-specific
  "NPY",       # numpy idioms
  "PT",        # pytest style
  "T20",       # no print()
]
ignore = [
  "D105", "D107",    # magic methods / __init__ do not need docstrings
  "PLR0913",         # many-argument functions are common in model constructors
  "PLR2004",         # magic values: same reasoning as readability-magic-numbers in C++
  "E501",            # line length is the formatter's job
]

[tool.ruff.lint.pydocstyle]
convention = "pep257"

[tool.ruff.lint.per-file-ignores]
"**/tests/**" = ["D", "ANN", "PLR", "ARG"]
"**/test_*.py" = ["D", "ANN", "PLR", "ARG"]
"tools/**" = ["T20"]          # CLI tools may print
"**/launch/*.launch.py" = ["D", "ANN"]

[tool.ruff.lint.isort]
known-first-party = ["nuway_ml", "nuway_eval", "nuway_carla_bridge", "nuway_perception", "nuway_prediction", "nuway_planning", "nuway_bringup", "nuway_viz"]

[tool.mypy]
python_version = "3.10"
strict = true
warn_unreachable = true
files = ["ml/nuway_ml", "tools", "tests"]
exclude = ["ros2_ws/(build|install|log)/", "data/"]

[[tool.mypy.overrides]]
module = ["carla", "carla.*", "rclpy", "rclpy.*", "rosidl_runtime_py", "nuway_msgs.*", "carla_msgs.*", "torch.*", "webdataset", "webdataset.*"]
ignore_missing_imports = true

[[tool.mypy.overrides]]
# ROS nodes and launch files: typed, but not strict (rclpy has no stubs).
module = ["nuway_carla_bridge.*", "nuway_perception.*", "nuway_prediction.*", "nuway_planning.*", "nuway_eval.*", "nuway_bringup.*", "nuway_viz.*"]
strict = false
disallow_untyped_defs = true
```

Notes:

- `ruff format` replaces Black; `ruff check` replaces flake8, isort, pyupgrade, pydocstyle and the pylint subset. No other Python linter or formatter is used, so there is exactly one configuration to keep in sync.
- Rule groups may only be disabled repo-wide in `pyproject.toml` with a comment. Inline `# noqa: <RULE>` requires a trailing justification: `# noqa: PLR0912  -- CARLA blueprint matrix, table-driven`. Bare `# noqa` and `# type: ignore` without a rule code are rejected in review.
- `mypy --strict` is the bar for `ml/nuway_ml`, `tools/` and `tests/`. The `ros2_ws` Python packages must still have fully typed function signatures (`disallow_untyped_defs`), but strict mode is relaxed because `rclpy` and generated message packages ship without stubs.
- Tool versions are pinned once, in `uv.lock` (§6.4). `.pre-commit-config.yaml` uses `repo: local` hooks that call `uv run ruff`, `uv run mypy`, `uv run clang-format`, etc., so pre-commit never carries its own second set of pins.

### 7.4 How the tools are run

All tools are the `uv`-managed binaries from §6 (`uv run clang-format`, `uv run ruff`, …). Versions come from `uv.lock`; see §6.4.

| Layer | What | When |
|-------|------|------|
| Editor | C++: clang-format on save, clangd using `compile_commands.json` from `ros2_ws/build/`. Python: ruff format on save, ruff and mypy language servers | continuously |
| Pre-commit | `pre-commit` (`repo: local` hooks via `uv run`): `clang-format --dry-run --Werror` on staged C++ files; `clang-tidy` on staged `.cpp` files using the merged `compile_commands.json`; `gersemi --check` on staged `CMakeLists.txt`/`*.cmake`; `ruff format --check` and `ruff check` on staged `.py` files; `mypy` on the packages containing staged `.py` files | every commit |
| Local full run | C++: `tools/lint/format_cpp.sh [--fix]` and `tools/lint/tidy_cpp.sh [--fix]` (wraps `run-clang-tidy -p ros2_ws/build`). Python: `tools/lint/lint_py.sh [--fix]` (runs `ruff format`, `ruff check`, `mypy` from the repo root) | before pushing; Claude Code after every change |
| Build / test | C++: every `nuway_*` `ament_cmake` package sets `CMAKE_EXPORT_COMPILE_COMMANDS ON` and honors `-DNUWAY_CLANG_TIDY=ON` which sets `CMAKE_CXX_CLANG_TIDY` so `colcon build` fails on tidy errors. Python: `pytest` runs with `--strict-markers`; ruff and mypy are separate CI jobs, not pytest plugins | opt-in locally, on in CI |
| CI | `ros:humble` container, `setup_env.sh`, uv cache keyed on `uv.lock`. C++: format + gersemi check on the full tree; `colcon build --cmake-args -DNUWAY_CLANG_TIDY=ON`; a second build + `colcon test` with `-DNUWAY_SANITIZE=address,undefined`. Python: `uv run ruff format --check .`, `uv run ruff check .`, `uv run mypy`, `uv run pytest -m "not slow and not carla and not gpu"` | every push / PR |

Expected local workflow for a C++ change:

```
source setup_env.sh                       # once per shell; colcon defaults + venv
colcon build --packages-select <pkg>      # Ninja/ccache/mold/compile_commands via colcon_defaults.yaml
tools/lint/format_cpp.sh --fix            # uv run clang-format + gersemi
tools/lint/tidy_cpp.sh --packages <pkg>   # merges compile_commands, uv run clang-tidy
colcon test --packages-select <pkg> && colcon test-result --verbose
```

Expected local workflow for a Python change:

```
source setup_env.sh
tools/lint/lint_py.sh --fix        # uv run ruff format, ruff check --fix, mypy
uv run pytest ml tools             # or the relevant subset
```

Every step must be clean before a commit. A commit that touches C++ or Python and was not run through these steps is treated as a bug.

---

## 8. Checklist for Claude Code

Before declaring a C++ task done:

1. `clang-format` reports no diff.
2. `clang-tidy` reports no warnings for the changed package (with `-DNUWAY_CLANG_TIDY=ON` or `tools/lint/tidy_cpp.sh`).
3. Every new header has the `NUWAY_<PKG>_<FILE>_HPP_` guard and a file-level comment.
4. Every new class/function follows the naming table in §2.1; every parameter member matches its YAML key plus `_`.
5. No `throw` in library code; exceptions from third-party code caught at the node boundary.
6. Frames and units are stated in comments at every interface.
7. New library code has a gtest under `test/` that does not link `rclcpp`.
8. `docs/` updated if a node, topic, parameter, or file was renamed.

Before declaring a Python task done:

1. `ruff format --check` reports no diff.
2. `ruff check` reports no violations; every `# noqa` carries a rule code and a justification.
3. `mypy` is clean for the touched packages; every `# type: ignore` carries an error code.
4. Every public function, class and module has a PEP 257 docstring stating frames, units and tensor shapes where relevant.
5. Parity modules keep the same function names (modulo case) and argument order as their `nuway_common` twin.
6. New code has a pytest under the package's `tests/` directory.
7. `docs/` updated if a node, topic, parameter, CLI flag, or file was renamed.

---

## 9. Python conventions

Applies to `ml/`, `tools/`, `tests/`, and the rclpy packages and launch files in `ros2_ws/`. Enforced by `ruff` and `mypy` with the root `pyproject.toml` (§7.3), at the same level as the C++ rules: not clean, not merged.

### 9.1 Standards

- [PEP 8](https://peps.python.org/pep-0008/) for code layout and naming, [PEP 257](https://peps.python.org/pep-0257/) for docstrings, [PEP 484](https://peps.python.org/pep-0484/)/[PEP 604](https://peps.python.org/pep-0604/) for type hints. No house style on top; `ruff` is the arbiter.
- Formatting is fully delegated to `ruff format` (Black-compatible): 88 columns, double quotes, trailing commas as the formatter decides. Never hand-format. `# fmt: off` / `# fmt: on` only around literal tables, always paired.
- Python 3.10 exactly (`target-version = "py310"`): use `match`, `X | None`, `dict[str, int]` builtins generics; no `typing.Optional`, `typing.List`. No 3.11+ features.

### 9.2 Naming

| Entity | Rule | Example |
|--------|------|---------|
| Modules, packages | `lower_snake_case`, short | `flow_matching.py`, `nuway_ml.common` |
| Classes, exceptions, type aliases | `CamelCase`; exceptions end in `Error` | `class GridSpec`, `class RouteLoadError(Exception)` |
| Functions, methods, variables, parameters | `lower_snake_case` | `to_frenet()`, `lateral_error` |
| Module-level constants | `UPPER_SNAKE_CASE` | `MAX_STEER_RAD = 1.22` |
| Private module/class members | single leading underscore | `_sample_bilinear()`, `self._pub_cmd` |
| Type variables | `CamelCase`, short, optionally `_T` suffix | `T`, `ArrayT` |
| Test files / functions | `test_<unit>.py`, `test_<behavior>()` | `tests/test_frenet.py::test_round_trip_on_arc` |

Extra project rules:

- Units in names when not obvious from the type: `speed_mps`, `dt_s`, `yaw_rad`. Same suffix set as C++ (§2.1).
- Parity modules mirror their C++ twin: `nuway_common/frenet.hpp::ToFrenet` ↔ `nuway_ml/common/frenet.py::to_frenet`, same argument order, same return structure. The cross-language parity test reads one-to-one.
- rclpy node classes end in `Node`; publisher/subscription/timer attributes are `self._pub_<what>`, `self._sub_<what>`, `self._timer_<what>`; callbacks are `_on_<message>()` / `_on_timer()`. This is the snake_case image of the C++ node conventions (§4).
- ROS parameter names are identical to the YAML keys in `configs/`, stored as `self._<key>`.

### 9.3 Types

- Every function and method signature is fully annotated (`ANN` rules, `disallow_untyped_defs`). Return type included, `-> None` spelled out.
- `mypy --strict` for `ml/nuway_ml`, `tools/`, `tests/`. Typed but non-strict for `ros2_ws` Python because `rclpy` and generated message packages have no stubs (§7.3).
- Tensor and array shapes are documented in the docstring in the form `(B, N, 2)`; use `jaxtyping`-style comments only if the project adopts the library later (it is not in the dependency list).
- Data records are `@dataclass(frozen=True, slots=True)` or `TypedDict`; never bare dicts crossing a module boundary. Dataset schema lives in `nuway_ml/common/schema.py` (see `01_directory_structure.md`).
- `Any` only at the boundary with untyped libraries (CARLA API, rclpy messages) and immediately narrowed. Never `# type: ignore` without an error code.

### 9.4 Docstrings and comments

- PEP 257 docstrings (`convention = "pep257"`): one-line summary in the imperative, blank line, then details. Every public module, class and function has one; private helpers when non-obvious.
- State frames, units, and shapes at every interface, exactly as in C++ (§2.7): `"""Return agent poses in the map frame (ROS convention), shape (N, 3) as x, y, yaw_rad."""`
- `# TODO(username): text` or `# TODO(M4): text`. No bare `TODO`.
- Module docstring first line states what the module provides and which milestone introduced it.

### 9.5 Language and library rules

- Imports: absolute only, sorted by ruff (`I` rules) into stdlib / third-party / first-party groups. No wildcard imports. No imports inside functions except to break a genuine cycle or to keep `torch`/`carla` out of a light CLI's startup path, with a comment.
- No `print()` in library code or nodes (`T20`); `logging.getLogger(__name__)` in `ml/`, `self.get_logger()` in rclpy nodes. CLI entry points under `tools/` may print.
- No bare `except:` and no `except Exception` that swallows; re-raise or convert to a domain `*Error` with `from`.
- Errors: raise exceptions (Python's idiom, unlike our C++ rule). Public library functions document what they raise. Nodes catch at the callback boundary and report via `/nuway/diag/<node>`, never let an exception kill the executor.
- `pathlib.Path` over `os.path`; `argparse` (or `tyro` if adopted) for CLIs; `dataclasses` or `omegaconf` structured configs for training configs, never ad-hoc dicts.
- `torch`: explicit `device` and `dtype` arguments, no implicit `.cuda()`; `torch.no_grad()` / `inference_mode()` in inference nodes; shapes asserted at module boundaries in debug mode. Random seeds set through one helper in `nuway_ml/common`.
- `numpy`: `NPY` rules apply (new-style `np.random.default_rng`, no deprecated aliases).
- Mutable default arguments never (`B006`). Comprehensions over `map`/`filter` with lambdas (`C4`).
- No environment variable reads in nodes; `tools/` may read them only for paths (`CARLA_ROOT`) and must document them in `--help`.

### 9.6 Tests

- `pytest`, files `test_<unit>.py` under `ml/tests/`, `tools/**/tests/`, or `tests/integration/`. One test file per module under test.
- Test function names read as a sentence: `test_round_trip_on_arc_within_tolerance`. Use `pytest.approx` or `np.testing.assert_allclose` with explicit tolerances, justified in a comment when not obvious.
- Markers registered in `pyproject.toml` (`slow`, `carla`, `gpu`); `--strict-markers` is on. Tests needing CARLA or a GPU are skipped, not failed, when the resource is missing.
- Tests are subject to `ruff format` and `ruff check` with the relaxations listed in `per-file-ignores` (§7.3); `mypy` still runs on them.
