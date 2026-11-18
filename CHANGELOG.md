# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [1.0.11]

### Security

Thanks to [@Jasperxjy](https://github.com/Jasperxjy) for privately reporting the following vulnerabilities:

* [GHSA-wjfh-7q6v-vrm2](https://github.com/nasa-jpl/rosa/security/advisories/GHSA-wjfh-7q6v-vrm2): Shell command injection in ROS2 tool arguments (affects 1.0.0–1.0.10)
* [GHSA-4vf9-r5hq-7qww](https://github.com/nasa-jpl/rosa/security/advisories/GHSA-4vf9-r5hq-7qww): Path traversal and arbitrary local file read in `read_log` (affects 1.0.0–1.0.10)
* [GHSA-x7qw-596f-9878](https://github.com/nasa-jpl/rosa/security/advisories/GHSA-x7qw-596f-9878): Shell command injection in ROS1 `roslaunch` arguments (affects 1.0.5–1.0.10)

### Added

* New `ros2_topic_pub` tool with `rate`, `duration` and `wait_timeout` options; it waits for a matching subscriber before publishing (#94)
* `ros2_log_directories` resolves the ROS2 log directory from the environment when `rclpy` is unavailable (#94)

### Changed

* ROS2 graph tools (`ros2_node_list`, `ros2_topic_list`, `ros2_service_list`, `ros2_node_info`, `ros2_topic_info`, `ros2_service_info`) now query a shared, hidden `rclpy` node instead of spawning the `ros2` CLI on every call, falling back to the CLI when `rclpy` is unavailable or a native query fails. DDS discovery wait is bounded by `ROSA_ROS2_DISCOVERY_TIMEOUT` (default 2s) (#94)
* `rclpy` is now optional: the ROS2 tools module loads without it (#94)
* ROS2 tools now invoke the `ros2` CLI with an argument list instead of a shell command string, and validate entity names passed to it
* `roslaunch` (ROS1) now launches with an argument list instead of `os.system`
* `roslaunch` (ROS1) now validates its arguments against allowlists: package names must match `[A-Za-z][A-Za-z0-9_-]*`, and launch files must be plain file names ending in `.launch`, `.xml` or `.test` (no paths, `..`, spaces, or `name:=value` arguments) (#93)
* `read_log` is now limited to ROS log directories (`ROS_LOG_DIR`, `ROS_HOME/log`, `~/.ros/log`, and the ROS1/ROS2 default log directories); additional directories can be allowed via `ROSA_LOG_DIRS`

## [1.0.10]

### Added

* Added multi-provider model configuration for the TurtleAgent via `LLM_PROVIDER` (`openai`, `anthropic`, `ollama`)
* Added optional dependency extras for provider-specific installs: `anthropic`, `ollama`, and `all`

### Changed

* Refactored ROSA agent construction to use provider-agnostic LangChain `create_tool_calling_agent`
* Updated CI pipeline dependency installation to include Rust tooling and modern pip install flags
* Renamed test package paths from `tests/rosa` to `tests/test_rosa` and updated testing docs accordingly

### Fixed

* Token usage reporting now disables safely for non-OpenAI/Azure chat models
* Invalid `LLM_PROVIDER` values now fail early with a clear error message

## [1.0.9]

### Changed

* Updated Dockerfile to install Rust dependencies (required for building tiktoken)
* Bumped LangChain versions

### Fixed

* Fixed CI pipeline to include Rust installation for proper dependency building

## [1.0.8]

### Changed

* Bumped LangChain versions

### Fixed

* Type hint errors in `ros2` tests

## [1.0.7]

### Added

* Support for local models using Ollama, particularly Llama >= 3.1

### Changed

* Bumped LangChain versions
* Updated ROS1 tools: `rosservice_call`, `roslaunch_list` and `rosnode_kill`

## [1.0.6]

### Added

* Implemented streaming capability for ROSA responses
* Added `pyproject.toml` for modern Python packaging
* Implemented asynchronous operations in TurtleAgent for better responsiveness

### Changed

* Updated Dockerfile for improved build process and development mode support
* Refactored TurtleAgent class for better modularity and streaming support
* Improved bounds checking for turtle movements
* Updated demo script for better cross-platform support and X11 forwarding
* Renamed `set_debuging` to `set_debugging` in system tools

### Fixed

* Corrected typos and improved documentation in various files
* Fixed potential issues with turtle movement calculations

### Removed

* Removed unnecessary logging statements from turtle tools

## [1.0.5]

### Added

* CI pipeline for automated testing
* Unit tests for ROSA tools and utilities

### Changed

* Improvements to various ROS2 tools
* Upgrade dependencies:
    * `langchain` to 0.2.14
    * `langchain_core` to 0.2.34
    * `langchain-openai` to 0.1.22

## [1.0.4]  - 2024-08-21

### Added

* Implemented ros2 topic echo tool.

### Changed

* Refactored ROS2 tools for better error handling and response parsing.
* Added blacklist parameters to relevant ROS2 tools.

### Fixed

* Fixed a bug where getting a list of ROS2 log files failed.

## [1.0.3]  - 2024-08-17

### Added

* `rosservice_call` tool for ROS1

### Changed

* Changed ROSA class methods from private to protected to allow easier overrides.
* Updated ros1 `roslog` tools to handle multiple logging directories.
* Upgrade dependencies:
    * `langchain` to 0.2.13
    * `langchain-community` to 0.2.12
    * `langchain_core` to 0.2.32
    * `langchain-openai` to 0.1.21

## [1.0.2] - 2024-08-14

### Changed

* Changed the `rostopic_echo` tool to both echo the topic and return the messages as a list

### Fixed

* Fixed a bug where both `ros1` and `ros2` tools were being imported before checking the `ros_version` parameter (#6) (
  ec578c10)

## [1.0.1] - 2024-08-10

### Added

* Added a working demo of ROSA controlling the TurtleSim robot in simulation

### Changed

* Changed the constructor of the `ROSA` class to accept tools in the form of `@tool` functions or Python packages
  containing `@tool` functions.
