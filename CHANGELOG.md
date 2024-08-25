# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added
- Streaming capability for ROSA responses
- New `help.py` module with `get_help` function for generating help messages in `turtle_agent` demo
- Asynchronous support in TurtleAgent class
- Live updating console output using `rich` library
- Command handler dictionary for special commands in TurtleAgent
- New `submit` method to handle both streaming and non-streaming responses in `turtle_agent` class

### Changed
- Updated TurtleAgent to support both streaming and non-streaming modes
- Refactored `run` method in TurtleAgent to use asynchronous operations
- Updated Dockerfile for better layering and reduced image size
- Changed launch file to accept a `streaming` argument

### Removed
- Removed redundant logging statements from various tools

### Fixed
- Improved error handling and display in streaming mode


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
