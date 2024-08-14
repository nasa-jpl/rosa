# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

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
