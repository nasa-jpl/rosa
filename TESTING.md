# ROSA Testing

## Introduction
This document provides an overview of the testing architecture for ROSA. It covers the categories of tests applied across the software development lifecycle and describes the automated execution of these tests via GitHub Actions.

---

## Testing Categories

The following test categories are included in our testing setup. Further details are provided below.

- [ ] Static Code Analysis
- [x] Unit Tests
- [x] Security Tests
- [ ] Build Tests
- [ ] Acceptance Tests
- [x] Integration Tests
- [x] Performance Tests
- [ ] Usability Tests

### Unit Tests

#### Calculation Tools

- **Location:** `./tests/rosa/tools/test_calculation.py`
- **Purpose:** To ensure the mathematical functions and calculations in ROSA perform correctly across various edge cases and inputs.
- **Running Tests:**
  - **Manually:**
    1. Navigate to the project root directory in the command line.
    2. Execute `pytest ./tests/rosa/tools/test_calculation.py`.
    3. View results: Results will appear in the command-line output or can be formatted into a report using the `pytest-html` plugin.
  - **Automatically:**
    - **Frequency:**
      - Triggered by code changes and commits to the `src/rosa/tools/calculation.py` file on GitHub.
      - Runs during nightly builds with other unit tests.
    - **Results Location:** [GitHub Actions Unit Test Results](https://github.com/nasa-jpl/rosa/actions/workflows/unit-tests.yml)
- **Contributing:**
  - **Framework Used:** [PyTest](https://docs.pytest.org/en/8.2.x/)
  - **Tips:**
    - Ensure that all functions in `calculation.py` are covered, including edge cases.
    - Test for both typical inputs and boundary conditions to ensure robustness.

#### Log Tools

- **Location:** `./tests/rosa/tools/test_log.py`
- **Purpose:** To verify the functionality of log reading and handling operations in ROSA, including error handling and filtering.
- **Running Tests:**
  - **Manually:**
    1. Navigate to the project root directory in the command line.
    2. Execute `pytest ./tests/rosa/tools/test_log.py`.
    3. View results: Results will appear in the command-line output or can be formatted into a report using the `pytest-html` plugin.
  - **Automatically:**
    - **Frequency:**
      - Triggered by code changes and commits to the `src/rosa/tools/log.py` file on GitHub.
      - Runs during nightly builds with other unit tests.
    - **Results Location:** [GitHub Actions Unit Test Results](https://github.com/nasa-jpl/rosa/actions/workflows/unit-tests.yml)
- **Contributing:**
  - **Framework Used:** [PyTest](https://docs.pytest.org/en/8.2.x/)
  - **Tips:**
    - Ensure all log-related edge cases are tested, including invalid paths, empty files, and large files.
    - Test the filtering functionality thoroughly.

#### ROS1 Tools

- **Location:** `./tests/rosa/tools/test_ros1.py`
- **Purpose:** To validate the integration and functionality of ROS1 tools within ROSA, ensuring correct interactions with the ROS1 environment.
- **Running Tests:**
  - **Manually:**
    1. Ensure that ROS1 is set up and running in your environment.
    2. Execute `pytest ./tests/rosa/tools/test_ros1.py`.
    3. View results: Results will appear in the command-line output or can be formatted into a report using the `pytest-html` plugin.
  - **Automatically:**
    - **Frequency:**
      - Triggered by code changes and commits to the `src/rosa/tools/ros1.py` file on GitHub.
      - Runs during nightly builds with other unit tests.
    - **Results Location:** [GitHub Actions Unit Test Results](https://github.com/nasa-jpl/rosa/actions/workflows/unit-tests.yml)
- **Contributing:**
  - **Framework Used:** [PyTest](https://docs.pytest.org/en/8.2.x/)
  - **Tips:**
    - Ensure tests are skipped if the ROS version is not compatible.
    - Test interactions with ROS1 nodes, topics, services, and parameters.

#### ROS2 Tools

- **Location:** `./tests/rosa/tools/test_ros2.py`
- **Purpose:** To validate the integration and functionality of ROS2 tools within ROSA, ensuring correct interactions with the ROS2 environment.
- **Running Tests:**
  - **Manually:**
    1. Ensure that ROS2 is set up and running in your environment.
    2. Execute `pytest ./tests/rosa/tools/test_ros2.py`.
    3. View results: Results will appear in the command-line output or can be formatted into a report using the `pytest-html` plugin.
  - **Automatically:**
    - **Frequency:**
      - Triggered by code changes and commits to the `src/rosa/tools/ros2.py` file on GitHub.
      - Runs during nightly builds with other unit tests.
    - **Results Location:** [GitHub Actions Unit Test Results](https://github.com/nasa-jpl/rosa/actions/workflows/unit-tests.yml)
- **Contributing:**
  - **Framework Used:** [PyTest](https://docs.pytest.org/en/8.2.x/)
  - **Tips:**
    - Ensure tests are skipped if the ROS version is not compatible.
    - Test interactions with ROS2 nodes, topics, services, and parameters.

#### ROSA Tools

- **Location:** `./tests/rosa/tools/test_rosa_tools.py`
- **Purpose:** To test the general ROSA tools and their integration, including utility functions and framework-specific features.
- **Running Tests:**
  - **Manually:**
    1. Navigate to the project root directory in the command line.
    2. Execute `pytest ./tests/rosa/tools/test_rosa_tools.py`.
    3. View results: Results will appear in the command-line output or can be formatted into a report using the `pytest-html` plugin.
  - **Automatically:**
    - **Frequency:**
      - Triggered by code changes and commits to the `src/rosa/tools/rosa_tools.py` file on GitHub.
      - Runs during nightly builds with other unit tests.
    - **Results Location:** [GitHub Actions Unit Test Results](https://github.com/nasa-jpl/rosa/actions/workflows/unit-tests.yml)
- **Contributing:**
  - **Framework Used:** [PyTest](https://docs.pytest.org/en/8.2.x/)
  - **Tips:**
    - Ensure compatibility across different ROS versions.
    - Validate tool initialization and injection of dependencies.

#### System Tools

- **Location:** `./tests/rosa/tools/test_system.py`
- **Purpose:** To verify the system-related functionalities within ROSA, such as verbosity settings, debugging, and system wait functions.
- **Running Tests:**
  - **Manually:**
    1. Navigate to the project root directory in the command line.
    2. Execute `pytest ./tests/rosa/tools/test_system.py`.
    3. View results: Results will appear in the command-line output or can be formatted into a report using the `pytest-html` plugin.
  - **Automatically:**
    - **Frequency:**
      - Triggered by code changes and commits to the `src/rosa/tools/system.py` file on GitHub.
      - Runs during nightly builds with other unit tests.
    - **Results Location:** [GitHub Actions Unit Test Results](https://github.com/nasa-jpl/rosa/actions/workflows/unit-tests.yml)
- **Contributing:**
  - **Framework Used:** [PyTest](https://docs.pytest.org/en/8.2.x/)
  - **Tips:**
    - Test the different system states, such as verbosity and debugging modes.
    - Validate the correct implementation of timing functions.
