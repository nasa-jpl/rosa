# MTC Node Refactoring Plan

## Current State Analysis
- **mtc_node.cpp**: 762 lines - **TOO LARGE**
- Multiple concerns mixed in one file
- Existing partial refactoring started but not complete

## What Was Missing - Now Added to CMakeLists.txt

### 1. **New Library: `mtc_planners`**
   - **File**: `src/mtc_planners.cpp` + `include/motion_planner/mtc_planners.hpp`
   - **Responsibility**: Initialize and manage all planner instances
   - **Contents**:
     - PipelinePlanner setup (forward & return with custom joint weights)
     - JointInterpolationPlanner setup
     - CartesianPath planner setup
     - Robot model caching
     - Path constraints initialization

### 2. **Complete Library: `mtc_task_stages`**
   - **File**: `src/mtc_task_stages.cpp` + `include/motion_planner/mtc_task_stages.hpp`
   - **Responsibility**: Create and manage MTC tasks
   - **Contents**:
     - `createTask()` - forward pick-and-place task
     - `createTaskReturn()` - return task
     - `doTask()` - execute forward task
     - `doTaskReturn()` - execute return task
     - Depends on: `mtc_planners` for planner instances

### 3. **New Library: `mtc_scene_manager`**
   - **File**: `src/mtc_scene_manager.cpp` + `include/motion_planner/mtc_scene_manager.hpp`
   - **Responsibility**: Manage planning scene setup
   - **Contents**:
     - `setupPlanningScene()` - initialize collision objects, containers, tables
     - `getCylinderPosition()` - get target cylinder coordinates
     - Uses scene_objects preset definitions

### 4. **Refactored Main: `mtc_node.cpp`**
   - **Responsibility**: Node lifecycle and service handling
   - **Contents**:
     - Service callbacks (execute_task, execute_return_task)
     - Node initialization
     - Thread management
     - Orchestration of the three libraries

## Benefits of This Structure

| Aspect | Before | After |
|--------|--------|-------|
| **File Size** | 762 lines in one file | Split into 4 modules |
| **Reusability** | Hard to reuse components | Each module independent |
| **Testing** | Cannot unit test parts | Easy to test each module |
| **Maintenance** | Hard to modify one aspect | Isolated concerns |
| **Compilation** | All or nothing | Incremental builds with libraries |

## File Organization

```
include/motion_planner/
├── mtc_planners.hpp          [NEW]
├── mtc_task_stages.hpp       [EXISTING - needs update]
├── mtc_scene_manager.hpp     [NEW]
└── scene_objects.hpp         [EXISTING]

src/
├── mtc_planners.cpp          [NEW]
├── mtc_task_stages.cpp       [EXISTING - needs update]
├── mtc_scene_manager.cpp     [NEW]
├── mtc_node.cpp              [REFACTOR to ~200 lines]
└── minimal_planner.cpp       [EXISTING]
```

## CMakeLists.txt Changes

### Added Libraries:
```cmake
add_library(mtc_planners src/mtc_planners.cpp)
add_library(mtc_task_stages src/mtc_task_stages.cpp)
add_library(mtc_scene_manager src/mtc_scene_manager.cpp)
```

### Dependency Graph:
```
mtc_node (executable)
├── mtc_planners (library)
├── mtc_task_stages (library)
│   └── mtc_planners (library)
└── mtc_scene_manager (library)
```

### All components depend on:
- ROS2 core packages (rclcpp, geometry_msgs, etc.)
- MoveIt2 packages (moveit_core, moveit_ros_planning, etc.)
- MTC (moveit_task_constructor_core, moveit_task_constructor_msgs)
- Generated service interfaces (motion_planner/srv/ExecuteTask)

## Next Steps

1. **Create `mtc_planners.hpp` header**
   - Define `MTCPlanners` class
   - Declare planner instances as member variables
   - Declare initialization method

2. **Create `mtc_planners.cpp` implementation**
   - Move all planner initialization code from mtc_node.cpp
   - Move robot model caching code
   - Move path constraints initialization

3. **Update `mtc_task_stages.hpp` header**
   - Fix typo: `SharedPTr` → `SharedPtr`
   - Add missing include guards verification
   - Add node parameter for planner access

4. **Update `mtc_task_stages.cpp` implementation**
   - Use planners from MTCPlanners instead of duplicating
   - Move createTask() and createTaskReturn() from mtc_node.cpp

5. **Create `mtc_scene_manager.hpp` header**
   - Define `MTCSceneManager` class
   - Declare setupPlanningScene() method
   - Declare getCylinderPosition() method

6. **Create `mtc_scene_manager.cpp` implementation**
   - Move setupPlanningScene() from mtc_node.cpp
   - Move getCylinderPosition() from mtc_node.cpp

7. **Refactor `mtc_node.cpp`**
   - Reduce to ~200 lines
   - Focus only on: node setup, service callbacks, orchestration
   - Use the three library components

## Code Statistics (Target)

| Component | Lines | Purpose |
|-----------|-------|---------|
| mtc_planners.cpp | ~150 | Planner setup |
| mtc_task_stages.cpp | ~350 | Task creation & execution |
| mtc_scene_manager.cpp | ~100 | Scene management |
| mtc_node.cpp | ~150 | Main node + orchestration |
| **Total** | ~750 | **Modular and reusable** |

## Testing Strategy

After refactoring, each module can be tested independently:
- Unit test `mtc_planners` for correct planner configuration
- Unit test `mtc_scene_manager` for correct scene setup
- Integration test `mtc_task_stages` with mock planners
- Integration test `mtc_node` with all components
