# yeastcpppathplannertrajectoryfollower - Growth Opportunities

(Only analyzing the yeast wrapper: `src/pathplannertrajectoryfollower.cpp`, `src/extendedrobotconfig.cpp`, and `include/yeastcpppathplannertrajectoryfollower/`)

## Improvement Opportunities

### 1. Fix the units on acceleration computation
`src/pathplannertrajectoryfollower.cpp:250,271-273`: Convert `dt` to seconds before dividing. Use `std::chrono::duration<double>` instead of `milliseconds` to get floating-point seconds directly.

### 2. Cache `trajectory.to_json()` result in `path_from_trajectory()`
The method calls `trajectory.to_json()` 9 times (lines 52-90). Store the result in a local variable to avoid 9 unnecessary JSON copies.

### 3. Add config validation in `set_config()` / `set_follower_config()`
Validate that required keys exist at configuration time, producing clear error messages, rather than deferring to runtime crashes during `begin()`.

### 4. Remove or use the `file_path` parameter in `begin_choreo()`
Either pass `file_path` to `PathPlannerPath::fromChoreoTrajectory()` if it supports it, or remove the parameter from the API to avoid confusion.

### 5. Add support for new motor types in `ExtendedRobotConfig`
`src/extendedrobotconfig.cpp:52-73`: The motor lookup is a series of if/else-if chains. New motors (e.g., Kraken X44) would need manual additions. Consider a `std::unordered_map` for extensibility.

## Architectural Enhancements

### 6. Decouple PathPlanner initialization from `begin()`
Currently `begin()` creates a new controller and follow command each time. If the config does not change between trajectories, the controller could be reused.

### 7. Use `std::chrono::steady_clock` instead of `system_clock`
`src/pathplannertrajectoryfollower.cpp:178,210,243`: `system_clock` can jump (NTP adjustments, daylight saving). `steady_clock` is monotonic and appropriate for measuring elapsed time in control loops.

### 8. Expose trajectory metadata
`get_path_poses()` returns poses but no timing, velocity targets, or constraint info. Exposing the full trajectory state would enable better visualization and debugging.

## Testing Gaps

### 9. No unit tests
Critical untested paths:
- `path_from_trajectory()` with various trajectory JSON structures
- `follow()` acceleration calculation (the units bug)
- `begin()` -> `follow()` -> `status().finished` lifecycle
- `ExtendedRobotConfig::from_json()` with all motor type strings
- `ExtendedRobotConfig::from_json()` with missing keys
- Memory leak from `release()` + `reset()` pattern
- Named command registration and callback firing
