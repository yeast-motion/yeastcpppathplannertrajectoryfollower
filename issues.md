# yeastcpppathplannertrajectoryfollower - Issues

(Only analyzing the yeast wrapper: `src/pathplannertrajectoryfollower.cpp`, `src/extendedrobotconfig.cpp`, and `include/yeastcpppathplannertrajectoryfollower/`)

## Bugs / Correctness Problems

### 1. Acceleration calculation divides by `dt` in milliseconds, not seconds
In `src/pathplannertrajectoryfollower.cpp:250-273`, `dt` is computed in milliseconds via `std::chrono::duration_cast<std::chrono::milliseconds>`. Then on lines 271-273, acceleration is calculated as `(speed - measurement) / dt`. Since `dt` is in milliseconds but velocities are in m/s, the resulting acceleration is off by a factor of 1000 (too small by 1000x). Should use seconds or convert.

### 2. `dt` is `float` but result of `count()` is integer for `milliseconds`
`src/pathplannertrajectoryfollower.cpp:250`: `float dt = std::chrono::duration_cast<std::chrono::milliseconds>(...).count()`. The `count()` on `milliseconds` returns an integer type, so sub-millisecond precision is lost. Also, on the very first call, `dt` will be the entire duration since `begin()` was called, which could be large and meaningless.

### 3. `begin()` does not set `finished = false`
In `src/pathplannertrajectoryfollower.cpp:149-179`, the `begin(Trajectory, ...)` overload never sets `this->finished = false`. The two `begin_choreo()` overloads do (lines 211, 244). If `begin()` is called after a previous trajectory completed, `finished` remains `true` and `follow()` immediately reports the trajectory as finished.

### 4. Memory leak pattern with `unique_ptr`
`src/pathplannertrajectoryfollower.cpp:161-163`:
```cpp
this->follow_path_command.release();
this->follow_path_command.reset(nullptr);
this->follow_path_command.reset(new FollowPathCommand(...));
```
The `.release()` on line 161 releases ownership without deleting the object, leaking the previously held `FollowPathCommand`. The `.reset(nullptr)` on line 162 is then a no-op. Should just use `.reset(new ...)` directly -- a single `reset()` call deletes the old object and takes ownership of the new one. This pattern is repeated at lines 194-196 and 227-229.

### 5. `follow()` calls `Execute()` even after `IsFinished()` returns true
`src/pathplannertrajectoryfollower.cpp:253-258`: The code checks `IsFinished()`, sets `finished = true`, but then unconditionally calls `Execute()` on the next line. After a command reports finished, calling `Execute()` again may produce undefined behavior depending on the PathPlanner command implementation.

### 6. `to_json()` called repeatedly on `trajectory` in `path_from_trajectory()`
In `src/pathplannertrajectoryfollower.cpp:52-90`, `trajectory.to_json()` is called 9 separate times. Since `Trajectory::to_json()` returns a copy of the internal JSON, this creates 9 unnecessary copies. Should call it once and store the result.

### 7. `begin_choreo()` ignores `file_path` parameter
`src/pathplannertrajectoryfollower.cpp:182-183` and line 215: `(void) file_path;` explicitly suppresses the unused parameter. The caller provides a file path that is never used, which is confusing API design.

## Code Smells

### 8. `controller_from_config` and `config_from_json` are free functions but should probably be static methods or in a utility namespace
`src/pathplannertrajectoryfollower.cpp:111-137`: These are free functions in an anonymous translation unit that create shared state objects. They could be static methods on the class for better organization.

### 9. `register_named_commands` recursion can be infinite
`src/pathplannertrajectoryfollower.cpp:28-47`: The recursive traversal of JSON checks `is_object() || is_array()` and then iterates children. If the JSON has deeply nested or circular structure (unlikely but not validated), this could stack overflow.

### 10. No validation of `follower_config_json` or `config_json`
`set_config()` and `set_follower_config()` just store raw JSON without validation. If required keys like `TranslationPIDConstants`, `RotationPIDConstants`, or `Period` are missing, the error only surfaces later during `begin()` with an opaque nlohmann::json exception.
