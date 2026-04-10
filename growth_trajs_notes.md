# Growth Opportunities - trajectories

## Completeness

### 1. Mirror all RED autos to BLUE (or vice versa)
Multiple RED-only paths exist (climb paths, lemon bump, trench partner). Either create BLUE equivalents or document why they are one-alliance-only.

### 2. Fix inconsistent casing on `Blue_OutpostSide_Pickup.path`
**File:** `paths/Blue_OutpostSide_Pickup.path`  
Rename to `BLUE_OutpostSide_Pickup.path` to match the convention used by all other BLUE paths.

## Organization

### 3. Add a manifest or index file
Currently the relationship between autos and their constituent paths is only defined inside the `.auto` JSON files. A human-readable manifest listing each auto and its paths (in order) would help operators understand what each auto does without opening JSON files.

### 4. Remove unused/orphaned paths
Several paths may not be referenced by any `.auto` file (e.g., `Climb.path`, `Bump.path`, individual `driveLeft*.path` and `driveRight*.path` files). An audit script that cross-references `.auto` files against `.path` files would identify dead paths.

### 5. Organize paths into subdirectories by alliance
Currently all paths are in a flat `paths/` directory with 90+ files. Organizing into `paths/BLUE/`, `paths/RED/`, and `paths/shared/` would improve navigability.

## Validation

### 6. Add a CI validation script
A script that:
- Verifies every path referenced in `.auto` files exists in the `paths/` directory
- Checks naming convention consistency (all-caps alliance prefix)
- Verifies RED/BLUE auto symmetry
- Validates JSON schema of all files
would catch issues before deployment.

### 7. Verify PathPlanner settings match robot parameters
**File:** `settings.json`  
The settings define robot dimensions (`robotWidth: 0.889`, `robotTrackwidth: 0.53975`, module positions, etc.) that must match the swerve_drivetrain_node parameters. Currently there's no automated check for consistency between `settings.json` and the swerve drivetrain's YAML config.

### 8. Add follower PID tuning documentation
**File:** `follower_settings.json`  
The PID constants (`TranslationP: 5.0`, `RotationP: 6.0`, all I and D terms are 0) have no documentation about how they were tuned or what units/scale they operate at. Recording tuning history or rationale alongside the values would help future tuning.
