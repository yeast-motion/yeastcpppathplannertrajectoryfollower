# Issues - trajectories

## Consistency & Correctness Problems

### 1. BLUE autos missing symmetry with RED autos
The following RED autos have no BLUE counterpart:
- `RED_OutpostSide_TrenchPartner.auto` -- no `BLUE_OutpostSide_TrenchPartner.auto` or `BLUE_DepotSide_TrenchPartner.auto` exists.

The autonomous_node code (`autos/__init__.py`) has `RED_OutpostSide_TrenchPartner` in the enum and map, but there is no BLUE equivalent autonomous routine or trajectory.

### 2. RED has paths with no BLUE counterpart and vice versa
Several paths exist for only one alliance, suggesting incomplete mirroring:
- `RED_OutpostBumpShoot_IntakeLoop.path` -- no BLUE equivalent
- `RED_OutpostPreClimb_Climb.path` -- no BLUE equivalent
- `RED_OutpostSide_Climb.path` -- no BLUE equivalent
- `RED_OutpostSideShoot_PreClimb.path` -- no BLUE equivalent
- `RED_OutpostSide_LemonBump.path` -- no BLUE equivalent
- `RED_OutpostSide_TrenchCycle2_Partner.path` -- no BLUE equivalent
- `BLUE_DepotSide_PostLine.path` -- no RED equivalent
- `BLUE_DepotSide_ShootAfterLine.path` -- no RED equivalent
- `BLUE_InDepot_DepotShoot.path` -- no RED equivalent
- `BLUE_OutpostSideCenterIntakeEnd_Shoot.path` -- no RED equivalent
- `BLUE_OutpostSide_Shoot.path` -- no RED equivalent
- `Blue_OutpostSide_Pickup.path` -- note inconsistent casing ("Blue" vs "BLUE")

### 3. Inconsistent naming convention
**File:** `paths/Blue_OutpostSide_Pickup.path`  
This file uses `Blue_` prefix while all other BLUE paths use `BLUE_`. This would cause lookup failures if code expects exact-case matching.

### 4. `settings.json` and `follower_settings.json` are in the same directory as autos
**Files:** `deploy/pathplanner/settings.json`, `deploy/pathplanner/follower_settings.json`  
These are in the parent `pathplanner/` directory, not in `paths/` or `autos/`. However, the `yeast_pathplannerlib_node` loads from `Parameters.trajectory_directory` which could point to a directory containing these files alongside paths. If it points to the parent `pathplanner/` directory, the JSON parser will attempt to parse these as trajectories (see yeast_pathplannerlib_node issues.md #2).

### 5. BLUE paths reference RED-specific strategies in some naming
Several BLUE paths reference positions/strategies that may not make sense for the BLUE alliance perspective:
- `BLUE_DepotTrenchShoot_HubIntake.path` vs `RED_DepotTrenchShoot_HubIntake.path` -- naming suggests these are mechanically mirrored but the actual coordinates should be verified.

### 6. No version pinning for PathPlanner format
**Files:** All `.auto` files have `"version": "2025.0"`  
The `.path` files may not have a version field. If the PathPlanner format changes, there's no mechanism to detect or handle version mismatches.

## Structural Issues

### 7. Paths and autos are in separate subdirectories but loaded from one directory
**Structure:** `deploy/pathplanner/paths/` and `deploy/pathplanner/autos/`  
The `yeast_pathplannerlib_node` appears to load from a single `trajectory_directory` parameter. The autos and paths need to be loaded from their respective directories, or the loading code needs to handle subdirectory traversal. This is a coordination issue between the trajectories layout and the loading code.
