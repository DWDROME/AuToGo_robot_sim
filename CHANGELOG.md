# Changelog

All notable changes to this project will be documented in this file.

## [v0.1.0] - 2026-03-08

This is the first public baseline release of `AuToGo_robot_sim`.

### Highlights

- established a clearer public project baseline for ROS1 Noetic and Gazebo 11
- rewrote the main repository README in English
- added maintainer-facing repository documents:
  - `CONTRIBUTING.md`
  - `SECURITY.md`
  - issue templates
  - pull request template
- clarified the 2D SLAM workflow and separated it from the Livox / Fast-LIO playpen profile

### Fixed

- fixed the default world path in `scout_mini_empty_world.launch`
- replaced machine-specific absolute image paths in bundled SLAM maps with portable relative paths
- fixed Livox Mid-360 plugin configuration so forwarded launch parameters are no longer silently overridden
- normalized `robot_namespace` handling across scout mini Gazebo launch entrypoints and skid-steer controller topics

### Documentation

- clarified that `scout_mini_house.launch` and `scout_mini_empty_world.launch` are the recommended entrypoints for 2D SLAM
- documented the expected Hokuyo topic for GMapping as `/hokuyo/scan`
- made the repository positioning, scope, roadmap, and limitations easier to understand for outside users

### Supported Baseline

- Ubuntu 20.04
- ROS Noetic
- Gazebo 11
- multi-scene Gazebo simulation
- EKF localization
- 2D SLAM with GMapping
- autonomous navigation with the ROS Navigation Stack
- Fast-LIO experiments with Livox Mid-360

### Known Limitations

- `doc/navigation.md` is still incomplete
- some detailed workflow notes are still in Chinese
- Fast-LIO setup may require environment-specific topic, TF, and workspace adjustments
- the repository currently targets ROS1 Noetic only
