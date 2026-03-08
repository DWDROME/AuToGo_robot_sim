# Contributing to AuToGo_robot_sim

Thank you for contributing.

This repository is maintained as a practical robotics simulation baseline, so contributions should prioritize reproducibility, clarity, and minimal breakage across existing ROS Noetic workflows.

## Before You Open an Issue

Please check:

- whether the problem can be reproduced on Ubuntu 20.04 with ROS Noetic
- whether required dependencies were installed with `rosdep`
- whether the issue is specific to one launch file, world, sensor, or algorithm workflow

When reporting a problem, include:

- operating system and ROS version
- launch file name
- exact command used
- terminal logs or screenshots
- relevant ROS topics, TF frames, or bag files if available

## Before You Open a Pull Request

Please keep changes focused and explain:

- what problem is being solved
- which package or workflow is affected
- how the change was tested
- whether documentation also needs to be updated

## Suggested Contribution Areas

- documentation cleanup
- launch file fixes
- sensor configuration improvements
- Fast-LIO integration notes
- reproducibility improvements for SLAM and navigation workflows

## Style Expectations

- prefer small, focused pull requests
- avoid unrelated refactors
- preserve existing launch behavior unless the change explicitly updates it
- update relevant documentation when changing commands, topics, or expected workflows

## Validation

At minimum, please confirm the affected workflow still launches and behaves as expected in your local environment.

If your change touches documentation only, make sure commands and file paths are still correct.
