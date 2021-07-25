# Marionette

A library for motion planning, primarily intended for robotics, written in pure rust.

## Goals

- Build a general-purpose motion-planning library for robotics.
- Provide a library, not a "system" or a "framework", that can be used simply by adding a Cargo dependency.
- Minimize dependencies to things such as message brokers (such as the ROS master), while allowing for their use when needed.
- Maximize flexibility by breaking problems down into their most basic elements, and abstracting these away behind interfaces using Rust's traits. Any component of the library should be relatively easy to replace by one provided by user code.
- Allow the user to use as much or as little of the library as desired.
- Provide sensible defaults, rather than requiring enormous masses of (generated) configuration files and "wizards".
- Explore the power of Rust to make the above goals easier to achieve.