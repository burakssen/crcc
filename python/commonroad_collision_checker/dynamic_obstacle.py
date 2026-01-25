import commonroad_collision_checker._core.dynamic_obstacle as core

# explicitly re-export classes to define the public API of this module
# this enables us to add wrappers for the Rust objects later as a non-breaking change
DynamicObstacle = core.DynamicObstacle
