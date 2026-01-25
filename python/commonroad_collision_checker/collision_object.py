import commonroad_collision_checker._core.collision_object as core

# explicitly re-export classes to define the public API of this module
# this enables us to add wrappers for the Rust objects later as a non-breaking change
CollisionObject = core.CollisionObject
Compound = core.Compound
Circle = core.Circle
Empty = core.Empty
HalfSpace = core.HalfSpace
FullSpace = core.FullSpace
Polygon = core.Polygon
Rectangle = core.Rectangle
Triangle = core.Triangle
