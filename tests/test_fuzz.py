import random

from crcc.collision_checker import CollisionEngine
from crcc.collision_object import Circle, Compound, Rectangle, Triangle


def random_shape(rng):
    kind = rng.randrange(4)
    center = (rng.uniform(-5.0, 5.0), rng.uniform(-5.0, 5.0))
    if kind == 0:
        return Circle(rng.uniform(0.1, 2.0), center)
    if kind == 1:
        return Rectangle(rng.uniform(0.1, 4.0), rng.uniform(0.1, 4.0), rng.uniform(-3.14, 3.14), center)
    if kind == 2:
        x, y = center
        return Triangle((x, y), (x + rng.uniform(0.2, 3.0), y), (x, y + rng.uniform(0.2, 3.0)))
    return Compound([Circle(0.5, center), Rectangle(0.5, 0.5, center=(center[0] + 1.0, center[1]))])


def test_engines_match_for_seeded_random_shapes():
    rng = random.Random(20260604)

    for index in range(500):
        left = random_shape(rng)
        right = random_shape(rng)
        parry = left.collides(right, engine=CollisionEngine.Parry)
        rhusics = left.collides(right, engine=CollisionEngine.Rhusics)
        collide = left.collides(right, engine=CollisionEngine.Collide)
        assert parry == rhusics, index
        assert parry == collide, index
