import random

from crcc import Circle, CollisionEngine, Compound, Pose, Rectangle, Triangle


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
    seed = 20260604
    rng = random.Random(seed)

    for index in range(500):
        left = random_shape(rng)
        right = random_shape(rng)
        parry = left.collides(right, engine=CollisionEngine.Parry)
        rhusics = left.collides(right, engine=CollisionEngine.Rhusics)
        collide = left.collides(right, engine=CollisionEngine.Collide)
        context = (
            f"seed={seed} case={index} left={type(left).__name__} right={type(right).__name__} "
            f"parry={parry} rhusics={rhusics} collide={collide}"
        )
        assert parry == rhusics, context
        assert parry == collide, context


def random_compound_parts(rng):
    center = (rng.uniform(-5.0, 5.0), rng.uniform(-5.0, 5.0))
    return [
        Circle(rng.uniform(0.1, 1.2), center),
        Rectangle(
            rng.uniform(0.2, 2.5),
            rng.uniform(0.2, 2.0),
            rng.uniform(-3.14, 3.14),
            (center[0] + rng.uniform(-1.5, 1.5), center[1] + rng.uniform(-1.5, 1.5)),
        ),
        Triangle(
            (center[0] + rng.uniform(-1.5, 1.5), center[1] + rng.uniform(-1.5, 1.5)),
            (center[0] + rng.uniform(0.2, 1.8), center[1] + rng.uniform(-0.2, 0.2)),
            (center[0] + rng.uniform(-0.2, 0.2), center[1] + rng.uniform(0.2, 1.8)),
        ),
    ]


def random_pose(rng):
    return Pose(
        (rng.uniform(-3.0, 3.0), rng.uniform(-3.0, 3.0)),
        rng.uniform(-3.14, 3.14),
    )


def test_compounds_match_expanded_children_for_seeded_random_shapes():
    seed = 20260701
    rng = random.Random(seed)

    for index in range(300):
        left_parts = random_compound_parts(rng)
        right_parts = random_compound_parts(rng)
        left = Compound(left_parts)
        right = Compound(right_parts)
        pos_left = random_pose(rng)
        pos_right = random_pose(rng)

        for engine in [CollisionEngine.Parry, CollisionEngine.Rhusics, CollisionEngine.Collide]:
            compound = left.collides(right, pos_self=pos_left, pos_other=pos_right, engine=engine)
            expanded = any(
                left_part.collides(right_part, pos_self=pos_left, pos_other=pos_right, engine=engine)
                for left_part in left_parts
                for right_part in right_parts
            )
            assert compound == expanded, (
                f"seed={seed} case={index} engine={engine} pos_left={pos_left} pos_right={pos_right} "
                f"compound={compound} expanded={expanded}"
            )
