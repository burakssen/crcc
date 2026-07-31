import pytest
from crcc import CollisionEngine

ENGINES = [CollisionEngine.Parry, CollisionEngine.Rhusics, CollisionEngine.Collide]


@pytest.fixture(params=ENGINES, ids=["parry", "rhusics", "collide"])
def engine(request: pytest.FixtureRequest) -> CollisionEngine:
    return request.param
