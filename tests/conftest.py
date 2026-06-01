import pytest
from crcc.collision_checker import CollisionEngine

ENGINES = [CollisionEngine.Parry, CollisionEngine.Rhusics, CollisionEngine.Collide]


@pytest.fixture(params=ENGINES, ids=["parry", "rhusics", "collide"])
def engine(request):
    return request.param
