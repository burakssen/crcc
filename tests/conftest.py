import pytest
from crcc.collision_checker import CollisionEngine

ENGINES = [CollisionEngine.Parry, CollisionEngine.Rhusics]


@pytest.fixture(params=ENGINES, ids=["parry", "rhusics"])
def engine(request):
    return request.param
