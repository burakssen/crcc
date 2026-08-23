import pytest
from crcc import CollisionBackend

BACKENDS = [CollisionBackend.Parry, CollisionBackend.Rhusics, CollisionBackend.Collide]


@pytest.fixture(params=BACKENDS, ids=["parry", "rhusics", "collide"])
def backend(request: pytest.FixtureRequest) -> CollisionBackend:
    return request.param
