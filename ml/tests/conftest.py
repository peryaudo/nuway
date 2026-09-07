from pathlib import Path

import pytest


@pytest.fixture
def repo_root_ml() -> Path:
    return Path(__file__).resolve().parents[2]
