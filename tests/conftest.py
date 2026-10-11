"""Shared test setup: make tools/ importable and give tests the repo root."""
import os
import sys

import pytest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
TOOLS = os.path.join(ROOT, "tools")
if TOOLS not in sys.path:
    sys.path.insert(0, TOOLS)


@pytest.fixture
def repo_root():
    return ROOT


@pytest.fixture
def config_dir():
    return os.path.join(ROOT, "src", "my_robot", "config")
