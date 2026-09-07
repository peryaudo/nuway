"""The import_light contract of docs/03 §6.1: nuway_ml.common imports without torch."""

import importlib
import subprocess
import sys

import pytest

MODULES = [
    "nuway_ml.common.geometry",
    "nuway_ml.common.frenet",
    "nuway_ml.common.carla_conv",
    "nuway_ml.common.occupancy",
    "nuway_ml.common.tick",
    "nuway_ml.common.qos",
    "nuway_ml.common.frames",
]


@pytest.mark.import_light
@pytest.mark.parametrize("module", MODULES)
def test_module_imports_without_torch_in_a_fresh_interpreter(module):
    code = (
        "import sys\n"
        f"import importlib; importlib.import_module({module!r})\n"
        "assert 'torch' not in sys.modules, 'torch was imported at module scope'\n"
    )
    subprocess.run([sys.executable, "-c", code], check=True)


@pytest.mark.import_light
def test_modules_import_in_process():
    for module in MODULES:
        importlib.import_module(module)
