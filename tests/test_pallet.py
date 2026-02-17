"""Unit tests for pallet boundary validation using dummy modules."""

import sys
import types

class DummySE3:
    def __init__(self, x=0, y=0, z=0):
        self.x = x
        self.y = y
        self.z = z

    def __mul__(self, other):
        if isinstance(other, DummySE3):
            return DummySE3(self.x + other.x, self.y + other.y, self.z + other.z)
        return self

class DummySO3:
    pass

sys.modules['spatialmath'] = types.SimpleNamespace(SE3=DummySE3, SO3=DummySO3)
sys.modules['numpy'] = types.SimpleNamespace()

import importlib.util
from pathlib import Path

spec = importlib.util.spec_from_file_location(
    "pallet", Path(__file__).resolve().parent.parent / "IK" / "pallet.py"
)
pallet = importlib.util.module_from_spec(spec)
spec.loader.exec_module(pallet)

Pallet = pallet.Pallet
Brick = pallet.Brick


def test_is_slot_valid_edges():
    rows, cols, layers = 2, 2, 2
    brick = Brick(0.1, 0.2, 0.06)
    pallet = Pallet(rows, cols, layers, brick)
    assert pallet.is_slot_valid(0, 0, 0)
    assert pallet.is_slot_valid(rows - 1, cols - 1, layers - 1)
