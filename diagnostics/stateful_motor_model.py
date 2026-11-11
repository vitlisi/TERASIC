"""Compatibility imports for the frozen Robot01 Stribeck physics. No fitting."""
import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src_python'))
from stribeck_motor import configure, friction_parameters, update
