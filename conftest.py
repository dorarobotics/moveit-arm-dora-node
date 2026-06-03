"""Shared pytest config — keeps unit tests independent of a real dora install."""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))
