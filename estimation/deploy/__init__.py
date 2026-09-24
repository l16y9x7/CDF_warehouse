"""Cosmetics Sort Deployment Package with layered architecture.

Layers:
- core: configuration, schemas, codecs, and module loading
- perception: SAM3 client, coordinate frames, and geometry cache
- pipeline: target pipeline, box selection, and SKU candidate arbitration
- algorithms: category geometric fitting (bottle, box, tube)
"""
import sys
from pathlib import Path

_DEPLOY_ROOT = Path(__file__).resolve().parent
for _subdir in ('core', 'perception', 'pipeline', 'algorithms'):
    _p = str(_DEPLOY_ROOT / _subdir)
    if _p not in sys.path:
        sys.path.insert(0, _p)
if str(_DEPLOY_ROOT) not in sys.path:
    sys.path.insert(0, str(_DEPLOY_ROOT))

# Backward-compatible re-exports
from .core import *
from .perception import *
from .pipeline import *
from .algorithms import *
