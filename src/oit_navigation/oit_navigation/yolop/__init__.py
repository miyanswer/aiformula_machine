"""
YOLOP Model and Inference Library embedded directly in oit_navigation.
"""

import sys
from pathlib import Path

# Add this directory's parent or local lib to sys.path so internal 'lib.models' / 'lib.config' resolve seamlessly
_YOLOP_DIR = Path(__file__).resolve().parent

if str(_YOLOP_DIR) not in sys.path:
    sys.path.insert(0, str(_YOLOP_DIR))

try:
    from .config.default import _C as cfg
    from .config.default import update_config
    from .models.YOLOP import get_net
    from .utils.augmentations import letterbox_for_img
    from .utils.utils import select_device
except ImportError:
    from config.default import _C as cfg
    from config.default import update_config
    from models.YOLOP import get_net
    from utils.augmentations import letterbox_for_img
    from utils.utils import select_device

__all__ = [
    "cfg",
    "update_config",
    "get_net",
    "letterbox_for_img",
    "select_device",
]
