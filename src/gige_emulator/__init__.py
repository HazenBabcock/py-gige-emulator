"""
Pure Python GigE Vision camera emulator.

Subclass EmulatedCamera, implement next_frame(), and hand it to
GigECameraServer. Any GigE Vision client on the network then sees a camera.
"""

from .camera import EmulatedCamera, Frame
from .features import (CommandFeature, EnumFeature, FeatureError, FeatureSet,
                       FloatFeature, IntFeature, StringFeature)
from .server import GigECameraServer

__version__ = "0.1.0"

__all__ = [
    "EmulatedCamera",
    "Frame",
    "GigECameraServer",
    "IntFeature",
    "FloatFeature",
    "EnumFeature",
    "CommandFeature",
    "StringFeature",
    "FeatureSet",
    "FeatureError",
]
