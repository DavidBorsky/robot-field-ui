"""Camera abstraction layer.

The robot is expected to use a Logitech camera connected to the Raspberry Pi.
This file provides a simulated camera for development now and a placeholder
hardware camera wrapper for later.
"""

try:
    from typing import Protocol
except ImportError:  # pragma: no cover - Python < 3.8 fallback
    class Protocol(object):
        pass


class CameraFrame:
    def __init__(self, frame_id, width, height, source):
        self.frame_id = frame_id
        self.width = width
        self.height = height
        self.source = source


class VisionTarget:
    def __init__(
        self,
        visible,
        x_offset_norm=0.0,
        y_offset_norm=0.0,
        confidence=0.0,
        label="",
    ):
        self.visible = visible
        self.x_offset_norm = x_offset_norm
        self.y_offset_norm = y_offset_norm
        self.confidence = confidence
        self.label = label


class Camera(Protocol):
    def start(self) -> None: ...
    def stop(self) -> None: ...
    def read_frame(self) -> CameraFrame: ...
    def get_target(self) -> VisionTarget: ...


class SimulatedCamera:
    """Simple development camera that can pretend a target is visible."""

    def __init__(self):
        self.running = False
        self.frame_id = 0
        self.target = VisionTarget(visible=False)

    def start(self) -> None:
        self.running = True

    def stop(self) -> None:
        self.running = False

    def set_target(self, target: VisionTarget) -> None:
        self.target = target

    def read_frame(self) -> CameraFrame:
        if not self.running:
            raise RuntimeError("SimulatedCamera is not running")
        self.frame_id += 1
        return CameraFrame(
            frame_id=self.frame_id,
            width=640,
            height=480,
            source="simulated-logitech",
        )

    def get_target(self) -> VisionTarget:
        return self.target


class HardwareCamera:
    """Placeholder for the real Logitech camera implementation."""

    def __init__(self, index: int = 0):
        self.index = index
        self.running = False

    def start(self) -> None:
        self.running = True

    def stop(self) -> None:
        self.running = False

    def read_frame(self) -> CameraFrame:
        if not self.running:
            raise RuntimeError("HardwareCamera is not running")
        return CameraFrame(frame_id=0, width=640, height=480, source="camera-{}".format(self.index))

    def get_target(self) -> VisionTarget:
        return VisionTarget(visible=False)


def create_camera(simulate: bool = True) -> Camera:
    if simulate:
        return SimulatedCamera()
    return HardwareCamera()
