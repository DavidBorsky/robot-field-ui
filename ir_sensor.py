"""IR edge protection helpers.

These sensors hang over the left and right edges of the robot and look for the
dark carpet boundary so the robot can avoid falling off the playing surface.
"""

from drivetrain import FrontBackDrive, MotorCommand


class IRSensorState:
    def __init__(self, left_edge_detected=False, right_edge_detected=False):
        self.left_edge_detected = left_edge_detected
        self.right_edge_detected = right_edge_detected

    @property
    def any_edge_detected(self) -> bool:
        return self.left_edge_detected or self.right_edge_detected

    @property
    def both_edges_detected(self) -> bool:
        return self.left_edge_detected and self.right_edge_detected


class EdgeCorrection:
    def __init__(self, command, edge_override_active, detail):
        self.command = command
        self.edge_override_active = edge_override_active
        self.detail = detail


class EdgeSafetyController:
    """Overrides path-following commands when the robot nears the carpet edge.

    Behavior:
    - left edge detected  -> strafe right to get back on carpet
    - right edge detected -> strafe left to get back on carpet
    - both edges detected -> stop and back away a little
    - no edge detected    -> keep the original command
    """

    def __init__(self, strafe_strength: float = 0.45, reverse_strength: float = 0.25):
        self.strafe_strength = max(0.0, min(1.0, strafe_strength))
        self.reverse_strength = max(0.0, min(1.0, reverse_strength))
        self.drive = FrontBackDrive()

    def apply(self, requested: MotorCommand, sensors: IRSensorState) -> EdgeCorrection:
        if sensors.both_edges_detected:
            command = self.drive.mix(forward=-self.reverse_strength, strafe=0.0)
            return EdgeCorrection(
                command=command,
                edge_override_active=True,
                detail="Both edge sensors detected boundary: reversing to safety.",
            )

        if sensors.left_edge_detected:
            command = self.drive.mix(forward=0.0, strafe=-self.strafe_strength)
            return EdgeCorrection(
                command=command,
                edge_override_active=True,
                detail="Left edge sensor detected boundary: strafing right.",
            )

        if sensors.right_edge_detected:
            command = self.drive.mix(forward=0.0, strafe=self.strafe_strength)
            return EdgeCorrection(
                command=command,
                edge_override_active=True,
                detail="Right edge sensor detected boundary: strafing left.",
            )

        return EdgeCorrection(
            command=requested,
            edge_override_active=False,
            detail="No edge detected: following path normally.",
        )
