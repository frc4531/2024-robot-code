import typing
import commands2
import wpilib
import wpimath.controller

from subsystems.wrist_subsystem import WristSubsystem


class WristPIDToPosition(commands2.PIDCommand):

    def __init__(self, wrist_sub: WristSubsystem, target_position: float) -> None:
        super().__init__(
            wpimath.controller.PIDController(10, 0, 0),
            # Close loop on absolute encoder
            wrist_sub.get_position,
            # Set reference to target
            target_position,
            # Pipe output to turn arm
            lambda output: wrist_sub.set_motor_speed(output),
            # Require the arm
            [wrist_sub]
        )

    def isFinished(self) -> bool:
        return False
