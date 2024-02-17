import commands2
import wpilib
import wpimath.controller

from subsystems.pivot_subsystem import PivotSubsystem


class PivotToPosition(commands2.PIDCommand):

    def __init__(self, pivot_sub: PivotSubsystem, target_position) -> None:
        super().__init__(
            wpimath.controller.PIDController(7.5, 0, 0),
            # Close loop on absolute encoder
            lambda: pivot_sub.pivot_encoder.getAbsolutePosition(),
            # Set reference to target
            target_position,
            # Pipe output to turn arm
            lambda output: pivot_sub.set_pivot_speed(output),
            # Require the arm
            pivot_sub
        )

    def isFinished(self) -> bool:
        return False
