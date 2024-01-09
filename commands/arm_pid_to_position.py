import typing
import commands2
import wpilib
import wpimath.controller

from subsystems.arm_subsystem import ArmSubsystem


class ArmPIDToPosition(commands2.PIDCommand):

    def __init__(self, arm_sub: ArmSubsystem, target_position: float) -> None:
        super().__init__(
            wpimath.controller.PIDController(7.5, 0, 0),
            # Close loop on absolute encoder
            arm_sub.get_position,
            # Set reference to target
            target_position,
            # Pipe output to turn arm
            lambda output: arm_sub.set_motor_speed(output),
            # Require the arm
            [arm_sub]
        )

    def isFinished(self) -> bool:
        return False
