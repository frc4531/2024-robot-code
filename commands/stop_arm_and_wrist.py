import typing
import commands2
import wpilib
import wpimath.controller

from subsystems.arm_subsystem import ArmSubsystem
from subsystems.wrist_subsystem import WristSubsystem


class StopArmAndWrist(commands2.CommandBase):

    def __init__(self, arm_sub: ArmSubsystem, wrist_sub: WristSubsystem) -> None:
        super().__init__()

        self.arm_sub = arm_sub
        self.wrist_sub = wrist_sub
        self.addRequirements([self.arm_sub, self.wrist_sub])

    def execute(self) -> None:
        self.arm_sub.arm_motor.set(0)
        self.wrist_sub.wrist_motor.set(0)

    def isFinished(self) -> bool:
        return False
