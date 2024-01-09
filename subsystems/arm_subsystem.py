import commands2
import wpilib
import wpimath.controller
from commands2 import SubsystemBase

from misc_constants.arm_constants import ArmConstants


class ArmSubsystem(SubsystemBase):
    # Create a new ArmSubsystem

    def __init__(self) -> None:
        super().__init__()

        self.arm_motor = wpilib.Spark(0)
        self.arm_motor.setInverted(True)

        self.arm_encoder = wpilib.DutyCycleEncoder(channel=0)
        self.arm_encoder.setPositionOffset(ArmConstants.kArmOffset)
        self.arm_encoder.setDutyCycleRange(0, 1)

    def periodic(self) -> None:
        wpilib.SmartDashboard.putNumber("Arm Encoder Value", self.get_position())

    def get_position(self):
        return self.arm_encoder.getAbsolutePosition()

    def set_motor_speed(self, speed):
        self.arm_motor.set(speed)
