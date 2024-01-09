import commands2
import wpilib
import wpimath.controller
from commands2 import SubsystemBase

from misc_constants.arm_constants import ArmConstants
from misc_constants.wrist_constants import WristConstants


class WristSubsystem(SubsystemBase):
    # Create a new ArmSubsystem

    def __init__(self) -> None:
        super().__init__()

        self.wrist_motor = wpilib.Spark(1)
        self.wrist_motor.setInverted(True)

        self.wrist_encoder = wpilib.DutyCycleEncoder(channel=1)
        self.wrist_encoder.setPositionOffset(WristConstants.kWristOffset)
        self.wrist_encoder.setDutyCycleRange(0, 1)

    def periodic(self) -> None:
        wpilib.SmartDashboard.putNumber("Wrist Encoder Value", self.get_position())

    def get_position(self):
        return self.wrist_encoder.getAbsolutePosition()

    def set_motor_speed(self, speed):
        self.wrist_motor.set(speed)
