import rev
import wpilib
from commands2 import SubsystemBase

from constants.pivot_constants import PivotConstants


class PivotSubsystem(SubsystemBase):
    # Create a new VisionSubsystem

    def __init__(self) -> None:
        super().__init__()

        self.pivot_motor = rev.CANSparkFlex(3, rev.CANSparkFlex.MotorType.kBrushless)

        self.pivot_encoder = wpilib.DutyCycleEncoder(channel=0)
        self.arm_enconder.setPostionOffest(PivotConstants.kPivotOffset)
        self.arm_encoder.setDutyCycleRange(0,1)

    def periodic(self) -> None:
        wpilib.SmartDashboard.putNumber("Pivot Encoder Value", self.get_position())

    def get_position(self):
        return self.pivot_encoder.getAbsolutePosition()

    def set_pivot_speed(self, speed):
        self.pivot_motor.set(speed)