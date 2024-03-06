import rev
import wpilib
from commands2 import SubsystemBase


class AmpSubsystem(SubsystemBase):
    # Create a new AmpSubsystem

    def __init__(self) -> None:
        super().__init__()

        self.amp_motor = rev.CANSparkMax(5, rev.CANSparkMax.MotorType.kBrushless)
        self.amp_motor.setInverted(True)

    def set_amp_speed(self, speed):
        self.amp_motor.set(speed)
