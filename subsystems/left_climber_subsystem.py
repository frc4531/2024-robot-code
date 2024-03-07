import phoenix5
from commands2 import SubsystemBase


class LeftClimberSubsystem(SubsystemBase):
    # Create a new VisionSubsystem

    def __init__(self) -> None:
        super().__init__()

        self.left_climber = phoenix5.TalonFX(7, "rio")

    def set_left_climber(self, speed):
        self.left_climber.set(phoenix5.ControlMode.PercentOutput, speed)