import phoenix5
from commands2 import SubsystemBase


class RightClimberSubsystem(SubsystemBase):
    # Create a new VisionSubsystem

    def __init__(self) -> None:
        super().__init__()

        self.right_climber = phoenix5.TalonFX(8, "rio")

    def set_right_climber(self, speed):
        self.right_climber.set(phoenix5.ControlMode.PercentOutput, speed)
