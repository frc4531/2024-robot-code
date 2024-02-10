import phoenix5
from commands2 import SubsystemBase


class ClimberSubsystem(SubsystemBase):
    # Create a new VisionSubsystem

    def __init__(self) -> None:
        super().__init__()

        self.left_climber = phoenix5.TalonFX(7,"rio")
        self.right_climber = phoenix5.TalonFX(8,"rio")

    def set_motors(self, left_speed,right_speed):
        self.left_climber.set(phoenix5.ControlMode.PercentOutput, left_speed)
        self.right_climber.set(phoenix5.ControlMode.PercentOutput, right_speed)