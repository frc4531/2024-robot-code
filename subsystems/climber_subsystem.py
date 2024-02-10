import phoenix6
from commands2 import SubsystemBase


class ClimberSubsystem(SubsystemBase):
    # Create a new VisionSubsystem

    def __init__(self) -> None:
        super().__init__()

        self.left_climber = phoenix6.hardware.TalonFX(7,"rio")
        self.left_climber_control = phoenix6.controls.DutyCycleOut(0)

        self.right_climber = phoenix6.hardware.TalonFX(8,"rio")
        self.right_climber_control = phoenix6.controls.DutyCycleOut(0)