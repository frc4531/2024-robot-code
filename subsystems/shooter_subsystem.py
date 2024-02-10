import rev
from commands2 import SubsystemBase


class ShooterSubsystem(SubsystemBase):
    # Create a new VisionSubsystem

    def __init__(self) -> None:
        super().__init__()

        self.left_shooter_motor = rev.CANSparkFlex(1, rev.CANSparkFlex.MotorType.kBrushless)
        self.right_shooter_motor = rev.CANSparkFlex(2, rev.CANSparkFlex.MotorType.kBrushless)


