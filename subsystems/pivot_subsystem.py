import rev
from commands2 import SubsystemBase


class PivotSubsystem(SubsystemBase):
    # Create a new VisionSubsystem

    def __init__(self) -> None:
        super().__init__()

        self.pivot_motor = rev.CANSparkFlex(3, rev.CANSparkFlex.MotorType.kBrushless)