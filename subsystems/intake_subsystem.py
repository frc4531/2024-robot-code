import rev
from commands2 import SubsystemBase


class IntakeSubsystem(SubsystemBase):
    # Create a new ArmSubsystem

    def __init__(self) -> None:
        super().__init__()

        self.intake_motor = rev.CANSparkMax(4, rev.CANSparkMax.MotorType.kBrushless)

    def set_intake_speed(self, speed):
        self.intake_motor.set(speed)
