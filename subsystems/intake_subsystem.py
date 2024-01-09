import phoenix6
from commands2 import SubsystemBase


class IntakeSubsystem(SubsystemBase):
    # Create a new ArmSubsystem

    def __init__(self) -> None:
        super().__init__()

        self.intake_motor = phoenix6.hardware.TalonFX(1, "rio")
        self.intake_control = phoenix6.controls.DutyCycleOut(0)
        # self.intake_motor = ctre.WPI_TalonFX(1)

    def set_motor_speed(self, speed):
        self.intake_motor.set_control(self.intake_control.with_output(speed))
