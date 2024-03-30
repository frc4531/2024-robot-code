import commands2
from wpilib import SmartDashboard

from subsystems.intake_subsystem import IntakeSubsystem


class IntakeIn(commands2.CommandBase):

    def __init__(self, intake_sub: IntakeSubsystem) -> None:
        super().__init__()

        self.intake_sub = intake_sub
        self.addRequirements(self.intake_sub)

    def execute(self) -> None:
        shooter_on = SmartDashboard.getBoolean("LED_ShooterActive", True)
        if not shooter_on:
            self.intake_sub.set_intake_speed(0)
        else:
            self.intake_sub.set_intake_speed(0.9)


    def isFinished(self) -> bool:
        return False

    def end(self, interrupted: bool) -> None:
        self.intake_sub.set_intake_speed(0)
