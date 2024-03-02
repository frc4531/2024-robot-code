import commands2

from subsystems.climber_subsystem import ClimberSubsystem


class ClimbersUp(commands2.CommandBase):

    def __init__(self, climber_sub: ClimberSubsystem) -> None:
        super().__init__()

        self.climber_sub = climber_sub
        self.addRequirements(self.climber_sub)

    def execute(self) -> None:
        self.climber_sub.set_motors(-0.5, 0)

    def isFinished(self) -> bool:
        return False

    def end(self, interrupted: bool) -> None:
        self.climber_sub.set_motors(0, 0)
