import commands2

from subsystems.left_climber_subsystem import LeftClimberSubsystem


class ClimberLDown(commands2.CommandBase):

    def __init__(self, left_climber_sub: LeftClimberSubsystem) -> None:
        super().__init__()

        self.left_climber_sub = left_climber_sub
        self.addRequirements(self.left_climber_sub)

    def execute(self) -> None:
        self.left_climber_sub.set_left_climber(0.8)

    def isFinished(self) -> bool:
        return False

    def end(self, interrupted: bool) -> None:
        self.left_climber_sub.set_left_climber(0)
