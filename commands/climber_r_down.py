import commands2

from subsystems.right_climber_subsystem import RightClimberSubsystem


class ClimberRDown(commands2.CommandBase):

    def __init__(self, right_climber_sub: RightClimberSubsystem) -> None:
        super().__init__()

        self.right_climber_sub = right_climber_sub
        self.addRequirements(self.right_climber_sub)

    def execute(self) -> None:
        self.right_climber_sub.set_right_climber(0.8)

    def isFinished(self) -> bool:
        return False

    def end(self, interrupted: bool) -> None:
        self.right_climber_sub.set_right_climber(0)
