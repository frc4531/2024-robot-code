import commands2

from subsystems.pivot_subsystem import PivotSubsystem


class PivotUp(commands2.CommandBase):

    def __init__(self, pivot_sub: PivotSubsystem) -> None:
        super().__init__()

        self.pivot_sub = pivot_sub
        self.addRequirements(self.pivot_sub)

    def execute(self) -> None:
        self.pivot_sub.set_pivot_speed(0.1)

    def isFinished(self) -> bool:
        return False

    def end(self, interrupted: bool) -> None:
        self.pivot_sub.set_pivot_speed(0)
