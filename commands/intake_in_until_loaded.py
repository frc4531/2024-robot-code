import commands2

from subsystems.intake_subsystem import IntakeSubsystem


class IntakeInUntilLoaded(commands2.CommandBase):

    def __init__(self, intake_sub: IntakeSubsystem) -> None:
        super().__init__()

        self.intake_sub = intake_sub
        self.addRequirements(self.intake_sub)

        self.proxim_has_been_false = False

    def initialize(self) -> None:
        self.proxim_has_been_false = False

    def execute(self) -> None:
        self.intake_sub.set_intake_speed(0.7)

        if self.intake_sub.intake_prox.get():
            self.proxim_has_been_false = True

    def isFinished(self) -> bool:
        return not self.intake_sub.intake_prox.get() and self.proxim_has_been_false

    def end(self, interrupted: bool) -> None:
        self.intake_sub.set_intake_speed(0)
