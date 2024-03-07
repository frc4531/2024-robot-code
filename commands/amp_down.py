import commands2

from subsystems.amp_subsystem import AmpSubsystem


class AmpDown(commands2.CommandBase):

    def __init__(self, amp_sub: AmpSubsystem) -> None:
        super().__init__()

        self.amp_sub = amp_sub
        self.addRequirements(self.amp_sub)

    def execute(self) -> None:
        self.amp_sub.set_amp_speed(-0.1)

    def isFinished(self) -> bool:
        return False

    def end(self, interrupted: bool) -> None:
        self.amp_sub.set_amp_speed(0)
