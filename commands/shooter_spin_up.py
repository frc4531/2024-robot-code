import commands2

from subsystems.shooter_subsystem import ShooterSubsystem


class ShooterSpinUp(commands2.CommandBase):

    def __init__(self, shooter_sub: ShooterSubsystem) -> None:
        super().__init__()

        self.shooter_sub = shooter_sub
        self.addRequirements(self.shooter_sub)

    def execute(self) -> None:
        self.shooter_sub.set_velocities(-6000,6500)

    def isFinished(self) -> bool:
        return False

    def end(self, interrupted: bool) -> None:
        self.shooter_sub.stop_shooter()
