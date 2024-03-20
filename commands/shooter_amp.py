import commands2

from subsystems.shooter_subsystem import ShooterSubsystem


class ShooterAmp(commands2.CommandBase):

    def __init__(self, shooter_sub: ShooterSubsystem) -> None:
        super().__init__()

        self.shooter_sub = shooter_sub
        self.addRequirements(self.shooter_sub)

    def execute(self) -> None:
        self.shooter_sub.set_percentage_speed(-0.17, 0.17)  # 7:38, 0.2, 7:46 0.4

    def isFinished(self) -> bool:
        return False

    def end(self, interrupted: bool) -> None:
        self.shooter_sub.stop_shooter()
