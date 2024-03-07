import math
import commands2
from wpimath._controls._controls.controller import PIDController, ProfiledPIDControllerRadians
from wpimath._controls._controls.trajectory import TrajectoryGenerator, TrajectoryConfig, Trajectory
from wpimath.geometry import Pose2d, Rotation2d, Translation2d

from constants.swerve_constants import AutoConstants, DriveConstants
from subsystems.drivesubsystem import DriveSubsystem


class DriveAlongTrajectory(commands2.Swerve4ControllerCommand):
    thetaController = ProfiledPIDControllerRadians(
        AutoConstants.kPThetaController,
        0,
        0,
        AutoConstants.kThetaControllerConstraints,
    )
    thetaController.enableContinuousInput(-math.pi, math.pi)

    def __init__(self, drive_sub: DriveSubsystem, trajectory: Trajectory) -> None:
        super().__init__(
            trajectory,
            drive_sub.getPose,
            DriveConstants.kDriveKinematics,
            PIDController(AutoConstants.kPXController, 0, 0),
            PIDController(AutoConstants.kPYController, 0, 0),
            self.thetaController,
            drive_sub.setModuleStates,
            [drive_sub],
        )

        self.drive_sub = drive_sub
        self.addRequirements([self.drive_sub])
