import math
import typing
import commands2
import wpilib
import wpimath.controller
from wpimath._controls._controls.controller import PIDController, ProfiledPIDControllerRadians
from wpimath._controls._controls.trajectory import TrajectoryGenerator, TrajectoryConfig
from wpimath.geometry import Pose2d, Rotation2d, Translation2d

from constants import AutoConstants, DriveConstants
from subsystems.drivesubsystem import DriveSubsystem


class DriveAlongTrajectory(commands2.Swerve4ControllerCommand):
    thetaController = ProfiledPIDControllerRadians(
        AutoConstants.kPThetaController,
        0,
        0,
        AutoConstants.kThetaControllerConstraints,
    )
    thetaController.enableContinuousInput(-math.pi, math.pi)

    def __init__(self, drive_sub: DriveSubsystem) -> None:
        super().__init__(
            TrajectoryGenerator.generateTrajectory(
                # Start at the origin facing the +X direction
                Pose2d(0, 0, Rotation2d(0)),
                # Pass through these two interior waypoints, making an 's' curve path
                [Translation2d(1, 0), Translation2d(2, 0)],
                # End 3 meters straight ahead of where we started, facing forward
                Pose2d(2.5, 0, Rotation2d(0)),
                TrajectoryConfig(
                    AutoConstants.kMaxSpeedMetersPerSecond,
                    AutoConstants.kMaxAccelerationMetersPerSecondSquared,
                ),
            ),
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
