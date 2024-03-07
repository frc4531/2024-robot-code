import math
import commands2
from wpimath._controls._controls.controller import PIDController, ProfiledPIDControllerRadians, HolonomicDriveController
from wpimath._controls._controls.trajectory import TrajectoryGenerator, TrajectoryConfig, Trajectory
from wpimath.geometry import Pose2d, Rotation2d, Translation2d

from constants.swerve_constants import AutoConstants, DriveConstants
from subsystems.drivesubsystem import DriveSubsystem


class DriveAlongTrajectory(commands2.SwerveControllerCommand):

    def __init__(self, drive_sub: DriveSubsystem) -> None:
        # Create config for trajectory
        config = TrajectoryConfig(
            AutoConstants.kMaxSpeedMetersPerSecond,
            AutoConstants.kMaxAccelerationMetersPerSecondSquared,
        )
        # Add kinematics to ensure max speed is actually obeyed
        config.setKinematics(DriveConstants.kDriveKinematics)

        self.trajectory = TrajectoryGenerator.generateTrajectory(
            # Start at the origin facing the +X direction
            Pose2d(0, 0, Rotation2d(0)),
            # Pass through these two interior waypoints, making an 's' curve path
            [Translation2d(1, -1), Translation2d(2, 1)],
            # End 3 meters straight ahead of where we started, facing forward
            Pose2d(3, 0, Rotation2d(0)),  # -2.35
            config,
        )

        theta_controller = ProfiledPIDControllerRadians(
            AutoConstants.kPThetaController,
            0,
            0,
            AutoConstants.kThetaControllerConstraints,
        )

        theta_controller.enableContinuousInput(-math.pi, math.pi)

        holonomic_controller = HolonomicDriveController(PIDController(AutoConstants.kPXController, 0, 0),
            PIDController(AutoConstants.kPYController, 0, 0),
            theta_controller)

        super().__init__(
            trajectory=self.trajectory,
            pose=drive_sub.getPose,  # Functional interface to feed supplier
            kinematics=DriveConstants.kDriveKinematics,
            # Position controllers
            controller=holonomic_controller,
            outputModuleStates=drive_sub.setModuleStates,
            requirements=drive_sub,
        )

        self.drive_sub = drive_sub
        self.addRequirements(self.drive_sub)

    def initialize(self) -> None:
        # Reset odometry to the starting pose of the trajectory.
        self.drive_sub.resetOdometry(self.trajectory.initialPose())
