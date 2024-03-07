from wpimath._controls._controls.trajectory import TrajectoryGenerator, TrajectoryConfig
from wpimath.geometry import Pose2d, Translation2d, Rotation2d

from constants.swerve_constants import AutoConstants


class AutoTrajectories:
    test_trajectory = TrajectoryGenerator.generateTrajectory(
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
            )
