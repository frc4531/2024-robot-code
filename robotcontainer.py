import math

import commands2
import commands2.button

import wpimath
import wpilib

from wpimath.controller import PIDController, ProfiledPIDControllerRadians
from wpimath.geometry import Pose2d, Rotation2d, Translation2d
from wpimath.trajectory import TrajectoryConfig, TrajectoryGenerator

from commands.arm_pid_to_position import ArmPIDToPosition
from commands.drive_along_trajectory import DriveAlongTrajectory
from commands.intake_in import IntakeIn
from commands.intake_out import IntakeOut
from commands.stop_arm_and_wrist import StopArmAndWrist
from commands.wrist_pid_to_position import WristPIDToPosition
from constants import AutoConstants, DriveConstants, OIConstants
from subsystems.arm_subsystem import ArmSubsystem
from subsystems.intake_subsystem import IntakeSubsystem
from subsystems.wrist_subsystem import WristSubsystem
from subsystems.drivesubsystem import DriveSubsystem


class RobotContainer:
    """
    This class is where the bulk of the robot should be declared. Since Command-based is a
    "declarative" paradigm, very little robot logic should actually be handled in the :class:`.Robot`
    periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
    subsystems, commands, and button mappings) should be declared here.
    """

    def __init__(self) -> None:
        # The robot's subsystems
        self.robotDrive = DriveSubsystem()
        self.armSubsystem = ArmSubsystem()
        self.wristSubsystem = WristSubsystem()
        self.intakeSubsystem = IntakeSubsystem()

        # The driver controllers
        self.driverController = wpilib.Joystick(OIConstants.kDriverControllerPort)
        self.operatorController = wpilib.Joystick(OIConstants.kOperatorControllerPort)

        # Configure the button bindings
        self.configureButtonBindings()

        # Configure default commands
        self.robotDrive.setDefaultCommand(
            # The left stick controls translation of the robot.
            # Turning is controlled by the X axis of the right stick.
            commands2.RunCommand(
                lambda: self.robotDrive.drive(
                    -wpimath.applyDeadband(
                        (self.driverController.getY() * math.sin(self.robotDrive.getHeading() * (math.pi / 180))) +
                        (self.driverController.getX() * math.cos(self.robotDrive.getHeading() * (math.pi / 180))),
                        OIConstants.kDriveDeadband
                    ),
                    -wpimath.applyDeadband(
                        (-self.driverController.getY() * math.cos(self.robotDrive.getHeading() * (math.pi / 180))) +
                        (self.driverController.getX() * math.sin(self.robotDrive.getHeading() * (math.pi / 180))),
                        OIConstants.kDriveDeadband
                    ),
                    -wpimath.applyDeadband(
                        self.driverController.getZ(), OIConstants.kDriveDeadband
                    ),
                    True,
                    False,
                ),
                [self.robotDrive],
            )
        )

    def configureButtonBindings(self) -> None:
        """
        Use this method to define your button->command mappings. Buttons can be created by
        instantiating a :GenericHID or one of its subclasses (Joystick or XboxController),
        and then passing it to a JoystickButton.
        """

        # Set wheels to X (brake)
        commands2.button.JoystickButton(self.driverController, 7).toggleWhenPressed(
            commands2.RunCommand(
                lambda: self.robotDrive.setX(),
                [self.robotDrive],
            )
        )

        # Slow mode
        commands2.button.JoystickButton(self.driverController, 6).toggleWhenPressed(
            commands2.RunCommand(
                lambda: self.robotDrive.drive(
                    -wpimath.applyDeadband(
                        ((self.driverController.getY() * math.sin(self.robotDrive.getHeading() * (math.pi / 180))) +
                        (self.driverController.getX() * math.cos(self.robotDrive.getHeading() * (math.pi / 180)))) * 0.5,
                        OIConstants.kDriveDeadband
                    ),
                    -wpimath.applyDeadband(
                        ((-self.driverController.getY() * math.cos(self.robotDrive.getHeading() * (math.pi / 180))) +
                        (self.driverController.getX() * math.sin(self.robotDrive.getHeading() * (math.pi / 180)))) * 0.5,
                        OIConstants.kDriveDeadband
                    ),
                    -wpimath.applyDeadband(
                        self.driverController.getZ() * 0.5, OIConstants.kDriveDeadband
                    ),
                    True,
                    False,
                ),
                [self.robotDrive],
            )
        )

        # Intake in and out
        commands2.button.JoystickButton(self.operatorController, 5).whileTrue(
            IntakeIn(self.intakeSubsystem)
        )
        commands2.button.JoystickButton(self.operatorController, 6).whileTrue(
            IntakeOut(self.intakeSubsystem)
        )

        # CUBES MID
        commands2.button.JoystickButton(self.operatorController, 1).whenPressed(
            WristPIDToPosition(self.wristSubsystem, 0.65)
        )
        commands2.button.JoystickButton(self.operatorController, 1).whenPressed(
            ArmPIDToPosition(self.armSubsystem, 0.80)
        )

        # SINGLE SUBSTATION PICKUP
        commands2.button.JoystickButton(self.operatorController, 3).whenPressed(
            WristPIDToPosition(self.wristSubsystem, 0.45)
        )
        commands2.button.JoystickButton(self.operatorController, 3).whenPressed(
            ArmPIDToPosition(self.armSubsystem, 0.88)
        )

        # Same as last one above, but on driver controller for endgame
        commands2.button.JoystickButton(self.driverController, 8).whenPressed(
            WristPIDToPosition(self.wristSubsystem, 0.45)
        )
        commands2.button.JoystickButton(self.driverController, 8).whenPressed(
            ArmPIDToPosition(self.armSubsystem, 0.88)
        )

        # IDLE STATE - ARM IS UP, WRIST IS DOWN FOR MOVEMENT
        commands2.button.JoystickButton(self.operatorController, 4).whenPressed(
            WristPIDToPosition(self.wristSubsystem, 0.79)
        )
        commands2.button.JoystickButton(self.operatorController, 4).whenPressed(
            ArmPIDToPosition(self.armSubsystem, 0.67)
        )
        # FlOOR PICKUP - CUBES
        commands2.button.JoystickButton(self.operatorController, 2).whenPressed(
            WristPIDToPosition(self.wristSubsystem, 0.675)
        )
        commands2.button.JoystickButton(self.operatorController, 2).whenPressed(
            ArmPIDToPosition(self.armSubsystem, 0.9)
        )

        # STOP ARM AND WRIST
        commands2.button.JoystickButton(self.operatorController, 13).toggleWhenPressed(
            StopArmAndWrist(self.armSubsystem, self.wristSubsystem)
        )

    def disablePIDSubsystems(self) -> None:
        """Disables all ProfiledPIDSubsystem and PIDSubsystem instances.
        This should be called on robot disable to prevent integral windup."""

    def getAutonomousCommand(self) -> commands2.Command:
        """Use this to pass the autonomous command to the main {@link Robot} class.

        :returns: the command to run in autonomous
        """
        # Create config for trajectory
        config = TrajectoryConfig(
            AutoConstants.kMaxSpeedMetersPerSecond,
            AutoConstants.kMaxAccelerationMetersPerSecondSquared,
        )
        # Add kinematics to ensure max speed is actually obeyed
        config.setKinematics(DriveConstants.kDriveKinematics)

        # An example trajectory to follow. All units in meters.
        if wpilib.Preferences.getBoolean("startingOnSides"):
            exampleTrajectory = TrajectoryGenerator.generateTrajectory(
                # Start at the origin facing the +X direction
                Pose2d(0, 0, Rotation2d(0)),
                # Pass through these two interior waypoints, making an 's' curve path
                [Translation2d(0, -1), Translation2d(0, -2)],
                # End 3 meters straight ahead of where we started, facing forward
                Pose2d(0, -4.5, Rotation2d(0)),
                config,
            )
        elif wpilib.Preferences.getBoolean("onRedAlliance"):
            exampleTrajectory = TrajectoryGenerator.generateTrajectory(
                # Start at the origin facing the +X direction
                Pose2d(0, 0, Rotation2d(0)),
                # Pass through these two interior waypoints, making an 's' curve path
                [Translation2d(0, -1), Translation2d(0, -2)],
                # End 3 meters straight ahead of where we started, facing forward
                Pose2d(0, -2.45, Rotation2d(0)),
                config,
            )

        else:
            exampleTrajectory = TrajectoryGenerator.generateTrajectory(
                # Start at the origin facing the +X direction
                Pose2d(0, 0, Rotation2d(0)),
                # Pass through these two interior waypoints, making an 's' curve path
                [Translation2d(1, -1), Translation2d(2, 1)],
                # End 3 meters straight ahead of where we started, facing forward
                Pose2d(3, 0, Rotation2d(0)),  # -2.35
                config,
            )

        thetaController = ProfiledPIDControllerRadians(
            AutoConstants.kPThetaController,
            0,
            0,
            AutoConstants.kThetaControllerConstraints,
        )
        thetaController.enableContinuousInput(-math.pi, math.pi)

        swerveControllerCommand = commands2.Swerve4ControllerCommand(
            exampleTrajectory,
            self.robotDrive.getPose,  # Functional interface to feed supplier
            DriveConstants.kDriveKinematics,
            # Position controllers
            PIDController(AutoConstants.kPXController, 0, 0),
            PIDController(AutoConstants.kPYController, 0, 0),
            thetaController,
            self.robotDrive.setModuleStates,
            [self.robotDrive],
        )

        # Reset odometry to the starting pose of the trajectory.
        self.robotDrive.resetOdometry(exampleTrajectory.initialPose())

        # Run path following command, then stop at the end.
        return commands2.SequentialCommandGroup(
            commands2.ParallelDeadlineGroup(
                commands2.WaitCommand(2),
                WristPIDToPosition(self.wristSubsystem, 0.79),
                ArmPIDToPosition(self.armSubsystem, 0.67)
            ),
            commands2.ParallelDeadlineGroup(
                commands2.WaitCommand(1),
                WristPIDToPosition(self.wristSubsystem, 0.79),
                ArmPIDToPosition(self.armSubsystem, 0.67),
                IntakeOut(self.intakeSubsystem)
            ),
            commands2.ParallelDeadlineGroup(
                swerveControllerCommand.andThen(
                    lambda: self.robotDrive.drive(0, 0, 0, False, False)
                ),
                WristPIDToPosition(self.wristSubsystem, 0.79),
                ArmPIDToPosition(self.armSubsystem, 0.67)
            ),
            commands2.ParallelDeadlineGroup(
                commands2.WaitCommand(2),
                WristPIDToPosition(self.wristSubsystem, 0.45),
                ArmPIDToPosition(self.armSubsystem, 0.84)
            )
        )
        # return swerveControllerCommand.andThen(
        #     lambda: self.robotDrive.drive(0, 0, 0, False, False)
        # )
