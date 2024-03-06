import math

import commands2
import commands2.button

import wpimath
import wpilib

from wpimath.controller import PIDController, ProfiledPIDControllerRadians
from wpimath.geometry import Pose2d, Rotation2d, Translation2d
from wpimath.trajectory import TrajectoryConfig, TrajectoryGenerator

from commands.climber_r_down import ClimberRDown
from commands.climber_r_up import ClimberRUp
from commands.climbers_down import ClimbersDown
from commands.climbers_up import ClimbersUp
# from commands.drive_along_trajectory import DriveAlongTrajectory
from commands.intake_in import IntakeIn
from commands.intake_in_until_loaded import IntakeInUntilLoaded
from commands.intake_out import IntakeOut
from commands.pivot_down import PivotDown
from commands.pivot_up import PivotUp
from commands.pivot_to_position import PivotToPosition
from commands.shooter_amp import ShooterAmp
from commands.amp_up import AmpUp
from commands.shooter_spin_up import ShooterSpinUp
from commands.track_game_piece import TrackGamePiece
from commands.track_goal import TrackGoal
from constants.swerve_constants import AutoConstants, DriveConstants, OIConstants
from subsystems.climber_subsystem import ClimberSubsystem
from subsystems.intake_subsystem import IntakeSubsystem
from subsystems.drivesubsystem import DriveSubsystem
from subsystems.pivot_subsystem import PivotSubsystem
from subsystems.shooter_subsystem import ShooterSubsystem
from subsystems.vision_subsystem import VisionSubsystem
from subsystems.amp_subsystem import AmpSubsystem



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
        self.intakeSubsystem = IntakeSubsystem()
        self.shooterSubsystem = ShooterSubsystem()
        self.pivotSubsystem = PivotSubsystem()
        self.visionSubsystem = VisionSubsystem()
        self.climberSubsystem = ClimberSubsystem()
        self.ampSubsystem = AmpSubsystem()

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
                    False,
                    False,
                ),
                self.robotDrive,
            )
        )

    def configureButtonBindings(self) -> None:
        """
        Use this method to define your button->command mappings. Buttons can be created by
        instantiating a :GenericHID or one of its subclasses (Joystick or XboxController),
        and then passing it to a JoystickButton.
        """

        # Set wheels to X (brake)
        commands2.button.JoystickButton(self.driverController, 7).toggleOnTrue(
            commands2.RunCommand(
                lambda: self.robotDrive.setX(),
                self.robotDrive,
            )
        )

        # Slow mode
        commands2.button.JoystickButton(self.driverController, 6).toggleOnTrue(
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
                        self.driverController.getZ() * 0.5, OIConstants.kDriveTurnDeadband
                    ),
                    True,
                    False,
                ),
                self.robotDrive,
            )
        )
        # Toggle Speaker Vision Tracking
        commands2.button.JoystickButton(self.operatorController, 10).toggleOnTrue(
            TrackGoal(self.visionSubsystem, self.robotDrive, self.pivotSubsystem, self.driverController)
        )
        # Climber controls
        commands2.button.JoystickButton(self.operatorController, 1).whileTrue(
            ClimbersUp(self.climberSubsystem)
        )
        commands2.button.JoystickButton(self.operatorController, 2).whileTrue(
            ClimbersDown(self.climberSubsystem)
        )
        commands2.button.JoystickButton(self.operatorController, 3).whileTrue(
            ClimberRUp(self.climberSubsystem)
        )
        commands2.button.JoystickButton(self.operatorController, 4).whileTrue(
            ClimberRDown(self.climberSubsystem)
        )

        # Toggle Shooter
        commands2.button.JoystickButton(self.operatorController, 9).toggleOnTrue(
            ShooterSpinUp(self.shooterSubsystem)
        )
        # Shooter Amp
        commands2.button.JoystickButton(self.operatorController, 11).whileTrue(
            ShooterAmp(self.shooterSubsystem)
        )
        commands2.button.JoystickButton(self.operatorController, 11).whileTrue(
            IntakeIn(self.intakeSubsystem)
        )
        # Pivot up and down
        commands2.button.JoystickButton(self.operatorController, 5).whileTrue(
            PivotUp(self.pivotSubsystem) #7:45 : 0.41
        ) # Middle (Speaker)
        commands2.button.JoystickButton(self.operatorController, 6).whileTrue(
            PivotDown(self.pivotSubsystem)
        ) # High
        # # Low (Podium)
        commands2.button.JoystickButton(self.driverController, 8).onTrue(
            PivotToPosition(self.pivotSubsystem, 0.34) #7:45 : 0.41
        ) # Middle (Speaker)
        # commands2.button.JoystickButton(self.operatorController, 6).onTrue(
        #     PivotToPosition(self.pivotSubsystem, 0.4)
        # ) # High
        # commands2.button.JoystickButton(self.operatorController, 7).onTrue(
        #     PivotToPosition(self.pivotSubsystem, 0.45)
        # )

        # Intake in and out
        # commands2.button.JoystickButton(self.operatorController, 3).whileTrue(
        #     IntakeIn(self.intakeSubsystem)
        # )
        # commands2.button.JoystickButton(self.operatorController, 4).whileTrue(
        #     IntakeOut(self.intakeSubsystem)
        # )
        commands2.button.JoystickButton(self.operatorController, 12).onTrue(
            IntakeInUntilLoaded(self.intakeSubsystem)
        )
        commands2.button.JoystickButton(self.operatorController, 7).onTrue(
            TrackGamePiece(self.visionSubsystem, self.robotDrive, self.intakeSubsystem, self.driverController)
        )
        commands2.button.JoystickButton(self.operatorController, 13).toggleOnTrue(
            ShooterSpinUp(self.shooterSubsystem)
        )
        # Amp Commands
        commands2.button.JoystickButton(self.driverController, 8).whileTrue(
            AmpUp(self.ampSubsystem)
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
        return commands2.SequentialCommandGroup()

        # return swerveControllerCommand.andThen(
        #     lambda: self.robotDrive.drive(0, 0, 0, False, False)
        # )
